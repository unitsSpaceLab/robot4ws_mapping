#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <cmath>

#include <robot4ws_mapping/utilities.hpp>

#include <robot4ws_msgs/Dynamixel_parameters1.h>
#include <robot4ws_msgs/SlipUpdate.h>
#include <robot4ws_mapping/get_surface_normal.h>
#include <robot4ws_msgs/Vector3Array.h>

#include <opencv2/opencv.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <robot4ws_msgs/SlipUpdateRawData.h>
#include <robot4ws_msgs/SlipUpdateRawDataArray.h>
#include <std_msgs/Header.h>

#include <functional>
#include <map>


//worflow: wheel position (x,y) ---> descritize to cell ---> get grid index of the cell (only when needed) --> publish the cell index

// PolarPoint struct now include also ground truth velocity and wheel load
struct PolarPoint {
    std::pair<float,float> value;  // slip ratio and slip angle
    double angle;                  // approach angle (beta_c)
    double velocity_commanded;     // rho_c_i
    double steer_angle;            // delta_c_i
    double wheel_load;             // F_n_i (wheel load)
    double vxg_sim;                // x component of v_sim in ground frame
    double vyg_sim;                // y component of v_sim in ground frame
};


class SlipNode
{
public:
    // Position-based hash function for grid map positions
    struct PositionHash {
        double cell_size_;

        // Default constructor needed for unordered_map
        PositionHash() : cell_size_(0.2) {} // Use default cell size
        PositionHash(double cs) : cell_size_(cs) {}

        std::size_t operator()(const grid_map::Position& pos) const {
            // Discretize to cell centers to handle floating point precision
            int x_cell = static_cast<int>(std::round(pos.x() / cell_size_));
            int y_cell = static_cast<int>(std::round(pos.y() / cell_size_));
            return std::hash<int>()(x_cell) ^ (std::hash<int>()(y_cell) << 1);
        }
    };

    // Position equality comparison for grid map positions
    struct PositionEqual {
        double cell_size_;

        // Default constructor needed for unordered_map
        PositionEqual() : cell_size_(0.2) {} // Use default cell size
        PositionEqual(double cs) : cell_size_(cs) {}

        bool operator()(const grid_map::Position& a, const grid_map::Position& b) const {
            int ax = static_cast<int>(std::round(a.x() / cell_size_));
            int ay = static_cast<int>(std::round(a.y() / cell_size_));
            int bx = static_cast<int>(std::round(b.x() / cell_size_));
            int by = static_cast<int>(std::round(b.y() / cell_size_));
            return ax == bx && ay == by;
        }
    };

    SlipNode()
    {
        //TODO: aggiungere wheel number
        load_params();

        // Initialize position-based map after loading cell_size parameter
        // we use here the bucket constructor to initialize with custom hash and equal functors
        position2slips_map = std::unordered_map<grid_map::Position, std::vector<PolarPoint>, PositionHash, PositionEqual>(
            10, PositionHash(cell_size), PositionEqual(cell_size)
        );

        pose_received = false;
        cmd_received = false;
        last_slip_data_update = ros::Time::now().toSec();

        if (slip_polynomial_degree == 2){
            slipFitCurve = [this](const std::vector<cv::Point2f>& points, const int degree){
                return fitLine(points, degree);
            };
        } else if (slip_polynomial_degree > 2){
            slipFitCurve = [this](const std::vector<cv::Point2f>& points, const int degree){
                return fitPolynomial(points, degree);
            };
        } else {
            ROS_ERROR("slip_node: Invalid slip polynomial degree: %d", slip_polynomial_degree);
        }

         if (slip_angle_polynomial_degree == 2){
            slipAngleFitCurve = [this](const std::vector<cv::Point2f>& points, const int degree){
                return fitLine(points, degree);
            };
        } else if (slip_angle_polynomial_degree > 2){
            slipAngleFitCurve = [this](const std::vector<cv::Point2f>& points, const int degree){
                return fitPolynomial(points, degree);
            };
        } else {
            ROS_ERROR("slip_node: Invalid slip angle polynomial degree: %d", slip_angle_polynomial_degree);
        }

        max_polynomial_degree = std::max(slip_polynomial_degree, slip_angle_polynomial_degree);

        odom_sub = nh_.subscribe(odom_topic_name, 5, &SlipNode::odom_callback, this);
        motor_sub = nh_.subscribe("/cmd_vel_motors", 5, &SlipNode::motor_callback, this);
        terramechanic_forces_sub = nh_.subscribe("/terramechanic_forces", 10, &SlipNode::terramechanic_forces_callback, this);

        slip_pub = nh_.advertise<robot4ws_msgs::SlipUpdate>(slip_update_topic_name, 1);
        slip_pub_raw_data = nh_.advertise<robot4ws_msgs::SlipUpdateRawDataArray>("slip_raw_data", 1);

        get_surface_normal_client = nh_.serviceClient<robot4ws_mapping::get_surface_normal>("get_surface_normal");

        ROS_INFO("Slip node ready...");
    }

    void terramechanic_forces_callback(const robot4ws_msgs::Vector3Array::ConstPtr& msg) {
        // Process all vectors in the message
        for (size_t i = 0; i < msg->names.size(); i++) {
            if (msg->names[i].find("::F_world") != std::string::npos) {
                this->wheel_load = msg->vectors[i].z;  // Update the class member variable
                //ROS_INFO_STREAM("Contact force z for " << msg->names[i] << ": " << wheel_load);
            }
        }
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!cmd_received) return;

        Eigen::Vector3d linear_velocity_robot;
        linear_velocity_robot << msg->twist.twist.linear.x,
                                msg->twist.twist.linear.y,
                                msg->twist.twist.linear.z;

        Eigen::Vector3d angular_velocity_robot;
        angular_velocity_robot << msg->twist.twist.angular.x,
                                msg->twist.twist.angular.y,
                                msg->twist.twist.angular.z;

        const auto& pose = msg->pose.pose;
        tf2::Transform odom_transform(
            tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            tf2::Vector3(pose.position.x, pose.position.y, pose.position.z)
        );

        for (size_t i = 0; i < wheel_commands.size(); ++i) {
            tf2::Vector3 translation_tf = wheels_tf[i].getOrigin();
            Eigen::Vector3d translation_eigen(translation_tf.x(), translation_tf.y(), translation_tf.z());

            // Calculate wheel velocity in robot frame
            Eigen::Vector3d wheel_velocity_robot = linear_velocity_robot + angular_velocity_robot.cross(translation_eigen);

            // Transform to wheel's steer frame
            Eigen::Matrix3d R_z = computeRotationMatrixZ(wheel_commands[i].second);
            Eigen::Vector3d wheel_velocity = R_z.inverse() * wheel_velocity_robot;

            // Calculate slip
            std::pair<float,float> wheel_slip = computeSlip(wheel_commands[i].first, wheel_velocity);

            // Get wheel position and transform
            tf2::Transform wheel_tf = getWheelTf(odom_transform, i);
            grid_map::Position wheel_map_position(wheel_tf.getOrigin().x(), wheel_tf.getOrigin().y());

            auto response = surface_normal_call(wheel_map_position);

            // Discretize to cell center for consistent position-based storage
            grid_map::Position cell_center_position = discretizeToCell(wheel_map_position);

            // Calculate approach angle
            double wheel_angle = compute_steer_uphill_angle(wheel_tf);

            Eigen::Vector3d velocity_ground = computeVelocityGround(wheel_velocity, wheel_tf, response.first);

            double wheel_load = computeWheelLoad();

            // Store all data including wheel load and ground frame velocity
            PolarPoint slip_polar_point = {
                wheel_slip,
                wheel_angle,
                wheel_commands[i].first,   // commanded velocity
                wheel_commands[i].second,  // steer angle
                wheel_load,                // vertical load
                velocity_ground.x(),       // vxg component
                velocity_ground.y()        // vyg component
            };

            // Store using position instead of index - for better handling map enlargements
            position2slips_map[cell_center_position].push_back(slip_polar_point);

            if (position2slips_map[cell_center_position].size() > max_polynomial_degree) {
                // Convert position to current index when needed for publishing
                grid_map::Index current_index;
                if (getCurrentMapIndex(cell_center_position, current_index)) {
                    auto curve_coeffs = fit_curves(cell_center_position);
                    publish_slip_update(curve_coeffs.first, curve_coeffs.second, current_index.x(), current_index.y());
                }
            }
        }

        // Periodically publish raw slip data
        double time_now = ros::Time::now().toSec();
        if (std::abs(last_slip_data_update - time_now) >= 3) {
            publish_slip_raw_data();
            last_slip_data_update = time_now;
        }
    }

    void motor_callback(const robot4ws_msgs::Dynamixel_parameters1::ConstPtr& msg){
        cmd_received = true;

        // BR
        wheel_commands[0].first = (msg -> One_Primary) * wheel_radius; // linear vel (m/s)
        wheel_commands[0].second = msg -> Five_Primary;
        //wheel_commands[0].second = computeRotationMatrixZ(msg -> Five_Primary);

        // FR
        wheel_commands[1].first = (msg -> Two_Primary) * wheel_radius;
        wheel_commands[1].second = msg -> Six_Primary;
        // wheel_commands[1].second = computeRotationMatrixZ(msg -> Six_Primary);

        // BL
        wheel_commands[2].first = (msg -> Three_Primary) * wheel_radius;
        wheel_commands[2].second = msg -> Seven_Primary;
        // wheel_commands[2].second = computeRotationMatrixZ(msg -> Seven_Primary);

        // FL
        wheel_commands[3].first = (msg -> Four_Primary) * wheel_radius;
        wheel_commands[3].second = msg -> Eight_Primary;
        // wheel_commands[3].second = computeRotationMatrixZ(msg -> Eight_Primary);
    }

    void publish_slip_update(std::vector<float> long_slip_coeffs, std::vector<float> trans_slip_coeffs, int x, int y){
        robot4ws_msgs::SlipUpdate update_slip_msg;

        std_msgs::Float32MultiArray array_long; 
        std_msgs::Float32MultiArray array_trasv;

        array_long.data = long_slip_coeffs;
        array_trasv.data = trans_slip_coeffs;
    
        update_slip_msg.longitudinal_coeffs = array_long;
        update_slip_msg.trasversal_coeffs = array_trasv;

        update_slip_msg.map_cell_x = x;
        update_slip_msg.map_cell_y = y;

        slip_pub.publish(update_slip_msg);
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    ros::Subscriber motor_sub;
    ros::Subscriber terramechanic_forces_sub;
    ros::Publisher slip_pub_raw_data;
    ros::Publisher slip_pub;
    ros::ServiceClient get_surface_normal_client;

    std::string odom_topic_name, slip_update_topic_name, foot_print_frame_id;

    double local_map_size, cell_size;
    double last_slip_data_update;
    double wheel_load = 0.0;

    std::function<std::vector<float>(const std::vector<cv::Point2f>&, const int degree)> slipFitCurve;
    std::function<std::vector<float>(const std::vector<cv::Point2f>&, const int degree)> slipAngleFitCurve;
    int slip_polynomial_degree, slip_angle_polynomial_degree, max_polynomial_degree;

    float wheel_radius;

    // (drive_velocity, steer)
    std::array<std::pair<float, float>, 4> wheel_commands;

    // Position-based storage instead of index-based - for better handling to map enlargements
    std::unordered_map<grid_map::Position, std::vector<PolarPoint>, PositionHash, PositionEqual> position2slips_map;

    std::array<tf2::Transform, 4> wheels_tf;

    nav_msgs::Odometry actual_odom_msg;
    bool pose_received;
    bool cmd_received;

    // Discretize position to cell center for consistent hashing
    grid_map::Position discretizeToCell(const grid_map::Position& pos) {
        double x_cell = std::round(pos.x() / cell_size) * cell_size;
        double y_cell = std::round(pos.y() / cell_size) * cell_size;
        return grid_map::Position(x_cell, y_cell);
    }

    // Get current map index for a position via service call
    bool getCurrentMapIndex(const grid_map::Position& position, grid_map::Index& index) {
        robot4ws_mapping::get_surface_normal srv;
        srv.request.position.x = position.x();
        srv.request.position.y = position.y();
        srv.request.position.z = 0.0;

        if (get_surface_normal_client.call(srv)) {
            index.x() = srv.response.map_cell_x;
            index.y() = srv.response.map_cell_y;
            return true;
        } else {
            ROS_WARN("slip_node: Failed to get current map index for position (%.2f, %.2f)", position.x(), position.y());
            return false;
        }
    }

    tf2::Transform getWheelTf(const tf2::Transform& odom_transform, size_t wheel_index){
        const Eigen::Quaterniond eigen_quaternion(computeRotationMatrixZ(wheel_commands[wheel_index].second));
        wheels_tf[wheel_index].setRotation(tf2::Quaternion(
            eigen_quaternion.x(), eigen_quaternion.y(), eigen_quaternion.z(), eigen_quaternion.w()
        ));

        return odom_transform * wheels_tf[wheel_index];
    }

    // Updated to use position-based storage
    std::pair<std::vector<float>, std::vector<float>> fit_curves(const grid_map::Position& position){
        std::vector<PolarPoint> polarPoints = position2slips_map[position];

        // Compute linear regression for slip
        std::vector<cv::Point2f> cartesian_slip_point = slip2cartesian_points(polarPoints, true);
        std::vector<float> slip_line_coeffs = slipFitCurve(cartesian_slip_point, slip_polynomial_degree);

        // Compute linear regression for slip angle
        std::vector<cv::Point2f> cartesian_slip_angle_point = slip2cartesian_points(polarPoints, false);
        std::vector<float> slip_angle_line_coeffs = slipAngleFitCurve(cartesian_slip_angle_point, slip_angle_polynomial_degree);

        return {slip_line_coeffs, slip_angle_line_coeffs};
    }

    std::vector<cv::Point2f> slip2cartesian_points(const std::vector<PolarPoint>& slip_points, bool slip_value){
        std::vector<cv::Point2f> cartesianPoints;
        cartesianPoints.reserve(slip_points.size());

        for (const auto& point : slip_points){
            cartesianPoints.emplace_back(slip_value ? point.value.first : point.value.second, point.angle);
        }
        return cartesianPoints;
    }

    std::vector<float> fitLine(const std::vector<cv::Point2f>& points, int degree){
        cv::Vec4f line;
        cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);

        float vx = line[0], vy = line[1];
        float x0 = line[2], y0 = line[3];

        if (std::fabs(vx) < 1e-6) {
            // vertical line: infinite slope
            return std::vector<float>{std::numeric_limits<float>::infinity(), x0};
        }

        float m = vy / vx;
        float c = y0 - m * x0;

        return std::vector<float>{m, c};
    }

    std::vector<float> fitPolynomial(const std::vector<cv::Point2f>& points, int degree) {
        if (points.size() <= degree) {
            throw std::invalid_argument("Il numero di punti deve essere maggiore del grado del polinomio");
        }

        // Numero di punti
        size_t N = points.size();

        // Matrice di Vandermonde
        Eigen::MatrixXd X(N, degree + 1);
        Eigen::VectorXd Y(N);

        for (size_t i = 0; i < N; ++i) {
            double xi = points[i].x;
            double yi = points[i].y;

            // Popola la riga della matrice di Vandermonde
            for (int j = 0; j <= degree; ++j) {
                X(i, j) = std::pow(xi, j);
            }

            // Popola il vettore Y
            Y(i) = yi;
        }

        // Risolvi il sistema lineare: (X^T * X) * coeff = X^T * Y
        Eigen::VectorXd coeff = (X.transpose() * X).ldlt().solve(X.transpose() * Y);

        // Converti i coefficienti in un vettore standard
        std::vector<float> coefficients(coeff.data(), coeff.data() + coeff.size());
        return coefficients;
    }

    Eigen::Matrix3d computeRotationMatrixZ(double theta){
        double c = cos(theta);
        double s = sin(theta);
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix << c, -s, 0,
                        s,  c, 0,
                        0,  0, 1;
        return rotation_matrix;
    }

    Eigen::Vector3d computeVelocityGround(const Eigen::Vector3d& wheel_velocity, 
                                        const tf2::Transform& wheel_tf, 
                                        const geometry_msgs::Vector3& surface_normal) {
        // Convert surface normal to Eigen vector
        Eigen::Vector3d normal(surface_normal.x, surface_normal.y, surface_normal.z);
        normal.normalize();

        // Define ground frame axes (z is normal, x is uphill direction, y completes right-handed frame)
        Eigen::Vector3d z_ground = normal;
        Eigen::Vector3d x_ground = compute_surface_uphill(normal);
        x_ground.normalize();
        Eigen::Vector3d y_ground = z_ground.cross(x_ground);
        y_ground.normalize();

        // Create rotation matrix from wheel frame to ground frame
        tf2::Matrix3x3 basis = wheel_tf.getBasis();
        Eigen::Matrix3d R_wheel_to_global;
        R_wheel_to_global << basis[0][0], basis[0][1], basis[0][2],
                            basis[1][0], basis[1][1], basis[1][2],
                            basis[2][0], basis[2][1], basis[2][2];

        Eigen::Matrix3d R_global_to_ground;
        R_global_to_ground.row(0) = x_ground;
        R_global_to_ground.row(1) = y_ground;
        R_global_to_ground.row(2) = z_ground;

        return R_global_to_ground * R_wheel_to_global * wheel_velocity;
    }

    std::pair<float, float> computeSlip(float theoretical_vel, Eigen::Vector3d real_vel) {
        std::pair<float, float> slip;
        slip.second = atan2(real_vel[1], real_vel[0]);

        float delta_v = theoretical_vel - real_vel[0];

        if (std::abs(theoretical_vel) > std::abs(real_vel[0])) {
            slip.first = delta_v / theoretical_vel;
        } else if (std::abs(real_vel[0]) > 0) {
            slip.first = delta_v / real_vel[0];
        } else {
            slip.first = 0;
        }
        return slip;
    }

    Eigen::Vector3d compute_surface_uphill(const Eigen::Vector3d& surface_normal){
        Eigen::Vector3d gravity(0, 0, -1);
        return -(gravity - (surface_normal.dot(gravity) / surface_normal.squaredNorm()) * surface_normal);
    }

    std::pair<geometry_msgs::Vector3, std::pair<uint32_t, uint32_t>> surface_normal_call(grid_map::Position position){
        geometry_msgs::Point point;
        point.x = position.x();
        point.y = position.y();
        point.z = 0.0;

        robot4ws_mapping::get_surface_normal srv;
        srv.request.position = point;
        if(get_surface_normal_client.call(srv)){
            return {srv.response.normal, {srv.response.map_cell_x, srv.response.map_cell_y}};
        }
        else {
            ROS_WARN("slip_node: 'get_surface_normal' service call error.");
            geometry_msgs::Vector3 error_value;
            error_value.x = std::numeric_limits<double>::quiet_NaN();
            error_value.y = std::numeric_limits<double>::quiet_NaN();
            error_value.z = std::numeric_limits<double>::quiet_NaN();
            return {error_value, {point.x, point.y}};
        }
    }

    double compute_steer_uphill_angle(const tf2::Transform& transform){
        // Rotazione ZYZ
        // Estrarre la matrice di rotazione dalla trasformazione
        tf2::Matrix3x3 rotation_matrix(transform.getRotation());

        // Variabili per gli angoli di Eulero
        double beta, gamma;

        // Estrarre gli elementi della matrice
        double r31 = rotation_matrix[2][0];
        double r32 = rotation_matrix[2][1];
        double r33 = rotation_matrix[2][2];

        // Calcolare beta (rotazione attorno a Y, intermedia)
        beta = std::acos(r33);

        // Verificare se siamo in una singolarità
        if (std::sin(beta) > 1e-6) { // Caso generale
            gamma = std::atan2(r32, -r31); // Seconda rotazione attorno a Z
            if(beta > 0) gamma = gamma + M_PI; // downhill -> uphill
        } else { // Caso di singolarità
            gamma = 0.0; // Nessuna rotazione secondaria
        }

        // Change from [0 2pi] to [-pi pi]
        if (gamma > M_PI){
            gamma = gamma - 2*M_PI;
        }

        return gamma;
    }


    double computeWheelLoad() {
        return wheel_load;
    }


    void load_robot_static_tf(){
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);

        try {
            geometry_msgs::TransformStamped br_wheel_link = tf_buffer.lookupTransform(foot_print_frame_id, "Archimede_br_wheel_link", ros::Time(0),ros::Duration(5.0));
            tf2::fromMsg(br_wheel_link.transform, wheels_tf[0]);

            geometry_msgs::TransformStamped bl_wheel_link = tf_buffer.lookupTransform(foot_print_frame_id, "Archimede_bl_wheel_link", ros::Time(0),ros::Duration(5.0));
            tf2::fromMsg(bl_wheel_link.transform, wheels_tf[2]);

            geometry_msgs::TransformStamped fl_wheel_link = tf_buffer.lookupTransform(foot_print_frame_id, "Archimede_fl_wheel_link", ros::Time(0),ros::Duration(5.0));
            tf2::fromMsg(fl_wheel_link.transform, wheels_tf[3]);

            geometry_msgs::TransformStamped fr_wheel_link = tf_buffer.lookupTransform(foot_print_frame_id, "Archimede_fr_wheel_link", ros::Time(0),ros::Duration(5.0));
            tf2::fromMsg(fr_wheel_link.transform, wheels_tf[1]);
            }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }

    void load_params(){
        //Gridmap params
        if (!nh_.getParam("gridmap/local_map_size", local_map_size)) {
            local_map_size = 5.0;
            ROS_WARN_STREAM("Parameter [gridmap/local_map_size] not found. Using default value: " << local_map_size);
        }
        if (! nh_.getParam("gridmap/cell_size",cell_size))
        {
            cell_size = 0.2;
            ROS_WARN_STREAM("Parameter [gridmap/cell_size] not found. Using default value: " << cell_size);
        }

        //Topics
        if (! nh_.getParam("gridmap/odom_topic_name",odom_topic_name))
        {
            odom_topic_name = "/odom";
            ROS_WARN_STREAM("Parameter [gridmap/odom_topic_name] not found. Using default value: " << odom_topic_name);
        }
        if (! nh_.getParam("gridmap/slip_update_topic_name",slip_update_topic_name))
        {
            slip_update_topic_name = "slip_update";
            ROS_WARN_STREAM("Parameter [gridmap/slip_update_topic_name] not found. Using default value: " << slip_update_topic_name);
        }

        if (! nh_.getParam("gridmap/foot_print_frame_id",foot_print_frame_id))
        {
            foot_print_frame_id = "footprint";
            ROS_WARN_STREAM("Parameter [gridmap/foot_print_frame_id] not found. Using default value: " << foot_print_frame_id);
        }

        if (! nh_.getParam("gridmap/slip/slip_polynomial_degree",slip_polynomial_degree))
        {
            slip_polynomial_degree = 2;
            ROS_WARN_STREAM("Parameter [gridmap/slip/slip_polynomial_degree] not found. Using default value: " << slip_polynomial_degree);
        }
        if (! nh_.getParam("gridmap/slip/slip_angle_polynomial_degree",slip_angle_polynomial_degree))
        {
            slip_angle_polynomial_degree = 2;
            ROS_WARN_STREAM("Parameter [gridmap/slip/slip_angle_polynomial_degree] not found. Using default value: " << slip_angle_polynomial_degree);
        }

        if (! nh_.getParam("gridmap/wheel_radius",wheel_radius))
        {
            wheel_radius = 0.085;
            ROS_WARN_STREAM("Parameter [gridmap/wheel_radius] not found. Using default value: " << wheel_radius);
        }
    }
    
    void publish_slip_raw_data() {
        robot4ws_msgs::SlipUpdateRawDataArray msg_array;
        msg_array.header.stamp = ros::Time::now();

        // Process each map cell's slip data using position-based storage
        for (const auto& pair : position2slips_map) {
            const grid_map::Position& position = pair.first;
            const std::vector<PolarPoint>& points_in_cell = pair.second;

            if (points_in_cell.empty()) continue;

            robot4ws_msgs::SlipUpdateRawData single_data_point;

            // Get current indices for this position
            grid_map::Index current_index;
            if (getCurrentMapIndex(position, current_index)) {
                single_data_point.map_cell_x = current_index.x();
                single_data_point.map_cell_y = current_index.y();
            } else {
                continue;
            }

            // Prepare arrays to hold the data
            size_t num_points = points_in_cell.size();

            single_data_point.slip_ratio.data.resize(num_points);
            single_data_point.slip_angle.data.resize(num_points);
            single_data_point.approach_angles.data.resize(num_points);
            single_data_point.commanded_velocities.data.resize(num_points);
            single_data_point.steer_angle.data.resize(num_points);
            single_data_point.wheel_loads.data.resize(num_points);         
            single_data_point.vxg_ground_truth.data.resize(num_points);    
            single_data_point.vyg_ground_truth.data.resize(num_points);    

            // Populate arrays with data from each point
            for (size_t i = 0; i < num_points; ++i) {
                single_data_point.slip_ratio.data[i] = points_in_cell[i].value.first;
                single_data_point.slip_angle.data[i] = points_in_cell[i].value.second;
                single_data_point.approach_angles.data[i] = points_in_cell[i].angle;
                single_data_point.commanded_velocities.data[i] = points_in_cell[i].velocity_commanded;
                single_data_point.steer_angle.data[i] = points_in_cell[i].steer_angle;
                single_data_point.wheel_loads.data[i] = points_in_cell[i].wheel_load;           
                single_data_point.vxg_ground_truth.data[i] = points_in_cell[i].vxg_sim;         
                single_data_point.vyg_ground_truth.data[i] = points_in_cell[i].vyg_sim;         
            }

            msg_array.data.push_back(single_data_point);
        }

        // Only publish if we have data
        if (!msg_array.data.empty()) {
            slip_pub_raw_data.publish(msg_array);
            ROS_INFO_THROTTLE(10.0, "Published slip raw data with %zu cell entries", msg_array.data.size());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slip_node");

    SlipNode SlipNode;

    ros::spin();
    return 0;
}