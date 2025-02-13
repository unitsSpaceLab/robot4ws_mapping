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

#include <opencv2/opencv.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <functional>

struct PolarPoint{
    std::pair<float,float> value;
    double angle;
};


class SlipNode
{
public:
    SlipNode()
    {
        //TODO: aggiungere wheel number
        load_params();
        pose_received = false;
        cmd_received = false;

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
       
        max_polynomial_degree =  std::max(slip_polynomial_degree, slip_angle_polynomial_degree);

        odom_sub = nh_.subscribe(odom_topic_name, 5, &SlipNode::odom_callback, this);
        motor_sub = nh_.subscribe("/cmd_vel_motors", 5, &SlipNode::motor_callback, this);

        slip_pub = nh_.advertise<robot4ws_msgs::SlipUpdate>(slip_update_topic_name, 1);

        get_surface_normal_client = nh_.serviceClient<robot4ws_mapping::get_surface_normal>("get_surface_normal");

        ROS_INFO("Slip node ready...");
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
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

        for (size_t i = 0; i < wheel_commands.size(); ++i){

            tf2::Vector3 translation_tf = wheels_tf[i].getOrigin();
            Eigen::Vector3d translation_eigen(translation_tf.x(), translation_tf.y(), translation_tf.z());

            Eigen::Vector3d wheel_velocity_robot = linear_velocity_robot + angular_velocity_robot.cross(translation_eigen);

            Eigen::Matrix3d R_z = computeRotationMatrixZ(wheel_commands[i].second);
            Eigen::Vector3d wheel_velocity = R_z * wheel_velocity_robot;
            std::pair<float,float> wheel_slip = computeSlip(wheel_commands[i].first, wheel_velocity);

            tf2::Transform wheel_tf = getWheelTf(odom_transform, i);

            // grid_map::Position wheel_map_position = getWheelPosition(odom_transform, i); 
            grid_map::Position wheel_map_position = grid_map::Position(
                wheel_tf.getOrigin().x(),
                wheel_tf.getOrigin().y()
            );

            double wheel_angle = compute_steer_uphill_angle(wheel_tf);

            auto response = surface_normal_call(wheel_map_position);

            //SURFACE NORMAL NOT USED
            /* geometry_msgs::Vector3 normal = response.first;

            if (std::isnan(normal.x) || std::isnan(normal.y) || std::isnan(normal.z)){
                ROS_ERROR("slip_node: Invalid normal vector received from get_surface_normal service.");
                return;
            } */

            uint32_t map_cell_x = response.second.first;
            uint32_t map_cell_y = response.second.second;
            grid_map::Index map_index(map_cell_x, map_cell_y);

            //Eigen::Vector3d eigen_normal(normal.x, normal.y, normal.z);
            //float wheel_angle = compute_steer_uphill_angle(eigen_normal, i);

            PolarPoint slip_polar_point = {wheel_slip, wheel_angle};

            position2slips_map[map_index].push_back(slip_polar_point);
            if(position2slips_map[map_index].size() > max_polynomial_degree){
                auto curve_coeffs = fit_curves(map_index);
                publish_slip_update(curve_coeffs.first, curve_coeffs.second, map_cell_x, map_cell_y);
            }
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
    ros::Publisher slip_pub;
    ros::ServiceClient get_surface_normal_client;

    std::string odom_topic_name, slip_update_topic_name, foot_print_frame_id;

    double local_map_size, cell_size;

    std::function<std::vector<float>(const std::vector<cv::Point2f>&, const int degree)> slipFitCurve;
    std::function<std::vector<float>(const std::vector<cv::Point2f>&, const int degree)> slipAngleFitCurve;
    int slip_polynomial_degree, slip_angle_polynomial_degree, max_polynomial_degree;

    float wheel_radius;

    // (drive_velocity, steer)
    std::array<std::pair<float, float>, 4> wheel_commands;

    std::unordered_map<grid_map::Index, std::vector<PolarPoint>, robot4ws_mapping::IndexHash, robot4ws_mapping::IndexEqual> position2slips_map;

    std::array<tf2::Transform, 4> wheels_tf;

    nav_msgs::Odometry actual_odom_msg;
    bool pose_received;
    bool cmd_received;

    tf2::Transform getWheelTf(const tf2::Transform& odom_transform, size_t wheel_index){
        const Eigen::Quaterniond eigen_quaternion(computeRotationMatrixZ(wheel_commands[wheel_index].second));
        wheels_tf[wheel_index].setRotation(tf2::Quaternion(
            eigen_quaternion.x(), eigen_quaternion.y(), eigen_quaternion.z(), eigen_quaternion.w()
        ));

        return odom_transform * wheels_tf[wheel_index];
    }

    /* grid_map::Position getWheelPosition(const tf2::Transform& odom_transform, size_t wheel_index){
        const Eigen::Quaterniond eigen_quaternion(computeRotationMatrixZ(wheel_commands[wheel_index].second));
        wheels_tf[wheel_index].setRotation(tf2::Quaternion(
            eigen_quaternion.x(), eigen_quaternion.y(), eigen_quaternion.z(), eigen_quaternion.w()
        ));

        const auto transformed_wheel = odom_transform * wheels_tf[wheel_index];

        return grid_map::Position(
            transformed_wheel.getOrigin().x(),
            transformed_wheel.getOrigin().y()
        );
    } */

    std::pair<std::vector<float>, std::vector<float>> fit_curves(grid_map::Index wheel_map_index){

        std::vector<PolarPoint> polarPoints = position2slips_map[wheel_map_index];

        // Compute linear regression for slip
        std::vector<cv::Point2f> cartesian_slip_point = slip2cartesian_points(polarPoints, true);
        std::vector<float> slip_line_coeffs = slipFitCurve(cartesian_slip_point, slip_polynomial_degree);

        // Compute linear regression for slip angle
        std::vector<cv::Point2f> cartesian_slip_angle_point = slip2cartesian_points(polarPoints, false);
        std::vector<float> slip_angle_line_coeffs = slipAngleFitCurve(cartesian_slip_angle_point, slip_angle_polynomial_degree);

        return {slip_line_coeffs, slip_angle_line_coeffs};
    }

    /* std::vector<cv::Point2f> slip2cartesian_points(const std::vector<PolarPoint>& slip_points, bool longitudinal){
        std::vector<cv::Point2f> cartesianPoints;
        cartesianPoints.reserve(slip_points.size());

        for (const auto& point : slip_points) {
            float r = longitudinal ? point.value.first : point.value.second;
            float cos_beta = std::cos(point.angle);
            float sin_beta = std::sin(point.angle);
            cartesianPoints.emplace_back(r * cos_beta, r * sin_beta);
        }
        return cartesianPoints;
    } */

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

    /* float compute_steer_uphill_angle(const Eigen::Vector3d& normal, int i){
        Eigen::Vector3d uphill_vector = compute_surface_uphill(normal);
        return std::acos(wheel_commands[i].second.dot(uphill_vector) / (wheel_commands[i].second.norm() * uphill_vector.norm()));
    } */

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

        // Calcolare beta (rotazione attorno a Z, intermedia)
        beta = std::acos(r33);

        // Verificare se siamo in una singolarità
        if (std::sin(beta) > 1e-6) { // Caso generale
            gamma = std::atan2(r32, -r31); // Seconda rotazione attorno a Z
            if(beta > 0) gamma = gamma + M_PI; // downhill -> uphill
        } else { // Caso di singolarità
            gamma = 0.0; // Nessuna rotazione secondaria
        }

        return gamma;
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

        /* //Slip layer names
        if (!nh_.getParam("gridmap/slip/slip_layer_name",slip_layer_name))
        {
            slip_layer_name = "/slip";
            ROS_WARN_STREAM("Parameter [gridmap/slip_layer_name] not found. Using default value: " << slip_layer_name);
        }
        if (!nh_.getParam("gridmap/slip/slip_longitudinal_vertical_axis_layer_name",slip_longitudinal_vertical_axis_layer_name))
        {
            slip_longitudinal_vertical_axis_layer_name = "/slip_longitudinal_vertical_axis";
            ROS_WARN_STREAM("Parameter [gridmap/slip_longitudinal_vertical_axis_layer_name] not found. Using default value: " << slip_longitudinal_vertical_axis_layer_name);
        }
        if (!nh_.getParam("gridmap/slip/slip_longitudinal_horizontal_axis_layer_name",slip_longitudinal_horizontal_axis_layer_name))
        {
            slip_longitudinal_horizontal_axis_layer_name = "/slip_longitudinal_horizontal_axis";
            ROS_WARN_STREAM("Parameter [gridmap/slip_longitudinal_horizontal_axis_layer_name] not found. Using default value: " << slip_longitudinal_horizontal_axis_layer_name);
        }
        if (!nh_.getParam("gridmap/slip/slip_trasversal_vertical_axis_layer_name",slip_trasversal_vertical_axis_layer_name))
        {
            slip_trasversal_vertical_axis_layer_name = "/slip_trasversal_vertical_axis";
            ROS_WARN_STREAM("Parameter [gridmap/slip_trasversal_vertical_axis_layer_name] not found. Using default value: " << slip_trasversal_vertical_axis_layer_name);
        }
        if (!nh_.getParam("gridmap/slip/slip_trasversal_horizontal_axis_layer_name",slip_trasversal_horizontal_axis_layer_name))
        {
            slip_trasversal_horizontal_axis_layer_name = "/slip_trasversal_horizontal_axis";
            ROS_WARN_STREAM("Parameter [gridmap/slip_trasversal_horizontal_axis_layer_name] not found. Using default value: " << slip_trasversal_horizontal_axis_layer_name);
        } */

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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slip_node");

    SlipNode SlipNode;

    ros::spin();
    return 0;
}
