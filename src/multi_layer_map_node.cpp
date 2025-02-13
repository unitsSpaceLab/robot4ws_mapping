#include <ros/ros.h>
#include "robot4ws_mapping/utilities.hpp"

#include <sensor_msgs/PointCloud2.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <robot4ws_mapping/GridMapUpdateMsg.h>
#include <robot4ws_mapping/get_surface_normal.h>
#include <robot4ws_msgs/SlipUpdate.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

class MultiLayerMapNode
{
public:
    MultiLayerMapNode(){
        load_params();
        init_slip_layers();
        load_robot_static_tf();
        initial_variance = 100;

        init_global_map();

        pub_grid_map_update_sub = nh_.subscribe(grid_map_update_topic_name, 10, &MultiLayerMapNode::grid_map_update_callback, this);
        slip_sub = nh_.subscribe(slip_update_topic_name, 10, &MultiLayerMapNode::slip_callback, this);

        grid_map_pub = nh_.advertise<grid_map_msgs::GridMap>(grid_map_topic_name, 1, true);
        grid_map_global_update_pub = nh_.advertise<robot4ws_mapping::GridMapUpdateMsg>(grid_map_global_update_topic_name, 1, true);
        image_pub_ = nh_.advertise<sensor_msgs::Image>("grid_map_image", 1);


        surface_normal_srv = nh_.advertiseService("get_surface_normal", &MultiLayerMapNode::handle_surface_normal_request, this);
    }

    void grid_map_update_callback(const robot4ws_mapping::GridMapUpdateMsg::ConstPtr& msg){

        grid_map::GridMap local_map;
        grid_map::GridMapRosConverter::fromMessage(msg->local_gridmap, local_map);
        std::string layer_id = msg->header.frame_id;

        if (layer_id == elevation_layer_name){
            geometry_msgs::TransformStamped transform_msg = get_transform_msg(msg->odom, true);
            std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> transformed_point_list = applyTransform(local_map, elevation_layer_name, transform_msg);
            updateGlobalMapKalman(local_map, elevation_layer_name, elevation_variance_layer_name, transform_msg, transformed_point_list);
            publish_global_gridmap_update(msg);
        }
        else if(layer_id == surface_orientation_layer_name){
            std::vector<std::string> layers = {surface_orientation_x_layer_name, surface_orientation_y_layer_name, surface_orientation_z_layer_name, surface_orientation_variance_layer_name};
            updateGlobalMap(local_map, layers, nullptr);
        }
        else if(layer_id == obstacles_layer_name){
            std::vector<std::string> layers = {obstacles_layer_name, obstacles_variance_layer_name};
            updateGlobalMap(local_map, layers, nullptr);
        }
        else if(layer_id == color_layer_name){
            geometry_msgs::TransformStamped transform_msg = get_transform_msg(msg->odom, false);
            updateColorGlobalMap(local_map, transform_msg);
            /* std::vector<std::string> layers = {color_layer_name};
            updateGlobalMap(local_map, layers, nullptr); */
            publishColorMapImage();
        }
        publish_gridmap();
    }

    void publish_gridmap(){
        globalMap_.setTimestamp(ros::Time::now().toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(globalMap_, message);
        message.info.header.frame_id = grid_map_frame_id;
        grid_map_pub.publish(message);
    }

    void slip_callback(const robot4ws_msgs::SlipUpdate::ConstPtr& msg) {
        grid_map::Index map_index(msg->map_cell_x, msg->map_cell_y);
        
        std_msgs::Float32MultiArray longitudinal_coeffs = msg->longitudinal_coeffs;       
        std_msgs::Float32MultiArray trasversal_coeffs = msg->trasversal_coeffs;

        for (int i = 0; i < slip_polynomial_degree; ++i) {
            globalMap_.at(slip_layer_names[i], map_index) = longitudinal_coeffs.data[i];
        }

        for (int i = 0; i < slip_angle_polynomial_degree; ++i) {
            globalMap_.at(slip_angle_layer_names[i], map_index) = trasversal_coeffs.data[i];
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pub_grid_map_update_sub;

    ros::Subscriber slip_sub;
    ros::Publisher grid_map_pub;
    ros::Publisher grid_map_global_update_pub;
    ros::Publisher image_pub_;

    ros::ServiceServer surface_normal_srv;

    //GridMap attributes:
    grid_map::GridMap globalMap_;
    grid_map::Position last_center_position_global_frame;
    std::string grid_map_frame_id;

    double global_map_size, local_map_size;
    double cell_size, initial_variance;

    double slip_angle_polynomial_degree, slip_polynomial_degree;

    //Layer names
    std::string elevation_layer_name, elevation_variance_layer_name;
    std::string cloudPoint_counter_layer_name, color_layer_name;
    std::string obstacles_layer_name, obstacles_variance_layer_name, surface_orientation_layer_name;
    std::string surface_orientation_x_layer_name, surface_orientation_y_layer_name, surface_orientation_z_layer_name, surface_orientation_variance_layer_name;
    std::string slip_layer_name, slip_angle_layer_name;
    std::vector<std::string> slip_layer_names;
    std::vector<std::string> slip_angle_layer_names;

    //Topics
    std::string slip_update_topic_name, grid_map_update_topic_name, grid_map_topic_name, grid_map_global_update_topic_name;

    nav_msgs::Odometry actual_odom_msg;

    //Color map counter
    std::unordered_map<grid_map::Index, std::vector<int>, robot4ws_mapping::IndexHash, robot4ws_mapping::IndexEqual> colorCounts_;

    //Robot related params:
    std::string lidar_frame_id;
    std::string foot_print_frame_id;
    tf2::Transform foot2lidar_transform;

    void load_params(){
        //load gridmap params
        if (! nh_.getParam("gridmap/grid_map_frame_id",grid_map_frame_id))
        {
            grid_map_frame_id = "odom";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_frame_id] not found. Using default value: " << grid_map_frame_id);
        }
        if (! nh_.getParam("gridmap/global_map_size",global_map_size))
        {
            global_map_size = 5.0;
            ROS_WARN_STREAM("Parameter [gridmap/global_map_size] not found. Using default value: " << global_map_size);
        }
        if (!nh_.getParam("gridmap/local_map_size", local_map_size)) {
            local_map_size = 5.0;
            ROS_WARN_STREAM("Parameter [gridmap/local_map_size] not found. Using default value: " << local_map_size);
        }
        if (! nh_.getParam("gridmap/cell_size",cell_size))
        {
            cell_size = 0.2;
            ROS_WARN_STREAM("Parameter [gridmap/cell_size] not found. Using default value: " << cell_size);
        }

        //load layer names
        if (!nh_.getParam("gridmap/cloudPoint_counter_layer_name",cloudPoint_counter_layer_name))
        {
            cloudPoint_counter_layer_name = "cloudPoint_counter";
            ROS_WARN_STREAM("Parameter [gridmap/cloudPoint_counter_layer_name] not found. Using default value: " << cloudPoint_counter_layer_name);
        }
        if (!nh_.getParam("gridmap/elevation_layer_name",elevation_layer_name))
        {
            elevation_layer_name = "elevation";
            ROS_WARN_STREAM("Parameter [gridmap/elevation_layer_name] not found. Using default value: " << elevation_layer_name);
        }
        if (!nh_.getParam("gridmap/elevation_variance_layer_name",elevation_variance_layer_name))
        {
            elevation_variance_layer_name = "elevation_variance";
            ROS_WARN_STREAM("Parameter [gridmap/elevation_variance_layer_name] not found. Using default value: " << elevation_variance_layer_name);
        }
        if (!nh_.getParam("gridmap/obstacles_layer_name",obstacles_layer_name))
        {
            obstacles_layer_name = "obstacles";
            ROS_WARN_STREAM("Parameter [gridmap/obstacles_layer_name] not found. Using default value: " << obstacles_layer_name);
        }
        if (!nh_.getParam("gridmap/obstacles_variance_layer_name",obstacles_variance_layer_name))
        {
            obstacles_variance_layer_name = "obstacles_variance";
            ROS_WARN_STREAM("Parameter [gridmap/obstacles_variance_layer_name] not found. Using default value: " << obstacles_variance_layer_name);
        }
        if (!nh_.getParam("gridmap/surface_orientation_x_layer_name",surface_orientation_x_layer_name))
        {
            surface_orientation_x_layer_name = "surface_orientation_x";
            ROS_WARN_STREAM("Parameter [gridmap/surface_orientation_x_layer_name] not found. Using default value: " << surface_orientation_x_layer_name);
        }
        if (!nh_.getParam("gridmap/surface_orientation_y_layer_name",surface_orientation_y_layer_name))
        {
            surface_orientation_y_layer_name = "surface_orientation_y";
            ROS_WARN_STREAM("Parameter [gridmap/surface_orientation_y_layer_name] not found. Using default value: " << surface_orientation_y_layer_name);
        }
        if (!nh_.getParam("gridmap/surface_orientation_z_layer_name",surface_orientation_z_layer_name))
        {
            surface_orientation_z_layer_name = "surface_orientation_z_layer";
            ROS_WARN_STREAM("Parameter [gridmap/surface_orientation_z_layer_name] not found. Using default value: " << surface_orientation_z_layer_name);
        }
        if (!nh_.getParam("gridmap/surface_orientation_variance_layer_name",surface_orientation_variance_layer_name))
        {
            surface_orientation_variance_layer_name = "surface_orientation_variance";
            ROS_WARN_STREAM("Parameter [gridmap/surface_orientation_variance_layer_name] not found. Using default value: " << surface_orientation_variance_layer_name);
        }
        if (!nh_.getParam("gridmap/surface_orientation_layer_name",surface_orientation_layer_name))
        {
            surface_orientation_layer_name = "surface_orientation";
            ROS_WARN_STREAM("Parameter [gridmap/surface_orientation_layer_name] not found. Using default value: " << surface_orientation_layer_name);
        }
        if (!nh_.getParam("gridmap/color/color_layer_name",color_layer_name))
        {
            color_layer_name = "color";
            ROS_WARN_STREAM("Parameter [gridmap/color_layer_name] not found. Using default value: " << color_layer_name);
        }

        // Slip curves coefficients
        if (!nh_.getParam("gridmap/slip/slip_angle_layer_name",slip_angle_layer_name))
        {
            slip_angle_layer_name = "slip_angle";
            ROS_WARN_STREAM("Parameter [gridmap/slip/slip_angle_layer_name] not found. Using default value: " << slip_angle_layer_name);
        }
        if (!nh_.getParam("gridmap/slip/slip_layer_name",slip_layer_name))
        {
            slip_layer_name = "slip";
            ROS_WARN_STREAM("Parameter [gridmap/slip/slip_layer_name] not found. Using default value: " << slip_layer_name);
        }
        if (!nh_.getParam("gridmap/slip/slip_polynomial_degree",slip_polynomial_degree))
        {
            slip_polynomial_degree = 4;
            ROS_WARN_STREAM("Parameter [gridmap/slip/slip_polynomial_degree] not found. Using default value: " << slip_polynomial_degree);
        }
        if (!nh_.getParam("gridmap/slip/slip_angle_polynomial_degree",slip_angle_polynomial_degree))
        {
            slip_angle_polynomial_degree = 3;
            ROS_WARN_STREAM("Parameter [gridmap/slip/slip_angle_polynomial_degree] not found. Using default value: " << slip_angle_polynomial_degree);
        }
       
        //load topic names
        if (! nh_.getParam("gridmap/grid_map_update_topic_name",grid_map_update_topic_name))
        {
            grid_map_update_topic_name = "grid_map_update";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_update_topic_name] not found. Using default value: " << grid_map_update_topic_name);
        }
        if (! nh_.getParam("gridmap/grid_map_topic_name",grid_map_topic_name))
        {
            grid_map_topic_name = "grid_map";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_topic_name] not found. Using default value: " << grid_map_topic_name);
        }
        if (! nh_.getParam("gridmap/grid_map_global_update_topic_name",grid_map_global_update_topic_name))
        {
            grid_map_global_update_topic_name = "grid_map_global_update";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_global_update_topic_name] not found. Using default value: " << grid_map_global_update_topic_name);
        }
        if (! nh_.getParam("gridmap/slip_update_topic_name",slip_update_topic_name))
        {
            slip_update_topic_name = "slip_update";
            ROS_WARN_STREAM("Parameter [gridmap/slip_update_topic_name] not found. Using default value: " << slip_update_topic_name);
        }

        //load robot related params
        if (! nh_.getParam("gridmap/lidar_frame_id",lidar_frame_id))
        {
            lidar_frame_id = "velodyne";
            ROS_WARN_STREAM("Parameter [gridmap/lidar_frame_id] not found. Using default value: " << lidar_frame_id);
        }
        if (! nh_.getParam("gridmap/foot_print_frame_id",foot_print_frame_id))
        {
            foot_print_frame_id = "footprint";
            ROS_WARN_STREAM("Parameter [gridmap/foot_print_frame_id] not found. Using default value: " << foot_print_frame_id);
        }
    }

    void init_global_map(){
        globalMap_.setFrameId(grid_map_frame_id);
        globalMap_.setGeometry(grid_map::Length(global_map_size, global_map_size), cell_size);

        globalMap_.add(elevation_layer_name, 0.0);
        globalMap_.add(elevation_variance_layer_name, initial_variance);

        globalMap_.add(obstacles_layer_name, 0.0);
        globalMap_.add(obstacles_variance_layer_name, initial_variance);

        globalMap_.add(surface_orientation_x_layer_name, 0);
        globalMap_.add(surface_orientation_y_layer_name, 0);
        globalMap_.add(surface_orientation_z_layer_name, 0);
        globalMap_.add(surface_orientation_variance_layer_name, initial_variance);

        globalMap_.add(color_layer_name, robot4ws_mapping::Unknown);

        for (const auto& name : slip_layer_names) {
            globalMap_.add(name, 0);
        }

        for (const auto& name : slip_angle_layer_names) {
            globalMap_.add(name, 0);
        }
    }

    void load_robot_static_tf(){
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);

        try {
            geometry_msgs::TransformStamped lidar = tf_buffer.lookupTransform(foot_print_frame_id, lidar_frame_id, ros::Time(0),ros::Duration(5.0));
            tf2::fromMsg(lidar.transform, foot2lidar_transform);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }

    void init_slip_layers(){
        
        slip_layer_names.resize(slip_polynomial_degree);
        slip_angle_layer_names.resize(slip_angle_polynomial_degree);
        
        for (int i = 0; i < slip_polynomial_degree; ++i) {
            slip_layer_names[i] = slip_layer_name + std::to_string(i + 1);
        }

        for (int i = 0; i < slip_angle_polynomial_degree; ++i) {
            slip_angle_layer_names[i] = slip_angle_layer_name + std::to_string(i + 1);
        }

        // Stampa per verifica
        std::cout << "Slip Layer Names:\n";
        for (const auto& name : slip_layer_names) {
            std::cout << name << std::endl;
        }

        std::cout << "\nSlip Angle Layer Names:\n";
        for (const auto& name : slip_angle_layer_names) {
            std::cout << name << std::endl;
        }

    }
    
    /* std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> applyTransform(grid_map::GridMap& localMap, const std::string& layer_name, const geometry_msgs::TransformStamped& transform){
        
        checkEnlargeMap(localMap, transform);

        std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> local_global_points_mapping;

        for (grid_map::GridMapIterator it(localMap); !it.isPastEnd(); ++it){
            const grid_map::Index index(*it);
            const float& layer_value = localMap.at(layer_name, index);

            grid_map::Position localPosition;
            localMap.getPosition(index, localPosition);

            if (std::isnan(localPosition.x()) || std::isnan(localPosition.y()) || std::isnan(layer_value)){
                ROS_WARN("Skipping local position with NaN value: x = %f, y = %f, z = %f", localPosition.x(), localPosition.y(), layer_value);
                continue;
            }

            //skip empty local cells
            //TODO: generalizza con layer bool se cella aggiornata o meno
            if(localMap.atPosition(cloudPoint_counter_layer_name, localPosition) == 0) continue;

            geometry_msgs::PointStamped localPoint, globalPoint;
            localPoint.point.x = localPosition.x();
            localPoint.point.y = localPosition.y();
            localPoint.point.z = layer_value;

            try {
                tf2::doTransform(localPoint, globalPoint, transform);
                local_global_points_mapping.push_back(std::make_tuple(globalPoint, localPoint));
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
            }   
        }
        return local_global_points_mapping;
    }
     */

    std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> applyTransform(grid_map::GridMap& localMap, const std::string& layer_name, geometry_msgs::TransformStamped transform){
        geometry_msgs::PointStamped max,min;
        max.point.x = -1000000;
        max.point.y = -1000000;
        min.point.x = 1000000;
        min.point.y = 1000000;

        std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> local_global_points_mapping;

        // Transform localmap to globalmap 
        for (grid_map::GridMapIterator it(localMap); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            const float& layer_value = localMap.at(layer_name, index);

            grid_map::Position localPosition;
            localMap.getPosition(index, localPosition);

            //skip empty local cells
            //TODO: generalizza con layer bool se cella aggiornata o meno
            if(localMap.atPosition(cloudPoint_counter_layer_name, localPosition) == 0) continue;

            if (std::isnan(localPosition.x()) || std::isnan(localPosition.y()) || std::isnan(layer_value)){
                ROS_WARN("Skipping local position with NaN value: x = %f, y = %f, z = %f", localPosition.x(), localPosition.y(), layer_value);
                continue;
            }

            // Convert local to global positions
            geometry_msgs::PointStamped localPoint, globalPoint;
            localPoint.point.x = localPosition.x();
            localPoint.point.y = localPosition.y();
            localPoint.point.z = layer_value;

            try {
                tf2::doTransform(localPoint, globalPoint, transform);
                max.point.x = std::max(max.point.x, globalPoint.point.x);
                max.point.y = std::max(max.point.y, globalPoint.point.y);
                min.point.x = std::min(min.point.x, globalPoint.point.x);
                min.point.y = std::min(min.point.y, globalPoint.point.y);

                local_global_points_mapping.push_back(std::make_tuple(globalPoint, localPoint));

            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
            }   
        }

        grid_map::Position maxGlobalPosition(max.point.x, max.point.y);
        grid_map::Position minGlobalPosition(min.point.x, min.point.y);

        if (!globalMap_.isInside(maxGlobalPosition) || !globalMap_.isInside(minGlobalPosition)) {
            enlargeMap(maxGlobalPosition, minGlobalPosition);
        }

        return local_global_points_mapping;
    }

    void updateGlobalMapKalman(grid_map::GridMap& localMap, const std::string& mean_layer_name, const std::string& variance_layer_name, geometry_msgs::TransformStamped transform, std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> local_global_points_mapping){
        
        for(const auto& t : local_global_points_mapping){

            geometry_msgs::PointStamped global_point = std::get<0>(t);  
            geometry_msgs::PointStamped local_point = std::get<1>(t);

            grid_map::Position globalPosition(global_point.point.x, global_point.point.y);
            grid_map::Position localPosition(local_point.point.x, local_point.point.y);

            try {
                float global_elevation = globalMap_.atPosition(mean_layer_name, globalPosition);
                float global_variance = globalMap_.atPosition(variance_layer_name, globalPosition);

                float local_elevation = global_point.point.z;
                float local_variance = localMap.atPosition(variance_layer_name, localPosition);

                float kalman_gain = global_variance / (global_variance + local_variance);

                float updated_elevation = global_elevation + kalman_gain * (local_elevation - global_elevation);
                float updated_variance = (1 - kalman_gain) * global_variance;
                
                globalMap_.atPosition(mean_layer_name, globalPosition) = updated_elevation;
                globalMap_.atPosition(variance_layer_name, globalPosition) = updated_variance;
            }
            catch (const std::out_of_range& e) {
                ROS_ERROR("Error updating elevation global map using Kalman");
            }
        }

        publish_gridmap();
    }

    void updateGlobalMap(grid_map::GridMap& localMap, const std::vector<std::string>& layers, const geometry_msgs::TransformStamped* transform){
        
        if (transform != nullptr) {
            grid_map::Position newPosition(transform->transform.translation.x, transform->transform.translation.y);
            localMap.setPosition(newPosition);
        }
        
        for (grid_map::GridMapIterator it(localMap); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            grid_map::Position position;
            localMap.getPosition(index, position);

            try {
                for (const auto& layer : layers) {
    
                    const float& value = localMap.at(layer, index);

                    if (std::isnan(value)) {
                        throw std::runtime_error("Value is NaN");
                    }

                    globalMap_.atPosition(layer, position) = value;
                }
            } catch (const std::exception& e) {
                ROS_WARN("Skipping position [%f, %f]: %s", position.x(), position.y(), e.what());
            }
        }
    }

    void enlargeMap(grid_map::Position& max_pos, grid_map::Position& min_pos){

        grid_map::Length currentLength = globalMap_.getLength();
        grid_map::Position currentPosition = globalMap_.getPosition();

        if(!(std::fmod(min_pos.x(), cell_size) == 0)){
            min_pos.x() = floor(min_pos.x() / cell_size) * cell_size;
        }
        if(!(std::fmod(min_pos.y(), cell_size) == 0)){
            min_pos.y() = floor(min_pos.y() / cell_size) * cell_size;
        }
        if(!(std::fmod(max_pos.x(), cell_size) == 0)){
            max_pos.x() = ceil(max_pos.x() / cell_size) * cell_size;
        }
        if(!(std::fmod(max_pos.y(), cell_size) == 0)){
            max_pos.y() = ceil(max_pos.y() / cell_size) * cell_size;
        }

        double newMinX = std::min(min_pos.x() - cell_size/2, currentPosition.x() - currentLength.x() / 2.0);
        double newMaxX = std::max(max_pos.x() + cell_size/2, currentPosition.x() + currentLength.x() / 2.0);
        double newMinY = std::min(min_pos.y() - cell_size/2, currentPosition.y() - currentLength.y() / 2.0);
        double newMaxY = std::max(max_pos.y() + cell_size/2, currentPosition.y() + currentLength.y() / 2.0);

        double newLengthX = newMaxX - newMinX;
        double newLengthY = newMaxY - newMinY;
        grid_map::Length newLength(newLengthX, newLengthY);
        grid_map::Position newPosition((newMinX + newMaxX) / 2.0, (newMinY + newMaxY) / 2.0);

        grid_map::GridMap newMap = grid_map::GridMap(globalMap_.getLayers());
        newMap.setGeometry(newLength, globalMap_.getResolution(), newPosition);

        //TODO: generalizzare

        for (const auto& layer : newMap.getLayers()) {
            if (layer.c_str() == elevation_variance_layer_name || layer.c_str() == surface_orientation_variance_layer_name || layer.c_str() == obstacles_variance_layer_name) newMap[layer].setConstant(initial_variance);
            else if(layer.c_str() == color_layer_name) newMap[layer].setConstant(robot4ws_mapping::Unknown);
            else newMap[layer].setZero();
        }
        
        for (grid_map::GridMapIterator iterator(globalMap_); !iterator.isPastEnd(); ++iterator) {
            grid_map::Position oldPosition;
            globalMap_.getPosition(*iterator, oldPosition);
            if (newMap.isInside(oldPosition)) {
                grid_map::Index newIndex;
                newMap.getIndex(oldPosition, newIndex);
                for (const auto& layer : globalMap_.getLayers()) {
                    newMap.at(layer, newIndex) = globalMap_.at(layer, *iterator);
                }
            }
        }

        last_center_position_global_frame = newPosition;
        globalMap_ = newMap;
    }

    geometry_msgs::TransformStamped get_transform_msg(const nav_msgs::Odometry& msg, bool foot2lidar){
        const geometry_msgs::Pose& pose = msg.pose.pose;

        tf2::Vector3 translation(pose.position.x, pose.position.y, pose.position.z);
        tf2::Quaternion rotation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        tf2::Transform odom_transform;
        odom_transform.setOrigin(translation);
        odom_transform.setRotation(rotation);

        geometry_msgs::TransformStamped gridmap_to_lidar_transform_msg;
        gridmap_to_lidar_transform_msg.header.stamp = msg.header.stamp;
        gridmap_to_lidar_transform_msg.child_frame_id = msg.header.frame_id;

        if(foot2lidar){       
            gridmap_to_lidar_transform_msg.transform = tf2::toMsg(odom_transform * foot2lidar_transform);
        }else{
            gridmap_to_lidar_transform_msg.transform = tf2::toMsg(odom_transform);
        }

        return gridmap_to_lidar_transform_msg;
    }

    void publish_global_gridmap_update(const robot4ws_mapping::GridMapUpdateMsg::ConstPtr& msg){

        grid_map::GridMap local_map;
        grid_map::GridMapRosConverter::fromMessage(msg->local_gridmap, local_map);

        const geometry_msgs::Pose& pose = msg->odom.pose.pose;
        double center_x = pose.position.x;
        double center_y = pose.position.y;

        grid_map::Position local_center(center_x, center_y);
        grid_map::Length local_size = local_map.getLength();

        grid_map::GridMap global_update;
        global_update.setFrameId(globalMap_.getFrameId());
        global_update.setTimestamp(globalMap_.getTimestamp());
        bool is_success;
        global_update = globalMap_.getSubmap(local_center, local_size, is_success);

        if(!is_success) ROS_ERROR("Requested submap of globalMap is failed");

        grid_map_msgs::GridMap global_map_update_msg;
        grid_map::GridMapRosConverter::toMessage(global_update, global_map_update_msg);

        robot4ws_mapping::GridMapUpdateMsg global_update_msg;

        global_update_msg.header.stamp = ros::Time::now();
        global_update_msg.header.frame_id = grid_map_global_update_topic_name;

        global_update_msg.local_gridmap = global_map_update_msg;
        
        global_update_msg.odom = msg->odom;
        grid_map_global_update_pub.publish(global_update_msg);

        //ROS_INFO("Published global map update centered at (%.2f, %.2f)", center_x, center_y);
    }

    void updateColorGlobalMap(grid_map::GridMap& localMap, const geometry_msgs::TransformStamped& transform){

        checkEnlargeMap(localMap, transform);

        for (grid_map::GridMapIterator it(localMap); !it.isPastEnd(); ++it){
            grid_map::Index index(*it);
            grid_map::Position localPosition;
            localMap.getPosition(index, localPosition);

            geometry_msgs::PointStamped localPoint, globalPoint;
            localPoint.point.x = localPosition.x();
            localPoint.point.y = localPosition.y();

            grid_map::Index globalIndex;

            try {
                tf2::doTransform(localPoint, globalPoint, transform);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
            }   

            grid_map::Position globalPosition(globalPoint.point.x, globalPoint.point.y);

            if(!globalMap_.isInside(globalPosition) || !globalMap_.getIndex(globalPosition, globalIndex)){
                ROS_WARN("Failed conversion local color map into global");
                continue;
            }

            try {
                float valueLocal = localMap.at(color_layer_name, index);

                if (std::isnan(valueLocal)) {
                    throw std::runtime_error("Value is NaN");
                }

                int colorIdLocal = static_cast<int>(valueLocal);
                if (colorIdLocal == robot4ws_mapping::Unknown) {
                    continue; 
                }

                std::vector<int>& countsVec = colorCounts_[globalIndex];

                if (countsVec.empty()) {
                    countsVec.resize(robot4ws_mapping::NUM_COLORS, 0);
                }

                if (colorIdLocal >= 0 && colorIdLocal < robot4ws_mapping::NUM_COLORS) {
                    countsVec[colorIdLocal]++;
                }

                int bestCount = -1;
                int bestColor = robot4ws_mapping::Unknown;
                for (int c = 0; c < robot4ws_mapping::NUM_COLORS; ++c){
                    if (countsVec[c] > bestCount) {
                        bestCount  = countsVec[c];
                        bestColor  = c;
                    }
                }

                globalMap_.at(color_layer_name, globalIndex) = static_cast<float>(bestColor);
            }
            catch (const std::exception& e) {
                ROS_WARN("Skipping position [%.2f, %.2f]: %s",
                         globalPosition.x(), globalPosition.y(), e.what());
            }
        }
    }

    /* void publishColorImage(grid_map::GridMap localMap) {
        // Creare le matrici per i singoli colori (R, G, B)
        cv::Mat red_image, green_image, blue_image;

        // Convertire i layer `red`, `green`, `blue` in immagini OpenCV separate
        grid_map::GridMapCvConverter::toImage<unsigned short, 1>(
            localMap, "red", CV_16UC1, 0.0f, 1.0f, red_image);
        grid_map::GridMapCvConverter::toImage<unsigned short, 1>(
            localMap, "green", CV_16UC1, 0.0f, 1.0f, green_image);
        grid_map::GridMapCvConverter::toImage<unsigned short, 1>(
            localMap, "blue", CV_16UC1, 0.0f, 1.0f, blue_image);

        // Convertire le immagini a 16 bit in 8 bit per ogni canale
        cv::Mat red_image_8bit, green_image_8bit, blue_image_8bit;
        red_image.convertTo(red_image_8bit, CV_8UC1, 1.0 / 256.0);   // Ridimensionare la gamma
        green_image.convertTo(green_image_8bit, CV_8UC1, 1.0 / 256.0);
        blue_image.convertTo(blue_image_8bit, CV_8UC1, 1.0 / 256.0);

        // Creare l'immagine RGB combinando i tre canali
        cv::Mat color_image;
        std::vector<cv::Mat> channels = {blue_image_8bit, green_image_8bit, red_image_8bit}; // OpenCV usa l'ordine BGR
        cv::merge(channels, color_image);

        // Convertire l'immagine OpenCV in un messaggio ROS
        sensor_msgs::Image ros_image;
        cv_bridge::CvImage cv_bridge_image;
        cv_bridge_image.encoding = "bgr8"; // Specificare l'encoding RGB (BGR per OpenCV)
        cv_bridge_image.image = color_image;
        cv_bridge_image.toImageMsg(ros_image);

        // Pubblicare l'immagine
        ros_image.header.stamp = ros::Time::now();
        //ros_image.header.frame_id = localMap.getFrameId();
        ros_image.header.frame_id = map_frame_id;
        image_pub_.publish(ros_image);
    } */

    void checkEnlargeMap(const grid_map::GridMap& localMap, const geometry_msgs::TransformStamped& transform){

        geometry_msgs::PointStamped max,min;
        max.point.x = -std::numeric_limits<double>::infinity();
        max.point.y = -std::numeric_limits<double>::infinity();
        min.point.x = std::numeric_limits<double>::infinity();
        min.point.y = std::numeric_limits<double>::infinity();

        grid_map::Position top_left, top_right, bottom_left, bottom_right;
        localMap.getPosition(grid_map::Index(0, 0), top_left);
        localMap.getPosition(grid_map::Index(0, localMap.getSize()(1) - 1), top_right);
        localMap.getPosition(grid_map::Index(localMap.getSize()(0) - 1, 0), bottom_left);
        localMap.getPosition(grid_map::Index(localMap.getSize()(0) - 1, localMap.getSize()(1) - 1), bottom_right);
        
        geometry_msgs::PointStamped local_points[4], global_points[4];

        local_points[0].point.x = top_left.x();
        local_points[0].point.y = top_left.y();
        local_points[0].point.z = 0.0;

        local_points[1].point.x = top_right.x();
        local_points[1].point.y = top_right.y();
        local_points[1].point.z = 0.0;

        local_points[2].point.x = bottom_left.x();
        local_points[2].point.y = bottom_left.y();
        local_points[2].point.z = 0.0;

        local_points[3].point.x = bottom_right.x();
        local_points[3].point.y = bottom_right.y();
        local_points[3].point.z = 0.0;

        try {
            for (int i = 0; i < 4; ++i){
                tf2::doTransform(local_points[i], global_points[i], transform);
               
                max.point.x = std::max(max.point.x, global_points[i].point.x);
                max.point.y = std::max(max.point.y, global_points[i].point.y);
                min.point.x = std::min(min.point.x, global_points[i].point.x);
                min.point.y = std::min(min.point.y, global_points[i].point.y);
            }
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform exception: %s", ex.what());
            return;
        }

        grid_map::Position maxGlobalPosition(max.point.x, max.point.y);
        grid_map::Position minGlobalPosition(min.point.x, min.point.y);

        if (!globalMap_.isInside(maxGlobalPosition) || !globalMap_.isInside(minGlobalPosition)) {
            enlargeMap(maxGlobalPosition, minGlobalPosition);
        }
    }

    bool handle_surface_normal_request(robot4ws_mapping::get_surface_normal::Request& req, robot4ws_mapping::get_surface_normal::Response& res){
        //ROS_INFO("Position request: (%f, %f)", req.position.x, req.position.y);

        grid_map::Position position(req.position.x, req.position.y);
        if(globalMap_.isInside(position)){
            geometry_msgs::Vector3 normal_vector;
            normal_vector.x = globalMap_.atPosition(surface_orientation_x_layer_name, position);        
            normal_vector.y = globalMap_.atPosition(surface_orientation_y_layer_name, position);
            normal_vector.z = globalMap_.atPosition(surface_orientation_z_layer_name, position);        
            res.normal = normal_vector;

            grid_map::Index idx;
            if (globalMap_.getIndex(position, idx)){
                res.map_cell_x = idx.x();
                res.map_cell_y = idx.y();
                return true;
            } else {
                ROS_WARN("multi_layer_map_node: Unable to convert position to index, position out of bounds.");
                return false;
            }

        } else{
            ROS_WARN("multi_layer_map_node: get_surface_normal service [Position request out of map range]");
            return false;
        }
    }

    void publishColorMapImage(){
        // Creare un'immagine OpenCV a 3 canali (BGR)
        grid_map::Size size = globalMap_.getSize();
        int numRows = size(0);
        int numCols = size(1);
        cv::Mat color_image(numRows, numCols, CV_8UC3, cv::Scalar(0, 0, 0)); // Default nero

        // Iterare su tutte le celle della mappa
        for (grid_map::GridMapIterator it(globalMap_); !it.isPastEnd(); ++it) {
            grid_map::Index index(*it);
            grid_map::Position position;
            globalMap_.getPosition(index, position);

            try {
                float value = globalMap_.at(color_layer_name, index);
                robot4ws_mapping::ColorId colorId = static_cast<robot4ws_mapping::ColorId>(static_cast<int>(value));

                cv::Vec3b bgrColor = robot4ws_mapping::colorIdToBGR(colorId);

                int row = index(0);
                int col = index(1);
                color_image.at<cv::Vec3b>(row, col) = bgrColor;
            } catch (const std::exception& e) {
                ROS_WARN("Skipping cell [%d, %d]: %s", index(0), index(1), e.what());
            }
        }

        // Convertire l'immagine OpenCV in un messaggio ROS
        sensor_msgs::Image ros_image;
        cv_bridge::CvImage cv_bridge_image;
        cv_bridge_image.encoding = "bgr8"; // Specificare l'encoding RGB (BGR per OpenCV)
        cv_bridge_image.image = color_image;
        cv_bridge_image.toImageMsg(ros_image);

        // Pubblicare l'immagine
        ros_image.header.stamp = ros::Time::now();
        ros_image.header.frame_id = globalMap_.getFrameId();
        image_pub_.publish(ros_image);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "elevation_node");

    MultiLayerMapNode multi_layer_map;

    ros::spin();
    return 0;
}
