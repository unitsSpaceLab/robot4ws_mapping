#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <robot4ws_mapping/GridMapUpdateMsg.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


class MultiLayerMapNode
{
public:
    MultiLayerMapNode(){   
        load_params();
        load_robot_static_tf();
        initial_variance = 100;

        init_global_map();

        pub_grid_map_update_sub = nh_.subscribe(grid_map_update_topic_name, 10, &MultiLayerMapNode::grid_map_update_callback, this);

        grid_map_pub = nh_.advertise<grid_map_msgs::GridMap>(grid_map_topic_name, 1, true);
        grid_map_global_update_pub = nh_.advertise<robot4ws_mapping::GridMapUpdateMsg>(grid_map_global_update_topic_name, 1, true);
    }
    
    void grid_map_update_callback(const robot4ws_mapping::GridMapUpdateMsg::ConstPtr& msg){

        grid_map::GridMap local_map;
        geometry_msgs::TransformStamped transform_msg = get_transform_msg(msg->odom);
        grid_map::GridMapRosConverter::fromMessage(msg->local_gridmap, local_map);
        std::string layer_id = msg->header.frame_id;

        if (layer_id == elevation_layer_name){
            std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> transformed_point_list = applyTransform(local_map, elevation_layer_name, transform_msg);
            updateGlobalMapKalman(local_map, elevation_layer_name, elevation_variance_layer_name, transform_msg, transformed_point_list);
            publish_global_gridmap_update(msg);
        }
        else if(layer_id == surface_orientation_layer_name){
            std::vector<std::string> layers = {surface_orientation_x_layer_name, surface_orientation_y_layer_name, surface_orientation_z_layer_name, surface_orientation_variance_layer_name};
            updateGlobalMap(local_map, layers);
        }
        else if(layer_id == obstacles_layer_name){
            std::vector<std::string> layers = {obstacles_layer_name, obstacles_variance_layer_name};
            updateGlobalMap(local_map, layers);
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
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub;
    ros::Subscriber pub_grid_map_update_sub;
    ros::Publisher grid_map_pub;
    ros::Publisher grid_map_global_update_pub;

    //GridMap attributes:
    grid_map::GridMap globalMap_;
    grid_map::Position last_center_position_global_frame;
    std::string grid_map_frame_id;
    
    double global_map_size, local_map_size;
    double cell_size, initial_variance;
    
    //Layer names
    std::string elevation_layer_name, elevation_variance_layer_name;
    std::string cloudPoint_counter_layer_name;
    std::string obstacles_layer_name, obstacles_variance_layer_name, surface_orientation_layer_name;
    std::string surface_orientation_x_layer_name, surface_orientation_y_layer_name, surface_orientation_z_layer_name, surface_orientation_variance_layer_name;

    //Topics
    std::string odom_topic_name, grid_map_update_topic_name, grid_map_topic_name, grid_map_global_update_topic_name;
    
    nav_msgs::Odometry actual_odom_msg;

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

        //load topic names
        if (! nh_.getParam("gridmap/odom_topic_name",odom_topic_name))
        {
            odom_topic_name = "/odom";
            ROS_WARN_STREAM("Parameter [gridmap/odom_topic_name] not found. Using default value: " << odom_topic_name);
        }
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
            enlargeMap(globalMap_, maxGlobalPosition, minGlobalPosition);
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
                ROS_ERROR("Error updating global map: %s", e.what());
            }
        }

        publish_gridmap();
    }
    
    void updateGlobalMap(grid_map::GridMap& localMap, const std::vector<std::string>& layers){
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

    void enlargeMap(grid_map::GridMap& map, grid_map::Position& max_pos, grid_map::Position& min_pos) {

        grid_map::Length currentLength = map.getLength();
        grid_map::Position currentPosition = map.getPosition();

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

        grid_map::GridMap newMap = grid_map::GridMap(map.getLayers());
        newMap.setGeometry(newLength, map.getResolution(), newPosition);

        //TODO: generalizzare

        for (const auto& layer : newMap.getLayers()) {
            if (layer.c_str() == elevation_variance_layer_name || layer.c_str() == surface_orientation_variance_layer_name || layer.c_str() == obstacles_variance_layer_name) newMap[layer].setConstant(initial_variance);
            else newMap[layer].setZero();
        }
        
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
            grid_map::Position oldPosition;
            map.getPosition(*iterator, oldPosition);
            if (newMap.isInside(oldPosition)) {
                grid_map::Index newIndex;
                newMap.getIndex(oldPosition, newIndex);
                for (const auto& layer : map.getLayers()) {
                    newMap.at(layer, newIndex) = map.at(layer, *iterator);
                }
            }
        }

        last_center_position_global_frame = newPosition;
        map = newMap;

        for (const auto& layer : map.getLayers()) {
            for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
                float value = map.at(layer, *it);
                // ROS_INFO("Layer '%s', Valore (dopo impostazione): %f", layer.c_str(), value);
            }
        }
    }

    geometry_msgs::TransformStamped get_transform_msg(const nav_msgs::Odometry& msg){
        const geometry_msgs::Pose& pose = msg.pose.pose;

        tf2::Vector3 translation(pose.position.x, pose.position.y, pose.position.z);
        tf2::Quaternion rotation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        tf2::Transform odom_transform;
        odom_transform.setOrigin(translation);
        odom_transform.setRotation(rotation);

        tf2::Transform transform = odom_transform * foot2lidar_transform;

        geometry_msgs::TransformStamped gridmap_to_lidar_transform_msg;
        gridmap_to_lidar_transform_msg.header.stamp = msg.header.stamp;
        gridmap_to_lidar_transform_msg.child_frame_id = msg.header.frame_id;
        gridmap_to_lidar_transform_msg.transform = tf2::toMsg(transform);
        return gridmap_to_lidar_transform_msg;
    }

    void publish_global_gridmap_update(const robot4ws_mapping::GridMapUpdateMsg::ConstPtr& msg) {

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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "elevation_node");

    MultiLayerMapNode multi_layer_map;

    ros::spin();
    return 0;
}
