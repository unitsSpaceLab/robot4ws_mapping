#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


class multi_layer_map{
public:
    multi_layer_map() {
        load_params();
        load_robot_static_tf();

        elevation_layer_name = "elevation";
        elevation_variance_layer_name = "elevation_variance";
        obstacles_layer_name = "obstacles";

        // Global map init
        globalMap_.setFrameId(grid_map_frame_id);
        globalMap_.setGeometry(grid_map::Length(global_map_size, global_map_size), cell_size);
        globalMap_.add(elevation_layer_name, 0.0);
        globalMap_.add(elevation_variance_layer_name, 1e6);
        globalMap_.add(obstacles_layer_name, 0.0);

        // Subscriber per PointCloud2
        cloud_sub = nh_.subscribe(lidar_topic_name, 10, &multi_layer_map::pointCloudCallback, this);
        odom_sub = nh_.subscribe(odom_topic_name, 5, &multi_layer_map::odom_callback, this);
        costmap_sub = nh_.subscribe(costmap2d_topic_name, 5, &multi_layer_map::costmap_callback, this);
        grid_map_pub = nh_.advertise<grid_map_msgs::GridMap>(grid_map_topic_name, 1, true);

        pose_received = false;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){

        if (!pose_received) {
            ROS_WARN("No pose data received yet");
            return;
        }

        geometry_msgs::TransformStamped actual_transform = odom_transform_msg;

        // Converti PointCloud2 in GridMap locale
        grid_map::GridMap localMap({elevation_layer_name, "cloudPoint_counter", elevation_variance_layer_name});
        localMap.setGeometry(grid_map::Length(local_map_size, local_map_size), cell_size);
        localMap[elevation_layer_name].setZero();
        localMap["cloudPoint_counter"].setZero();
        localMap[elevation_variance_layer_name].setConstant(1e6);

        // Codice per convertire PointCloud2 in GridMap
        if (cloud != nullptr) {
            for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x"), iter_y(*cloud, "y"), iter_z(*cloud, "z");
                 iter_x != iter_x.end();
                 ++iter_x, ++iter_y, ++iter_z)
            {
                if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
                    ROS_WARN("Skipping cloud point with NaN value: x = %f, y = %f, z = %f", *iter_x, *iter_y, *iter_z);
                    continue;
                }

                // skipping cloud points above the heigth treshold
                if(*iter_z > pc_height_threshold) continue;

                grid_map::Position position(*iter_x, *iter_y);
                if (localMap.isInside(position)) {
                    localMap.atPosition("cloudPoint_counter", position)+=1;
                    int cell_count = localMap.atPosition("cloudPoint_counter", position);
                    double old_mean = localMap.atPosition(elevation_layer_name, position);
                    double old_variance = localMap.atPosition(elevation_variance_layer_name, position);

                    double new_mean = (old_mean * (cell_count-1) + *iter_z)/cell_count;

                    localMap.atPosition(elevation_layer_name, position) = new_mean;  
                    localMap.atPosition(elevation_variance_layer_name, position) = lidar_variance;
                    }
            }
        }

        std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> transformed_point_tuple_list = applyTransform(localMap, elevation_layer_name, elevation_variance_layer_name, actual_transform);
        updateGlobalMapKalman(localMap, elevation_layer_name, elevation_variance_layer_name, actual_transform, transformed_point_tuple_list);
    }

    std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> applyTransform(grid_map::GridMap& localMap, const std::string& mean_layer_name, const std::string& variance_layer_name, geometry_msgs::TransformStamped transform){
        geometry_msgs::PointStamped max,min;
        max.point.x = -1000000;
        max.point.y = -1000000;
        min.point.x = 1000000;
        min.point.y = 1000000;

        std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> local_global_points_mapping;

        // Trasforma la mappa locale nella mappa globale
        for (grid_map::GridMapIterator it(localMap); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            const float& layer_value = localMap.at(mean_layer_name, index);

            grid_map::Position localPosition;
            localMap.getPosition(index, localPosition);

            //skip empty local cells
            if(localMap.atPosition("cloudPoint_counter", localPosition) == 0) continue;

            if (std::isnan(localPosition.x()) || std::isnan(localPosition.y()) || std::isnan(layer_value)){
                ROS_WARN("Skipping local position with NaN value: x = %f, y = %f, z = %f", localPosition.x(), localPosition.y(), layer_value);
                continue;
            }

            // Converti la posizione locale in globale
            geometry_msgs::PointStamped localPoint, globalPoint;
            localPoint.point.x = localPosition.x();
            localPoint.point.y = localPosition.y();
            localPoint.point.z = layer_value;

            // Usa la trasformazione salvata
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

        // fillBordersWithFive(globalMap_);
        // pub_tf(last_center_position_global_frame);
        publish_gridmap();
    }

    void updateGlobalMap(grid_map::GridMap& localMap, const std::string& layer_name, geometry_msgs::TransformStamped transform){
        geometry_msgs::PointStamped max,min;
        max.point.x = -1000000;
        max.point.y = -1000000;
        min.point.x = 1000000;
        min.point.y = 1000000;

        std::list <geometry_msgs::PointStamped> globalPoints;

        // Trasforma la mappa locale nella mappa globale
        for (grid_map::GridMapIterator it(localMap); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            const float& layer_value = localMap.at(layer_name, index);
            grid_map::Position localPosition;
            localMap.getPosition(index, localPosition);

            if (std::isnan(localPosition.x()) || std::isnan(localPosition.y()) || std::isnan(layer_value)){
                ROS_WARN("Skipping local position with NaN value: x = %f, y = %f, z = %f", localPosition.x(), localPosition.y(), layer_value);
                continue;
            }

            // Converti la posizione locale in globale
            geometry_msgs::PointStamped localPoint, globalPoint;
            //localPoint.header.frame_id = cloud->header.frame_id;
            localPoint.point.x = localPosition.x();
            localPoint.point.y = localPosition.y();
            localPoint.point.z = layer_value;

            // Usa la trasformazione salvata
            try {
                tf2::doTransform(localPoint, globalPoint, transform);
                max.point.x = std::max(max.point.x, globalPoint.point.x);
                max.point.y = std::max(max.point.y, globalPoint.point.y);
                min.point.x = std::min(min.point.x, globalPoint.point.x);
                min.point.y = std::min(min.point.y, globalPoint.point.y);

                globalPoints.push_back(globalPoint);

            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                return;
            }   
        }

        grid_map::Position maxGlobalPosition(max.point.x, max.point.y);
        grid_map::Position minGlobalPosition(min.point.x, min.point.y);

        if (!globalMap_.isInside(maxGlobalPosition) || !globalMap_.isInside(minGlobalPosition)) {
            //ROS_WARN("Position max(%f, %f) or position min(%f, %f) is out of range. Expanding the map.", maxGlobalPosition.x(), maxGlobalPosition.y(), minGlobalPosition.x(), minGlobalPosition.y());
            enlargeMap(globalMap_, maxGlobalPosition, minGlobalPosition);
        }

        for(geometry_msgs::PointStamped global_point : globalPoints){  
            grid_map::Position globalPosition(global_point.point.x, global_point.point.y);
            try {
                    globalMap_.atPosition(layer_name, globalPosition) = global_point.point.z;
                } catch (const std::out_of_range& e) {
                    ROS_ERROR("Error updating global map: %s", e.what());
                }
        }

        // fillBordersWithFive(globalMap_);
        // pub_tf(last_center_position_global_frame);
        publish_gridmap();
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        const geometry_msgs::Pose& pose = msg->pose.pose;

        tf2::Vector3 translation(pose.position.x, pose.position.y, pose.position.z);
        tf2::Quaternion rotation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        tf2::Transform odom_transform;
        odom_transform.setOrigin(translation);
        odom_transform.setRotation(rotation);

        tf2::Transform transform = odom_transform * foot2lidar_transform;
        
        odom_transform_msg.header.stamp = msg->header.stamp;
        odom_transform_msg.child_frame_id = msg->header.frame_id;
        odom_transform_msg.transform = tf2::toMsg(transform);

        pose_received = true;
    }

    void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        grid_map::GridMap localMap;
        grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "obstacles", localMap);

        //TODO: Genera buchi nella mappa, provare a fixare che semplifica molto.
        //globalMap_.addDataFrom(localMap, "obstacles", true, true);

        geometry_msgs::TransformStamped odom_transform_message;
        odom_transform_message.transform.translation.x = 0.0;
        odom_transform_message.transform.translation.y = 0.0;
        odom_transform_message.transform.translation.z = 0.0;

        odom_transform_message.transform.rotation.x = 0.0;
        odom_transform_message.transform.rotation.y = 0.0;
        odom_transform_message.transform.rotation.z = 0.0;
        odom_transform_message.transform.rotation.w = 1.0;
        updateGlobalMap(localMap, "obstacles", odom_transform_message);
    }

    void publish_gridmap()
    {
        globalMap_.setTimestamp(ros::Time::now().toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(globalMap_, message);
        message.info.header.frame_id = grid_map_frame_id;
        grid_map_pub.publish(message);
        //ROS_INFO("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
    }

    void enlargeMap(grid_map::GridMap& map, grid_map::Position& max_pos, grid_map::Position& min_pos) {
        // Ottieni i limiti attuali della mappa
        grid_map::Length currentLength = map.getLength();
        grid_map::Position currentPosition = map.getPosition();

        //arrotondiamo rispetto la grid_cell: per difetto le celle negative e per eccesso quelle positive
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

        // Calcola i nuovi limiti
        double newMinX = std::min(min_pos.x(), currentPosition.x() - currentLength.x() / 2.0);
        double newMaxX = std::max(max_pos.x(), currentPosition.x() + currentLength.x() / 2.0);
        double newMinY = std::min(min_pos.y(), currentPosition.y() - currentLength.y() / 2.0);
        double newMaxY = std::max(max_pos.y(), currentPosition.y() + currentLength.y() / 2.0);

        
        double newLengthX = newMaxX - newMinX;
        double newLengthY = newMaxY - newMinY;
        grid_map::Length newLength(newLengthX, newLengthY);
        grid_map::Position newPosition((newMinX + newMaxX) / 2.0, (newMinY + newMaxY) / 2.0);

        // Crea una nuova mappa con i nuovi limiti
        grid_map::GridMap newMap = grid_map::GridMap(map.getLayers());
        newMap.setGeometry(newLength, map.getResolution(), newPosition);

        //TODO: generalizzare

        for (const auto& layer : newMap.getLayers()) {
            if (layer.c_str() == elevation_variance_layer_name) newMap[layer].setConstant(1e6);
            else newMap[layer].setZero();
        }
        
        // Copia i dati dalla vecchia mappa alla nuova
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
        // Aggiorna la mappa originale
        map = newMap;

        for (const auto& layer : map.getLayers()) {
            for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
                float value = map.at(layer, *it);
                // ROS_INFO("Layer '%s', Valore (dopo impostazione): %f", layer.c_str(), value);
            }
        }
    }

    void load_params(){
        if (! nh_.getParam("gridmap/cell_size",cell_size))
        {
            cell_size = 0.2;
            ROS_WARN_STREAM("Parameter [gridmap/cell_size] not found. Using default value: " << cell_size);
        }
        if (! nh_.getParam("gridmap/global_map_size",global_map_size))
        {
            global_map_size = 5.0;
            ROS_WARN_STREAM("Parameter [gridmap/global_map_size] not found. Using default value: " << global_map_size);
        }
        if (! nh_.getParam("gridmap/local_map_size",local_map_size))
        {
            local_map_size = 5.0;
            ROS_WARN_STREAM("Parameter [gridmap/local_map_size] not found. Using default value: " << local_map_size);
        }
        if (! nh_.getParam("gridmap/point_cloud_max_height_threshold",pc_height_threshold))
        {
            pc_height_threshold = 2.0;
            ROS_WARN_STREAM("Parameter [gridmap/pc_height_threshold] not found. Using default value: " << pc_height_threshold);
        }
        if (! nh_.getParam("gridmap/lidar_variance",lidar_variance))
        {
            lidar_variance = 0.2;
            ROS_WARN_STREAM("Parameter [gridmap/lidar_variance] not found. Using default value: " << lidar_variance);
        }
        if (! nh_.getParam("gridmap/lidar_topic_name",lidar_topic_name))
        {
            lidar_topic_name = "/velodyne";
            ROS_WARN_STREAM("Parameter [gridmap/lidar_topic_name] not found. Using default value: " << lidar_topic_name);
        }
        if (! nh_.getParam("gridmap/odom_topic_name",odom_topic_name))
        {
            odom_topic_name = "/odom";
            ROS_WARN_STREAM("Parameter [gridmap/odom_topic_name] not found. Using default value: " << odom_topic_name);
        }
        if (! nh_.getParam("gridmap/costmap2d_topic_name",costmap2d_topic_name))
        {
            costmap2d_topic_name = "/costmap_2d_node/costmap/costmap";
            ROS_WARN_STREAM("Parameter [gridmap/costmap2d_topic_name] not found. Using default value: " << costmap2d_topic_name);
        }
        if (! nh_.getParam("gridmap/grid_map_frame_id",grid_map_frame_id))
        {
            grid_map_frame_id = "odom";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_frame_id] not found. Using default value: " << grid_map_frame_id);
        }
        /*if (!nh_.getParam("gridmap/lidar_pose/translation/x", lidar_pose.transform.translation.x) ||
            !nh_.getParam("gridmap/lidar_pose/translation/y", lidar_pose.transform.translation.y) ||
            !nh_.getParam("gridmap/lidar_pose/translation/z", lidar_pose.transform.translation.z) ||
            !nh_.getParam("gridmap/lidar_pose/rotation/x", lidar_pose.transform.rotation.x) ||
            !nh_.getParam("gridmap/lidar_pose/rotation/y", lidar_pose.transform.rotation.y) ||
            !nh_.getParam("gridmap/lidar_pose/rotation/z", lidar_pose.transform.rotation.z) ||
            !nh_.getParam("gridmap/lidar_pose/rotation/w", lidar_pose.transform.rotation.w)) {
                lidar_pose.transform.translation.x = 0;
                lidar_pose.transform.translation.y = 0;
                lidar_pose.transform.translation.z = 0;
                lidar_pose.transform.rotation.x = 0;
                lidar_pose.transform.rotation.y = 0;
                lidar_pose.transform.rotation.z = 0;
                lidar_pose.transform.rotation.w = 1;
                ROS_WARN_STREAM("Parameter [gridmap/lidar_pose] not found. Using default value: " << lidar_pose);
        }*/
       /*  if (! nh_.getParam("gridmap/base_link_frame_id",base_link_frame_id))
        {
            base_link_frame_id = "base_link";
            ROS_WARN_STREAM("Parameter [gridmap/base_link_frame_id] not found. Using default value: " << base_link_frame_id);
        } */
        //TODO: no needed?
        /* if (! nh_.getParam("gridmap/base_link_frame_id_start",base_link_frame_id_start))
        {
            base_link_frame_id_start = "base_link_start";
            ROS_WARN_STREAM("Parameter [gridmap/base_link_frame_id_start] not found. Using default value: " << base_link_frame_id_start);
        } */
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
        if (! nh_.getParam("gridmap/grid_map_topic_name",grid_map_topic_name))
        {
            grid_map_topic_name = "grid_map";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_topic_name] not found. Using default value: " << grid_map_topic_name);
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

private:
    //Ros attributes & topics:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber costmap_sub;
    ros::Publisher grid_map_pub; 
    
    std::string grid_map_topic_name;
    std::string lidar_topic_name;
    std::string odom_topic_name;
    std::string costmap2d_topic_name;

    //GridMap attributes:
    grid_map::GridMap globalMap_;
    grid_map::Position last_center_position_global_frame;
    std::string grid_map_frame_id;
    
    double global_map_size;
    double local_map_size;
    double cell_size;
    
    std::string elevation_layer_name;
    std::string elevation_variance_layer_name;
    std::string obstacles_layer_name;

    double pc_height_threshold;
    bool pose_received;
    
    //Robot related params:
    double lidar_variance;
    std::string lidar_frame_id;
    std::string foot_print_frame_id;
    tf2::Transform foot2lidar_transform;
    geometry_msgs::TransformStamped odom_transform_msg;
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "p2g_node");
    multi_layer_map p2g_node;
    ros::spin();
    return 0;
}