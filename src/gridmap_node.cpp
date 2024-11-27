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

class Pointcloud_to_gridmap{
public:
    Pointcloud_to_gridmap() {
        cell_size = 0.5;
        pc_height_threshold = 2; //treshold for discarding too high cloud points
        lidar_variance = 0.2;

        elevation_variance_layer_name = "elevation_variance";

        // Inizializza la mappa globale
        globalMap_.setFrameId("grid_map");
        globalMap_.setGeometry(grid_map::Length(5.0, 5.0), cell_size);
        globalMap_.add("elevation", 0.0);
        globalMap_.add(elevation_variance_layer_name, 1e6);
        globalMap_.add("obstacles", 0.0);

        // Subscriber per PointCloud2
        cloud_sub = nh_.subscribe("/velodyne_points", 10, &Pointcloud_to_gridmap::pointCloudCallback, this);
        odom_sub = nh_.subscribe("/lio_sam/mapping/odometry", 5, &Pointcloud_to_gridmap::odom_callback, this);
        //costmap_sub = nh_.subscribe("/costmap_2d_node/costmap/costmap", 5, &Pointcloud_to_gridmap::costmap_callback, this);
        grid_map_pub = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

        pose_received = false;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){

        if (!pose_received) {
            ROS_WARN("No pose data received yet");
            return;
        }

        geometry_msgs::TransformStamped actual_transform = transformStamped;

        // Converti PointCloud2 in GridMap locale
        grid_map::GridMap localMap({"elevation", "cloudPoint_counter", "elevation_variance"});
        localMap.setGeometry(grid_map::Length(5.0, 5.0), cell_size);
        localMap["elevation"].setZero();
        //localMap["elevation"].setConstant(NAN);
        localMap["cloudPoint_counter"].setZero();
        localMap["elevation_variance"].setConstant(1e6);

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
                
                // z_values.push_back(*iter_z)
                
                grid_map::Position position(*iter_x, *iter_y);
                if (localMap.isInside(position)) {
                    localMap.atPosition("cloudPoint_counter", position)+=1;
                    int cell_count = localMap.atPosition("cloudPoint_counter", position);
                    double old_mean = localMap.atPosition("elevation", position);
                    double old_variance = localMap.atPosition("elevation_variance", position);

                    double new_mean = (old_mean * (cell_count-1) + *iter_z)/cell_count;
                
                    localMap.atPosition("elevation", position) = new_mean;  
                    
                    //localMap.atPosition("elevation_variance", position) = (old_variance + (*iter_z - old_mean) * (*iter_z - new_mean))/cell_count;
                    localMap.atPosition("elevation_variance", position) = 9e-6;
                    
                    }
            }
        }

        updateGlobalMapKalman(localMap, "elevation", "elevation_variance", actual_transform);
    }

    void updateGlobalMapKalman(grid_map::GridMap& localMap, const std::string& mean_layer_name, const std::string& variance_layer_name, geometry_msgs::TransformStamped transform){
        geometry_msgs::PointStamped max,min;
        max.point.x = -1000000;
        max.point.y = -1000000;
        min.point.x = 1000000;
        min.point.y = 1000000;

        std::list <geometry_msgs::PointStamped> globalPoints;
        std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> local_global_points_mapping;

        // Trasforma la mappa locale nella mappa globale
        for (grid_map::GridMapIterator it(localMap); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            const float& layer_value = localMap.at(mean_layer_name, index);

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
                local_global_points_mapping.push_back(std::make_tuple(globalPoint, localPoint));


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


        for(const auto& t : local_global_points_mapping){  
            
            geometry_msgs::PointStamped global_point = std::get<0>(t);  
            geometry_msgs::PointStamped local_point = std::get<1>(t);

            grid_map::Position globalPosition(global_point.point.x, global_point.point.y);
            grid_map::Position localPosition(local_point.point.x, local_point.point.y);

            try {
                    //globalMap_.atPosition(layer_name, globalPosition) = global_point.point.z;
                    float global_elevation = globalMap_.atPosition(mean_layer_name, globalPosition);
                    float global_variance = globalMap_.atPosition(variance_layer_name, globalPosition);


                    float local_elevation = localMap.atPosition(mean_layer_name, localPosition);
                    float local_variance = localMap.atPosition(variance_layer_name, localPosition);

                    ROS_WARN("global_elevation: %f", global_elevation);
                    ROS_WARN("local_elevation: %f", local_elevation);
                    ROS_WARN("global_variance: %f", global_variance);
                    ROS_WARN("local_variance: %f", local_variance);

                    // Calcolo del guadagno di Kalman
                    float kalman_gain = global_variance / (global_variance + local_variance);

                    // Aggiorna l'elevazione della cella globale
                    float updated_elevation = global_elevation + kalman_gain * (local_elevation - global_elevation);

                    // Aggiorna la varianza nella cella globale
                    float updated_variance = (1 - kalman_gain) * global_variance;

                    globalMap_.atPosition(mean_layer_name, globalPosition) = updated_elevation;
                    globalMap_.atPosition(variance_layer_name, globalPosition) = updated_variance;
                
                
                } catch (const std::out_of_range& e) {
                    ROS_ERROR("Error updating global map: %s", e.what());
                }
        }

        fillBordersWithFive(globalMap_);
        pub_tf(last_center_position_global_frame);
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

        fillBordersWithFive(globalMap_);
        pub_tf(last_center_position_global_frame);
        publish_gridmap();
    }
    
    void fillBordersWithFive(grid_map::GridMap& map) {
    // Ottieni i limiti della mappa
        grid_map::Size size = map.getSize();
        int rows = size(0);
        int cols = size(1);

        // Itera sui bordi e riempi con il valore 5
        for (int i = 0; i < rows; ++i) {
            for (const auto& layer : map.getLayers()) {
                // Bordo sinistro
                map.at(layer, grid_map::Index(i, 0)) = -2.0;
                // Bordo destro
                map.at(layer, grid_map::Index(i, cols - 1)) = -2.0;
            }
        }

        for (int j = 0; j < cols; ++j) {
            for (const auto& layer : map.getLayers()) {
                // Bordo superiore
                map.at(layer, grid_map::Index(0, j)) = -2.0;
                // Bordo inferiore
                map.at(layer, grid_map::Index(rows - 1, j)) = -2.0;
            }
        }
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        //ROS_INFO("Received an odometry message!");

        const geometry_msgs::Pose& pose = msg->pose.pose;

        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.child_frame_id = msg->header.frame_id;
        transformStamped.transform.translation.x = pose.position.x;
        transformStamped.transform.translation.y = pose.position.y;
        transformStamped.transform.translation.z = pose.position.z;
        transformStamped.transform.rotation = pose.orientation;

        pose_received = true;
        pub_odom_tf(msg);
    }

    void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        //geometry_msgs::TransformStamped actual_transform = transformStamped;
        geometry_msgs::TransformStamped actual_transform;
        actual_transform.transform.translation.x = 0.0;
        actual_transform.transform.translation.y = 0.0;
        actual_transform.transform.translation.z = 0.0;

        actual_transform.transform.rotation.x = 0.0;
        actual_transform.transform.rotation.y = 0.0;
        actual_transform.transform.rotation.z = 0.0;
        actual_transform.transform.rotation.w = 1.0;

        grid_map::GridMap localMap;
        grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "obstacles", localMap);

        updateGlobalMap(localMap, "obstacles", actual_transform);
    }
 
    void publish_gridmap()
    {
        globalMap_.setTimestamp(ros::Time::now().toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(globalMap_, message);
        message.info.header.frame_id = "grid_map";
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
                ROS_INFO("Layer '%s', Valore (dopo impostazione): %f", layer.c_str(), value);
            }
        }
    }

    void pub_tf(grid_map::Position newPosition){
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = ros::Time::now();
        tf.header.frame_id = "grid_map";
        tf.child_frame_id = "grid_map_center";
        tf.transform.translation.x = newPosition.x();
        tf.transform.translation.y = newPosition.y();
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
        tf.transform.rotation.w = 1.0;

        tf_broadcaster.sendTransform(tf);
    }

    //TODO: move in another node (se non xe tf)
    void pub_odom_tf(const nav_msgs::Odometry::ConstPtr& msg){
    // Trasformazione dinamica da "map" a "base_link"
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;  // Usa il timestamp dell'odometria
    tf.header.frame_id = "map";
    tf.child_frame_id = "base_link";

    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;

    tf.transform.rotation.x = msg->pose.pose.orientation.x;
    tf.transform.rotation.y = msg->pose.pose.orientation.y;
    tf.transform.rotation.z = msg->pose.pose.orientation.z;
    tf.transform.rotation.w = msg->pose.pose.orientation.w;

    tf_broadcaster.sendTransform(tf); 
}


private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber costmap_sub;
    ros::Publisher grid_map_pub;
    grid_map::GridMap globalMap_;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    grid_map::Position last_center_position_global_frame;
    double cell_size;
    double pc_height_threshold;
    double lidar_variance;
    std::string elevation_variance_layer_name;
    
    bool pose_received;
    geometry_msgs::TransformStamped transformStamped;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "p2g_node");
    Pointcloud_to_gridmap p2g_node;
    ros::spin();
    return 0;
}