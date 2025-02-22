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
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>
#include <functional>

class multi_layer_map{
public:
    multi_layer_map() {
        load_params();
        load_robot_static_tf();

        elevation_layer_name = "elevation";
        elevation_variance_layer_name = "elevation_variance";
        obstacles_layer_name = "obstacles";
        surface_orientation_x_layer_name = "surface_orientation_x";
        surface_orientation_y_layer_name = "surface_orientation_y";
        surface_orientation_z_layer_name = "surface_orientation_z";
        surface_orientation_variance_layer_name = "surface_orientation_variance";

        // Global map init
        globalMap_.setFrameId(grid_map_frame_id);
        globalMap_.setGeometry(grid_map::Length(global_map_size, global_map_size), cell_size);
        globalMap_.add(elevation_layer_name, 0.0);
        globalMap_.add(elevation_variance_layer_name, 1e6);
        globalMap_.add(obstacles_layer_name, 0.0);
        globalMap_.add(surface_orientation_x_layer_name, 0);
        globalMap_.add(surface_orientation_y_layer_name, 0);
        globalMap_.add(surface_orientation_z_layer_name, 0);
        globalMap_.add(surface_orientation_variance_layer_name, 1e6);

        if(elevation_logic=="mean") cloud_sub = nh_.subscribe(lidar_topic_name, 10, &multi_layer_map::pointCloudCallback, this);
        else cloud_sub = nh_.subscribe(lidar_topic_name, 10, &multi_layer_map::pointCloudCallbackPercentile, this);

        odom_sub = nh_.subscribe(odom_topic_name, 5, &multi_layer_map::odom_callback, this);

        grid_map_pub = nh_.advertise<grid_map_msgs::GridMap>(grid_map_topic_name, 1, true);
        surface_normal_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("surface_normal", 1);

        pose_received = false;
    }

    struct IndexHash {
        // Hash for grid_map::Index
        std::size_t operator()(const grid_map::Index& index) const {
            return std::hash<int>()(index.x()) ^ (std::hash<int>()(index.y()) << 1);
        }
    };

    struct IndexEqual {
        // Equality for grid_map::Index
        bool operator()(const grid_map::Index& a, const grid_map::Index& b) const {
            return a.x() == b.x() && a.y() == b.y();
        }
    };

    void pointCloudCallbackPercentile(const sensor_msgs::PointCloud2::ConstPtr& cloud){
        if (!pose_received) {
            ROS_WARN("No pose data received yet");
            return;
        }

        geometry_msgs::TransformStamped actual_transform = gridmap_to_lidar_transform_msg;

        // Convert PointCloud2 to local GridMap
        grid_map::GridMap localMap({elevation_layer_name, "cloudPoint_counter", elevation_variance_layer_name});
        localMap.setGeometry(grid_map::Length(local_map_size, local_map_size), cell_size);
        localMap[elevation_layer_name].setZero();
        localMap["cloudPoint_counter"].setZero();
        localMap[elevation_variance_layer_name].setConstant(1e6);

        // Unordered map to store values for percentile computation
        std::unordered_map<grid_map::Index, std::vector<double>, IndexHash, IndexEqual> cell_values;

        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x"), iter_y(*cloud, "y"), iter_z(*cloud, "z");
                iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
                ROS_WARN("Skipping cloud point with NaN value: x = %f, y = %f, z = %f", *iter_x, *iter_y, *iter_z);
                continue;
            }

            // Skip cloud points above the height threshold
            if(*iter_z > pc_height_threshold) continue;

            grid_map::Position position(*iter_x, *iter_y);
            if (localMap.isInside(position)) {
                grid_map::Index index;
                localMap.getIndex(position, index);
                cell_values[index].push_back(*iter_z);
            }
        }

        // Process each cell to compute the percentile
        for (const auto& cell : cell_values) {
            const grid_map::Index& index = cell.first;
            std::vector<double> values = cell.second;

            // Sort values to compute percentile
            std::sort(values.begin(), values.end());
            size_t n = values.size();

            // Compute the i-th percentile index
            size_t percentile_index = static_cast<size_t>(percentile * (n - 1));
            double percentile_value = values[percentile_index];

            // Update GridMap with the percentile value
            grid_map::Position position;
            localMap.getPosition(index, position);
            localMap.at(elevation_layer_name, index) = percentile_value;
            localMap.at(elevation_variance_layer_name, index) = lidar_variance; // Set variance to a constant
            localMap.at("cloudPoint_counter", index) = n; // Store number of points in the cell
        }

        std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> transformed_point_tuple_list = applyTransform(localMap, elevation_layer_name, elevation_variance_layer_name, actual_transform);
        updateGlobalMapKalman(localMap, elevation_layer_name, elevation_variance_layer_name, actual_transform, transformed_point_tuple_list);

        // Update surface normals
        update_surface_orientation(transformed_point_tuple_list);
    } 

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){

        if (!pose_received) {
            ROS_WARN("No pose data received yet");
            return;
        }

        geometry_msgs::TransformStamped actual_transform = gridmap_to_lidar_transform_msg;

        // Convert PointCloud2 to local GridMap
        grid_map::GridMap localMap({elevation_layer_name, "cloudPoint_counter", elevation_variance_layer_name});
        localMap.setGeometry(grid_map::Length(local_map_size, local_map_size), cell_size);
        localMap[elevation_layer_name].setZero();
        localMap["cloudPoint_counter"].setZero();
        localMap[elevation_variance_layer_name].setConstant(1e6);

        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x"), iter_y(*cloud, "y"), iter_z(*cloud, "z");
                iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
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
    
        std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> transformed_point_tuple_list = applyTransform(localMap, elevation_layer_name, elevation_variance_layer_name, actual_transform);
        updateGlobalMapKalman(localMap, elevation_layer_name, elevation_variance_layer_name, actual_transform, transformed_point_tuple_list);
        
        //update surface normals
        update_surface_orientation(transformed_point_tuple_list);   
    }

    void update_surface_orientation(std::list<std::tuple<geometry_msgs::PointStamped, geometry_msgs::PointStamped>> local_global_points_mapping) {
        // Method to update surface orientation using finite difference theory and error propagation for variance
        for (const auto& t : local_global_points_mapping) {
            geometry_msgs::PointStamped global_point = std::get<0>(t);
            grid_map::Position position(global_point.point.x, global_point.point.y);

            // Elevation gradient along x and y
            double dz_dx = 0.0;
            double dz_dy = 0.0;

            // Gradient variance
            double variance_dz_dx = 0.0;
            double variance_dz_dy = 0.0;

            // Finite differences for calculating dz/dx and error propagation for variance
            if (!globalMap_.isInside(grid_map::Position(position.x() + cell_size, position.y()))) {
                // Right border (x-grid)
                grid_map::Index prevIdx1, prevIdx2;
                globalMap_.getIndex(grid_map::Position(position.x() - cell_size, position.y()), prevIdx1);
                globalMap_.getIndex(grid_map::Position(position.x() - 2 * cell_size, position.y()), prevIdx2);
                if (globalMap_.isValid(prevIdx1, elevation_layer_name) && globalMap_.isValid(prevIdx2, elevation_layer_name)) {
                    dz_dx = (3 * globalMap_.atPosition(elevation_layer_name, position)
                            - 4 * globalMap_.at(elevation_layer_name, prevIdx1)
                            + globalMap_.at(elevation_layer_name, prevIdx2)) / (2 * cell_size);
                    variance_dz_dx = (9 * globalMap_.atPosition(elevation_variance_layer_name, position) +
                                    16 * globalMap_.at(elevation_variance_layer_name, prevIdx1) +
                                    globalMap_.at(elevation_variance_layer_name, prevIdx2)) /
                                    (4 * cell_size * cell_size);
                }
            } else if (!globalMap_.isInside(grid_map::Position(position.x() - cell_size, position.y()))) {
                // Left border (x-grid)
                grid_map::Index nextIdx1, nextIdx2;
                globalMap_.getIndex(grid_map::Position(position.x() + cell_size, position.y()), nextIdx1);
                globalMap_.getIndex(grid_map::Position(position.x() + 2 * cell_size, position.y()), nextIdx2);
                if (globalMap_.isValid(nextIdx1, elevation_layer_name) && globalMap_.isValid(nextIdx2, elevation_layer_name)) {
                    dz_dx = (-3 * globalMap_.atPosition(elevation_layer_name, position)
                            + 4 * globalMap_.at(elevation_layer_name, nextIdx1)
                            - globalMap_.at(elevation_layer_name, nextIdx2)) / (2 * cell_size);
                    variance_dz_dx = (9 * globalMap_.atPosition(elevation_variance_layer_name, position) +
                                    16 * globalMap_.at(elevation_variance_layer_name, nextIdx1) +
                                    globalMap_.at(elevation_variance_layer_name, nextIdx2)) /
                                    (4 * cell_size * cell_size);
                }
            } else {
                // Central condition
                grid_map::Index previousIndex, nextIndex;
                globalMap_.getIndex(grid_map::Position(position.x() + cell_size, position.y()), nextIndex);
                globalMap_.getIndex(grid_map::Position(position.x() - cell_size, position.y()), previousIndex);
                if (globalMap_.isValid(nextIndex, elevation_layer_name) && globalMap_.isValid(previousIndex, elevation_layer_name)) {
                    dz_dx = (globalMap_.at(elevation_layer_name, nextIndex) - globalMap_.at(elevation_layer_name, previousIndex)) / (2 * cell_size);
                    variance_dz_dx = (globalMap_.at(elevation_variance_layer_name, nextIndex) +
                                    globalMap_.at(elevation_variance_layer_name, previousIndex)) /
                                    (4 * cell_size * cell_size);
                }
            }

            // Finite differences for calculating dz/dy and error propagation for variance
            if (!globalMap_.isInside(grid_map::Position(position.x(), position.y() + cell_size))) {
                // Upper border (y-grid)
                grid_map::Index prevIdx1, prevIdx2;
                globalMap_.getIndex(grid_map::Position(position.x(), position.y() - cell_size), prevIdx1);
                globalMap_.getIndex(grid_map::Position(position.x(), position.y() - 2 * cell_size), prevIdx2);
                if (globalMap_.isValid(prevIdx1, elevation_layer_name) && globalMap_.isValid(prevIdx2, elevation_layer_name)) {
                    dz_dy = (3 * globalMap_.atPosition(elevation_layer_name, position)
                            - 4 * globalMap_.at(elevation_layer_name, prevIdx1)
                            + globalMap_.at(elevation_layer_name, prevIdx2)) / (2 * cell_size);
                    variance_dz_dy = (9 * globalMap_.atPosition(elevation_variance_layer_name, position) +
                                    16 * globalMap_.at(elevation_variance_layer_name, prevIdx1) +
                                    globalMap_.at(elevation_variance_layer_name, prevIdx2)) /
                                    (4 * cell_size * cell_size);
                }
            } else if (!globalMap_.isInside(grid_map::Position(position.x(), position.y() - cell_size))) {
                // Lower border (y-grid)
                grid_map::Index nextIdx1, nextIdx2;
                globalMap_.getIndex(grid_map::Position(position.x(), position.y() + cell_size), nextIdx1);
                globalMap_.getIndex(grid_map::Position(position.x(), position.y() + 2 * cell_size), nextIdx2);
                if (globalMap_.isValid(nextIdx1, elevation_layer_name) && globalMap_.isValid(nextIdx2, elevation_layer_name)) {
                    dz_dy = (-3 * globalMap_.atPosition(elevation_layer_name, position)
                            + 4 * globalMap_.at(elevation_layer_name, nextIdx1)
                            - globalMap_.at(elevation_layer_name, nextIdx2)) / (2 * cell_size);
                    variance_dz_dy = (9 * globalMap_.atPosition(elevation_variance_layer_name, position) +
                                    16 * globalMap_.at(elevation_variance_layer_name, nextIdx1) +
                                    globalMap_.at(elevation_variance_layer_name, nextIdx2)) /
                                    (4 * cell_size * cell_size);
                }
            } else {
                // Central condition
                grid_map::Index previousIndex, nextIdx;
                globalMap_.getIndex(grid_map::Position(position.x(), position.y() + cell_size), nextIdx);
                globalMap_.getIndex(grid_map::Position(position.x(), position.y() - cell_size), previousIndex);
                if (globalMap_.isValid(nextIdx, elevation_layer_name) && globalMap_.isValid(previousIndex, elevation_layer_name)) {
                    dz_dy = (globalMap_.at(elevation_layer_name, nextIdx) - globalMap_.at(elevation_layer_name, previousIndex)) / (2 * cell_size);
                    variance_dz_dy = (globalMap_.at(elevation_variance_layer_name, nextIdx) +
                                    globalMap_.at(elevation_variance_layer_name, previousIndex)) /
                                    (4 * cell_size * cell_size);
                }
            }

            // Normalized normal vector and combined variance
            Eigen::Vector3d normal(-dz_dx, -dz_dy, 1.0);
            double variance_normal = variance_dz_dx + variance_dz_dy;

            normal.normalize();
            globalMap_.atPosition(surface_orientation_x_layer_name, position) = normal.x();
            globalMap_.atPosition(surface_orientation_y_layer_name, position) = normal.y();
            globalMap_.atPosition(surface_orientation_z_layer_name, position) = normal.z();
            globalMap_.atPosition(surface_orientation_variance_layer_name, position) = variance_normal;

            //update obstacle detection based on maximum traversable slope
            double slope = std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy);
            if (slope > max_slope_threshold) {
                globalMap_.atPosition(obstacles_layer_name, position) = 100.0; // Mark as obstacle
            } else {
                globalMap_.atPosition(obstacles_layer_name, position) = 0.0; // Mark as free
            }

        }
        visualization_msgs::MarkerArray markerArray = createNormalVectorMarkers();
        surface_normal_marker_pub.publish(markerArray);
    }

    visualization_msgs::MarkerArray createNormalVectorMarkers() {
        visualization_msgs::MarkerArray markerArray;
        int marker_id = 0;

        for (grid_map::GridMapIterator it(globalMap_); !it.isPastEnd(); ++it) {
            grid_map::Index index(*it);
            grid_map::Position position;
            globalMap_.getPosition(index, position);

            if (!globalMap_.isValid(index, surface_orientation_x_layer_name) || !globalMap_.isValid(index, surface_orientation_y_layer_name) || !globalMap_.isValid(index,surface_orientation_z_layer_name))
                continue;

            double x = globalMap_.at(surface_orientation_x_layer_name, index);
            double y = globalMap_.at(surface_orientation_y_layer_name, index);
            double z = globalMap_.at(surface_orientation_z_layer_name, index);

            double elevation_value = globalMap_.atPosition(elevation_layer_name, position);

            visualization_msgs::Marker marker;
            marker.header.frame_id = grid_map_frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = "gridmap_surface_normals";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;

            geometry_msgs::Point start;
            start.x = position.x();
            start.y = position.y();
            start.z = elevation_value;
            marker.points.push_back(start);

            geometry_msgs::Point end;
            end.x = position.x() + x;
            end.y = position.y() + y;
            end.z = elevation_value + z;
            marker.points.push_back(end);

            marker.scale.x = 0.03;
            marker.scale.y = 0.05;
            marker.scale.z = 0.02;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            markerArray.markers.push_back(marker);
        }
        return markerArray;
    }

    std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> applyTransform(grid_map::GridMap& localMap, const std::string& mean_layer_name, const std::string& variance_layer_name, geometry_msgs::TransformStamped transform){
        geometry_msgs::PointStamped max,min;
        max.point.x = -1000000;
        max.point.y = -1000000;
        min.point.x = 1000000;
        min.point.y = 1000000;

        std::list<std::tuple<geometry_msgs::PointStamped,geometry_msgs::PointStamped>> local_global_points_mapping;

        // Transform localmap to globalmap 
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
        
        gridmap_to_lidar_transform_msg.header.stamp = msg->header.stamp;
        gridmap_to_lidar_transform_msg.child_frame_id = msg->header.frame_id;
        gridmap_to_lidar_transform_msg.transform = tf2::toMsg(transform);

        pose_received = true;
    }
    
    void publish_gridmap()
    {
        globalMap_.setTimestamp(ros::Time::now().toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(globalMap_, message);
        message.info.header.frame_id = grid_map_frame_id;
        grid_map_pub.publish(message);
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
            if (layer.c_str() == elevation_variance_layer_name || layer.c_str() == surface_orientation_variance_layer_name) newMap[layer].setConstant(1e6);
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
        /* if (! nh_.getParam("gridmap/lidar_topic_name",lidar_topic_name))
        {
            lidar_topic_name = "/velodyne";
            ROS_WARN_STREAM("Parameter [gridmap/lidar_topic_name] not found. Using default value: " << lidar_topic_name);
        } */
        if (! nh_.getParam("gridmap/pc2_filtered_topic_name",lidar_topic_name))
        {
            lidar_topic_name = "/velodyne_filtered";
            ROS_WARN_STREAM("Parameter [gridmap/pc2_filtered_topic_name] not found. Using default value: " << lidar_topic_name);
        }
        if (! nh_.getParam("gridmap/odom_topic_name",odom_topic_name))
        {
            odom_topic_name = "/odom";
            ROS_WARN_STREAM("Parameter [gridmap/odom_topic_name] not found. Using default value: " << odom_topic_name);
        }
        if (! nh_.getParam("gridmap/grid_map_frame_id",grid_map_frame_id))
        {
            grid_map_frame_id = "odom";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_frame_id] not found. Using default value: " << grid_map_frame_id);
        }
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
        if (! nh_.getParam("gridmap/max_slope_threshold",max_slope_threshold))
        {
            max_slope_threshold = 0.523599; //30 deg in radiant
            ROS_WARN_STREAM("Parameter [gridmap/max_slope_threshold] not found. Using default value: " << max_slope_threshold);
        }
        if (! nh_.getParam("gridmap/elevation_logic",elevation_logic))
        {
            elevation_logic = "mean";
            ROS_WARN_STREAM("Parameter [gridmap/elevation_logic] not found. Using default value: " << elevation_logic);
        }
        if(elevation_logic == "percentile"){
            if (! nh_.getParam("gridmap/percentile",percentile))
            {
                percentile = 0.8;
                ROS_WARN_STREAM("Parameter [gridmap/percentile] not found. Using default value: " << percentile);
            }
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
    ros::Publisher grid_map_pub;
    ros::Publisher surface_normal_marker_pub;
    
    std::string grid_map_topic_name;
    std::string lidar_topic_name;
    std::string odom_topic_name;
    
    //GridMap attributes:
    grid_map::GridMap globalMap_;
    grid_map::Position last_center_position_global_frame;
    std::string grid_map_frame_id;
    
    double global_map_size;
    double local_map_size;
    double cell_size;
    double percentile;

    std::string elevation_logic;
    
    std::string surface_orientation_x_layer_name;
    std::string surface_orientation_y_layer_name;
    std::string surface_orientation_z_layer_name;
    std::string surface_orientation_variance_layer_name;
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
    geometry_msgs::TransformStamped gridmap_to_lidar_transform_msg;
    double max_slope_threshold;
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "p2g_node");
    multi_layer_map p2g_node;
    ros::spin();
    return 0;
}