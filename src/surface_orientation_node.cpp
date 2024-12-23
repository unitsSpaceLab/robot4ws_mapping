#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <robot4ws_mapping/GridMapUpdateMsg.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <visualization_msgs/MarkerArray.h>

class SurfaceOrientationMap{

public:
    SurfaceOrientationMap(){   
        load_params();

        global_map_update_sub = nh_.subscribe(grid_map_global_update_topic_name, 5, &SurfaceOrientationMap::update_surface_orientation_callback, this);

        local_grid_map_update_pub = nh_.advertise<robot4ws_mapping::GridMapUpdateMsg>(grid_map_update_topic_name, 1);
        surface_normal_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>(surface_orientation_layer_name, 1);

        ROS_INFO("Surface orientation node ready...");
    }

    void pub_local_grid_map_update(grid_map::GridMap localMap, nav_msgs::Odometry odom_msg){
        grid_map_msgs::GridMap map_msg;
        grid_map::GridMapRosConverter::toMessage(localMap, map_msg);

        robot4ws_mapping::GridMapUpdateMsg update_surface_msg;

        update_surface_msg.header.stamp = ros::Time::now();
        update_surface_msg.header.frame_id = surface_orientation_layer_name;

        update_surface_msg.local_gridmap = map_msg;
        
        update_surface_msg.odom = odom_msg;

        local_grid_map_update_pub.publish(update_surface_msg);
    }

    void update_surface_orientation_callback(const robot4ws_mapping::GridMapUpdateMsg::ConstPtr& msg){
        grid_map::GridMap globalMap_;
        grid_map::GridMapRosConverter::fromMessage(msg->local_gridmap, globalMap_);

        grid_map::GridMap localMap = initializeLocalMap(globalMap_);

        if (!validateGlobalMap(globalMap_) || !validateMapDimensions(globalMap_, localMap)) {
            return;
        }

        processOrientation(globalMap_, localMap);
        pub_local_grid_map_update(localMap, msg->odom);

        visualization_msgs::MarkerArray markerArray = createNormalVectorMarkers(localMap, globalMap_);
        surface_normal_marker_pub.publish(markerArray);
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber global_map_update_sub;
    ros::Publisher local_grid_map_update_pub;
    ros::Publisher surface_normal_marker_pub;

    //Gridmap params
    double cell_size;
    std::string grid_map_frame_id;

    //Topics
    std::string grid_map_global_update_topic_name, grid_map_update_topic_name;
    std::string surface_orientation_layer_name, surface_orientation_variance_layer_name, elevation_layer_name, elevation_variance_layer_name;
    std::string surface_orientation_x_layer_name, surface_orientation_y_layer_name, surface_orientation_z_layer_name;

    void load_params(){
        //Gridmap params
        if (! nh_.getParam("gridmap/grid_map_frame_id",grid_map_frame_id))
        {
            grid_map_frame_id = "odom";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_frame_id] not found. Using default value: " << grid_map_frame_id);
        }
        if (! nh_.getParam("gridmap/cell_size",cell_size))
        {
            cell_size = 0.2;
            ROS_WARN_STREAM("Parameter [gridmap/cell_size] not found. Using default value: " << cell_size);
        }
        
        //Topics
        if (! nh_.getParam("gridmap/grid_map_global_update_topic_name",grid_map_global_update_topic_name))
        {
            grid_map_global_update_topic_name = "/odom";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_global_update_topic_name] not found. Using default value: " << grid_map_global_update_topic_name);
        }
        if (! nh_.getParam("gridmap/grid_map_update_topic_name",grid_map_update_topic_name))
        {
            grid_map_update_topic_name = "grid_map_update";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_update_topic_name] not found. Using default value: " << grid_map_update_topic_name);
        }

        //Layer names
        if (!nh_.getParam("gridmap/surface_orientation_layer_name",surface_orientation_layer_name))
        {
            surface_orientation_layer_name = "surface_orientation";
            ROS_WARN_STREAM("Parameter [gridmap/surface_orientation_layer_name] not found. Using default value: " << surface_orientation_layer_name);
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
            surface_orientation_z_layer_name = "surface_orientation_z";
            ROS_WARN_STREAM("Parameter [gridmap/surface_orientation_z_layer_name] not found. Using default value: " << surface_orientation_z_layer_name);
        }
        if (!nh_.getParam("gridmap/surface_orientation_variance_layer_name",surface_orientation_variance_layer_name))
        {
            surface_orientation_variance_layer_name = "surface_orientation_variance";
            ROS_WARN_STREAM("Parameter [gridmap/surface_orientation_variance_layer_name] not found. Using default value: " << surface_orientation_variance_layer_name);
        }
        if (!nh_.getParam("gridmap/elevation_layer_name",elevation_layer_name))
        {
            elevation_layer_name = "elevation";
            ROS_WARN_STREAM("Parameter [gridmap/elevation_layer_name] not found. Using default value: " << elevation_layer_name);
        }
        if (!nh_.getParam("gridmap/elevation_layer_name",elevation_variance_layer_name))
        {
            elevation_variance_layer_name = "elevation_variance";
            ROS_WARN_STREAM("Parameter [gridmap/elevation_variance_layer_name] not found. Using default value: " << elevation_variance_layer_name);
        }
    }

    visualization_msgs::MarkerArray createNormalVectorMarkers(grid_map::GridMap localMap, grid_map::GridMap globalMap_){
            //TODO optimize, need just elevation not all globalMap_
            visualization_msgs::MarkerArray markerArray;
            int marker_id = 0;

            for (grid_map::GridMapIterator it(localMap); !it.isPastEnd(); ++it) {
                grid_map::Index index(*it);
                grid_map::Position position;
                localMap.getPosition(index, position);

                if (!localMap.isValid(index, surface_orientation_x_layer_name) || !localMap.isValid(index, surface_orientation_y_layer_name) || !localMap.isValid(index,surface_orientation_z_layer_name))
                    continue;

                double x = localMap.at(surface_orientation_x_layer_name, index);
                double y = localMap.at(surface_orientation_y_layer_name, index);
                double z = localMap.at(surface_orientation_z_layer_name, index);

                Eigen::Vector3d normal(x, y, z);
                normal.normalize();

                double elevation_value = globalMap_.at(elevation_layer_name, index);

                visualization_msgs::Marker marker;
                marker.header.frame_id = grid_map_frame_id;
                marker.header.stamp = ros::Time::now();
                marker.ns = surface_orientation_layer_name;
                marker.id = marker_id++;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;

                geometry_msgs::Point start;
                start.x = position.x();
                start.y = position.y();
                start.z = elevation_value;
                marker.points.push_back(start);

                geometry_msgs::Point end;
                end.x = position.x() + normal.x();
                end.y = position.y() + normal.y();
                end.z = elevation_value + normal.z();
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
    
    bool validateGlobalMap(const grid_map::GridMap& globalMap){
        if (!globalMap.exists(elevation_layer_name)) {
            ROS_WARN("Layer '%s' does not exist in the global map.", elevation_layer_name.c_str());
            return false;
        }
        return true;
    }

    grid_map::GridMap initializeLocalMap(const grid_map::GridMap& globalMap){
        grid_map::GridMap localMap;
        localMap.setGeometry(globalMap.getLength(), globalMap.getResolution());
        localMap.setFrameId(globalMap.getFrameId());
        localMap.setPosition(globalMap.getPosition());
        localMap.add(surface_orientation_x_layer_name, 0);
        localMap.add(surface_orientation_y_layer_name, 0);
        localMap.add(surface_orientation_z_layer_name, 0);
        localMap.add(surface_orientation_variance_layer_name, 1e6);
        return localMap;
    }

    bool validateMapDimensions(const grid_map::GridMap& globalMap, const grid_map::GridMap& localMap){
        if (globalMap.getLength().x() != localMap.getLength().x() || 
            globalMap.getLength().y() != localMap.getLength().y() || 
            globalMap.getResolution() != localMap.getResolution()) {
            ROS_ERROR("Error updating surface orientation: different map sizes or resolution");
            return false;
        }
        return true;
    }

    void processOrientation(const grid_map::GridMap& globalMap_, grid_map::GridMap& localMap){
        for (grid_map::GridMapIterator iterator(globalMap_); !iterator.isPastEnd(); ++iterator) {
            grid_map::Index index(*iterator);
            grid_map::Position position;
            globalMap_.getPosition(index, position);

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

            double variance_normal = variance_dz_dx + variance_dz_dy;

            try {
                localMap.atPosition(surface_orientation_x_layer_name, position) = -dz_dx;
                localMap.atPosition(surface_orientation_y_layer_name, position) = -dz_dy;
                localMap.atPosition(surface_orientation_z_layer_name, position) = 1.0;
                localMap.atPosition(surface_orientation_variance_layer_name, position) = variance_normal;
            } catch (const std::exception& e) {
                ROS_ERROR("Error updating surface orientation layers at position [%f, %f]: %s",
                        position.x(), position.y(), e.what());
                return;
            }
        } 
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "surface_orientation_node");

    SurfaceOrientationMap SurfaceOrientationMap;

    ros::spin();
    return 0;
}
