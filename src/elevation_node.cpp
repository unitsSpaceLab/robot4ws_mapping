#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <geometry_msgs/TransformStamped.h>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <geometry_msgs/PointStamped.h>
#include <list>
#include <tuple>
#include <robot4ws_mapping/GridMapUpdateMsg.h>
#include "robot4ws_mapping/utilities.hpp"

class ElevationMap
{
public:
    ElevationMap()
    {   
        load_params();
        pose_received = false;

        if(elevation_logic=="mean") cloud_sub = nh_.subscribe(pc2_filtered_topic_name, 10, &ElevationMap::pointCloudCallback, this);
        else cloud_sub = nh_.subscribe(pc2_filtered_topic_name, 10, &ElevationMap::pointCloudCallbackPercentile, this);
        
        odom_sub = nh_.subscribe(odom_topic_name, 5, &ElevationMap::odom_callback, this);
        
        elevation_pub = nh_.advertise<robot4ws_mapping::GridMapUpdateMsg>(grid_map_update_topic_name, 1);

        ROS_INFO("Elevation node ready...");
    }

    void pointCloudCallbackPercentile(const sensor_msgs::PointCloud2::ConstPtr& cloud)
    {
        if (!pose_received) {
            ROS_WARN("No pose data received yet");
            return;
        }

        nav_msgs::Odometry actual_odom = actual_odom_msg;

        grid_map::GridMap localMap({elevation_layer_name, cloudPoint_counter_layer_name, elevation_variance_layer_name});
        localMap.setGeometry(grid_map::Length(local_map_size, local_map_size), cell_size);
        localMap[elevation_layer_name].setZero();
        localMap[cloudPoint_counter_layer_name].setZero();
        localMap[elevation_variance_layer_name].setConstant(initial_variance);

        std::unordered_map<grid_map::Index, std::vector<double>, robot4ws_mapping::IndexHash, robot4ws_mapping::IndexEqual> cell_values;

        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x"), iter_y(*cloud, "y"), iter_z(*cloud, "z");
                iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
                ROS_WARN("Skipping cloud point with NaN value: x = %f, y = %f, z = %f", *iter_x, *iter_y, *iter_z);
                continue;
            }

            if(*iter_z > pc_height_threshold) continue;

            grid_map::Position position(*iter_x, *iter_y);
            if (localMap.isInside(position)) {
                grid_map::Index index;
                localMap.getIndex(position, index);
                cell_values[index].push_back(*iter_z);
            }
        }

        for (const auto& cell : cell_values) {
            const grid_map::Index& index = cell.first;
            std::vector<double> values = cell.second;

            std::sort(values.begin(), values.end());
            size_t n = values.size();

            size_t percentile_index = static_cast<size_t>(percentile * (n - 1));
            double percentile_value = values[percentile_index];

            grid_map::Position position;
            localMap.getPosition(index, position);
            localMap.at(elevation_layer_name, index) = percentile_value;
            localMap.at(elevation_variance_layer_name, index) = lidar_variance;
            localMap.at(cloudPoint_counter_layer_name, index) = n;
        }

        grid_map_msgs::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(localMap, msg);
        pub_grid_map_update(msg, actual_odom);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){

        if (!pose_received) {
            ROS_WARN("No pose data received yet");
            return;
        }

        nav_msgs::Odometry actual_odom = actual_odom_msg;

        // Convert PointCloud2 to local GridMap
        grid_map::GridMap localMap({elevation_layer_name, cloudPoint_counter_layer_name, elevation_variance_layer_name});
        localMap.setGeometry(grid_map::Length(local_map_size, local_map_size), cell_size);
        localMap[elevation_layer_name].setZero();
        localMap[cloudPoint_counter_layer_name].setZero();
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
                localMap.atPosition(cloudPoint_counter_layer_name, position)+=1;
                int cell_count = localMap.atPosition(cloudPoint_counter_layer_name, position);
                double old_mean = localMap.atPosition(elevation_layer_name, position);
                double old_variance = localMap.atPosition(elevation_variance_layer_name, position);

                double new_mean = (old_mean * (cell_count-1) + *iter_z)/cell_count;

                localMap.atPosition(elevation_layer_name, position) = new_mean;  
                localMap.atPosition(elevation_variance_layer_name, position) = lidar_variance;
                }
        }

        grid_map_msgs::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(localMap, msg);
        pub_grid_map_update(msg, actual_odom);   
    }
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        actual_odom_msg = *msg;
        pose_received = true;
    }

    void pub_grid_map_update(grid_map_msgs::GridMap msg, nav_msgs::Odometry odom_msg){
        robot4ws_mapping::GridMapUpdateMsg update_elevation_msg;

        update_elevation_msg.header.stamp = ros::Time::now();
        update_elevation_msg.header.frame_id = elevation_layer_name;

        update_elevation_msg.local_gridmap = msg;
        
        update_elevation_msg.odom = odom_msg;

        elevation_pub.publish(update_elevation_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub;
    ros::Subscriber odom_sub;
    ros::Publisher elevation_pub;

    std::string elevation_layer_name, elevation_variance_layer_name, odom_topic_name, grid_map_update_topic_name, pc2_filtered_topic_name, cloudPoint_counter_layer_name;
    std::string elevation_logic;
    double local_map_size, cell_size, lidar_variance, pc_height_threshold, percentile, initial_variance;

    nav_msgs::Odometry actual_odom_msg;
    bool pose_received;

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
        if (! nh_.getParam("gridmap/initial_variance",initial_variance))
        {
            initial_variance = 1e6;
            ROS_WARN_STREAM("Parameter [gridmap/initial_variance] not found. Using default value: " << initial_variance);
        }
        
        //Robot related params
        if (! nh_.getParam("gridmap/lidar_variance",lidar_variance))
        {
            lidar_variance = 0.2;
            ROS_WARN_STREAM("Parameter [gridmap/lidar_variance] not found. Using default value: " << lidar_variance);
        }

        //Topics
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
        if (!nh_.getParam("gridmap/pc2_filtered_topic_name",pc2_filtered_topic_name))
        {
            pc2_filtered_topic_name = "/velodyne_filtered";
            ROS_WARN_STREAM("Parameter [gridmap/pc2_filtered_topic_name] not found. Using default value: " << pc2_filtered_topic_name);
        }

        //Elevation params
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
        if (! nh_.getParam("gridmap/point_cloud_max_height_threshold",pc_height_threshold))
        {
            pc_height_threshold = 3.0;
            ROS_WARN_STREAM("Parameter [gridmap/pc_height_threshold] not found. Using default value: " << pc_height_threshold);
        }

        //Layer names
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
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "elevation_node");

    ElevationMap ElevationMap;

    ros::spin();
    return 0;
}
