#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <robot4ws_mapping/GridMapUpdateMsg.h>

class ObstaclesMap{

public:
    ObstaclesMap(){   
        load_params();

        orientation_map_update_sub = nh_.subscribe(grid_map_update_topic_name, 5, &ObstaclesMap::obstacles_callback, this);

        obstacles_map_update_pub = nh_.advertise<robot4ws_mapping::GridMapUpdateMsg>(grid_map_update_topic_name, 1);
        
        ROS_INFO("Obstacles node ready...");
    }

    void pub_local_grid_map_update(grid_map::GridMap localMap, nav_msgs::Odometry odom_msg){
        grid_map_msgs::GridMap map_msg;
        grid_map::GridMapRosConverter::toMessage(localMap, map_msg);

        robot4ws_mapping::GridMapUpdateMsg update_obstacles_msg;

        update_obstacles_msg.header.stamp = ros::Time::now();
        update_obstacles_msg.header.frame_id = obstacles_layer_name;

        update_obstacles_msg.local_gridmap = map_msg;
        
        update_obstacles_msg.odom = odom_msg;

        obstacles_map_update_pub.publish(update_obstacles_msg);
    }

    void obstacles_callback(const robot4ws_mapping::GridMapUpdateMsg::ConstPtr& msg){
        
        if (msg->header.frame_id == surface_orientation_layer_name){
            grid_map::GridMap orientationMap;
            grid_map::GridMapRosConverter::fromMessage(msg->local_gridmap, orientationMap);
            grid_map::GridMap obstaclesMap = initializeLocalMap(orientationMap);
            
            processObstacles(obstaclesMap, orientationMap);
            pub_local_grid_map_update(obstaclesMap, msg->odom);
        }
    }      
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber orientation_map_update_sub;
    ros::Publisher obstacles_map_update_pub;

    double max_slope_threshold;

    //Topics
    std::string grid_map_global_update_topic_name, grid_map_update_topic_name;
    std::string surface_orientation_layer_name, surface_orientation_x_layer_name, surface_orientation_y_layer_name, surface_orientation_z_layer_name;
    std::string obstacles_layer_name, obstacles_variance_layer_name, surface_orientation_variance_layer_name; 

    void load_params(){
        //Obstacle param
        if (! nh_.getParam("gridmap/max_slope_threshold",max_slope_threshold))
        {
            max_slope_threshold = 0.523599; //30 deg in radiant
            ROS_WARN_STREAM("Parameter [gridmap/max_slope_threshold] not found. Using default value: " << max_slope_threshold);
        }
        
        //Topics
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
            surface_orientation_z_layer_name = "surface_orientation_z";
            ROS_WARN_STREAM("Parameter [gridmap/surface_orientation_z_layer_name] not found. Using default value: " << surface_orientation_z_layer_name);
        }
        if (!nh_.getParam("gridmap/surface_orientation_variance_layer_name",surface_orientation_variance_layer_name))
        {
            surface_orientation_variance_layer_name = "surface_orientation_variance";
            ROS_WARN_STREAM("Parameter [gridmap/surface_orientation_variance_layer_name] not found. Using default value: " << surface_orientation_variance_layer_name);
        }
    }
    
    grid_map::GridMap initializeLocalMap(const grid_map::GridMap& orientationMap) {
        grid_map::GridMap localMap;
        localMap.setGeometry(orientationMap.getLength(), orientationMap.getResolution());
        localMap.setFrameId(orientationMap.getFrameId());
        localMap.setPosition(orientationMap.getPosition());
        localMap.add(obstacles_layer_name, 0);
        localMap.add(obstacles_variance_layer_name, 1e6);
        return localMap;
    }

    void processObstacles(grid_map::GridMap& obstaclesMap, const grid_map::GridMap& orientationMap){
        double dz_dx;
        double dz_dy;

        for (grid_map::GridMapIterator iterator(orientationMap); !iterator.isPastEnd(); ++iterator) {
            grid_map::Index index(*iterator);
            grid_map::Position position;
            orientationMap.getPosition(index, position);

            dz_dx = orientationMap.atPosition(surface_orientation_x_layer_name, position);
            dz_dy = orientationMap.atPosition(surface_orientation_y_layer_name, position);

            double slope = std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy);
            if (slope > max_slope_threshold) {
                obstaclesMap.atPosition(obstacles_layer_name, position) = 100.0; // Mark as obstacle
                obstaclesMap.atPosition(obstacles_variance_layer_name, position) = orientationMap.atPosition(surface_orientation_variance_layer_name, position);
            } else {
                obstaclesMap.atPosition(obstacles_layer_name, position) = 0.0; // Mark as free
                obstaclesMap.atPosition(obstacles_variance_layer_name, position) = orientationMap.atPosition(surface_orientation_variance_layer_name, position);
            }
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacles_node");

    ObstaclesMap ObstaclesMap;

    ros::spin();
    return 0;
}
