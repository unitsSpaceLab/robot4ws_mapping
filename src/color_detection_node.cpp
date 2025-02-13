#include <ros/ros.h>
#include "robot4ws_mapping/utilities.hpp"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <robot4ws_mapping/GridMapUpdateMsg.h>

#include <robot4ws_msgs/ColorDetection3DArray.h>
#include <robot4ws_msgs/ColorDetection3D.h>

#include <grid_map_cv/grid_map_cv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>

static const std::unordered_map<std::string, robot4ws_mapping::ColorId> colorMap = {
        {"unknown", robot4ws_mapping::ColorId::Unknown},
        {"red", robot4ws_mapping::ColorId::Red},
        {"green", robot4ws_mapping::ColorId::Green},
        {"blue", robot4ws_mapping::ColorId::Blue},
        {"yellow", robot4ws_mapping::ColorId::Yellow},
        {"orange", robot4ws_mapping::ColorId::Orange},
        {"cyan", robot4ws_mapping::ColorId::Cyan},
        {"purple", robot4ws_mapping::ColorId::Purple},
    };

class ColorDetector{

public:
    ColorDetector(){   
        load_params();
        initLocalMap();

        color_sub = nh_.subscribe(color_detection_topic_name, 1, &ColorDetector::color_detection3D_callback, this); 

        color_map_update_pub = nh_.advertise<robot4ws_mapping::GridMapUpdateMsg>(grid_map_update_topic_name, 1);
        grid_map_pub = nh_.advertise<grid_map_msgs::GridMap>("localMapColor_debug", 1, true);

        ROS_INFO("Color detector node ready...");
    }

    void color_detection3D_callback(const robot4ws_msgs::ColorDetection3DArray::ConstPtr& msg){ 
        resetGridMap();

        bool updated = false;
        for (const auto& detection : msg->detections){
            updated |= processDetection(detection);
        }

        if (updated) {
            pub_grid_map_update(localMap, msg -> odom);
            publish_gridmap_debug(localMap);
        }
    }

    void pub_grid_map_update(grid_map::GridMap localMap, nav_msgs::Odometry odom_msg){
        grid_map_msgs::GridMap map_msg;
        grid_map::GridMapRosConverter::toMessage(localMap, map_msg);

        robot4ws_mapping::GridMapUpdateMsg update_color_msg;

        update_color_msg.header.stamp = ros::Time::now();
        update_color_msg.header.frame_id = color_layer_name;

        update_color_msg.local_gridmap = map_msg;

        update_color_msg.odom = odom_msg;

        color_map_update_pub.publish(update_color_msg);
    }

    void publish_gridmap_debug(grid_map::GridMap localMap){
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(localMap, message);
        message.info.header.frame_id = "Archimede_footprint";
        grid_map_pub.publish(message);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber color_sub;
    ros::Publisher color_map_update_pub;
    ros::Publisher grid_map_pub;

    grid_map::GridMap localMap;
    std::string grid_map_frame_id;
    
    double cell_size, local_map_size;
    double height_threshold;
    double area_threshold;

    //Topics
    std::string grid_map_update_topic_name, color_detection_topic_name;
    std::string color_layer_name, color_confidence_layer_name;

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
        if (! nh_.getParam("gridmap/grid_map_frame_id",grid_map_frame_id))
        {
            grid_map_frame_id = "Archimede_foot_print";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_frame_id] not found. Using default value: " << grid_map_frame_id);
        }
        
        //Topics
        if (! nh_.getParam("gridmap/grid_map_update_topic_name",grid_map_update_topic_name))
        {
            grid_map_update_topic_name = "grid_map_update";
            ROS_WARN_STREAM("Parameter [gridmap/grid_map_update_topic_name] not found. Using default value: " << grid_map_update_topic_name);
        }
        if (! nh_.getParam("gridmap/color_detection_topic_name",color_detection_topic_name))
        {
            color_detection_topic_name = "color_detection";
            ROS_WARN_STREAM("Parameter [gridmap/color_detection_topic_name] not found. Using default value: " << color_detection_topic_name);
        }
        
        //Layer names
        if (!nh_.getParam("gridmap/color/color_layer_name",color_layer_name))
        {
            color_layer_name = "color";
            ROS_WARN_STREAM("Parameter [gridmap/color_layer_name] not found. Using default value: " << color_layer_name);
        }
        if (!nh_.getParam("gridmap/color/color_confidence_layer_name",color_confidence_layer_name))
        {
            color_confidence_layer_name = "color_confidence";
            ROS_WARN_STREAM("Parameter [gridmap/color_confidence_layer_name] not found. Using default value: " << color_confidence_layer_name);
        }

        //Color detection params
        if (!nh_.getParam("gridmap/color/height_threshold",height_threshold))
        {
            height_threshold = 0.1;
            ROS_WARN_STREAM("Parameter [gridmap/color/height_threshold] not found. Using default value: " << height_threshold);
        }
        if (!nh_.getParam("gridmap/color/area_threshold",area_threshold))
        {
            area_threshold = 0.25;
            ROS_WARN_STREAM("Parameter [gridmap/color/area_threshold] not found. Using default value: " << area_threshold);
        }
    }

    void initLocalMap(){
        localMap.setGeometry(grid_map::Length(local_map_size, local_map_size), cell_size);
        localMap.setFrameId(grid_map_frame_id);
        localMap.add(color_layer_name, robot4ws_mapping::Unknown);
        localMap.add(color_confidence_layer_name, 1e6);
    }

    bool processDetection(const robot4ws_msgs::ColorDetection3D& detection){
        const auto& bbox = detection.detection.bbox;
        const float z = bbox.center.position.z;
        const float area = bbox.size.x * bbox.size.y;
        
        robot4ws_mapping::ColorId color;
        auto it = colorMap.find(detection.color_name);
        if (it != colorMap.end()){
            color = it->second;
        } else {
            ROS_WARN("color_detection_node: Color '%s' not recognized.", detection.color_name.c_str());
            return false;
        }

        //ROS_INFO("z = %f, height_treshold = %f, area = %f, area_tresh = %f", z, height_threshold, area, area_threshold);

        //if (z > height_threshold || area < area_threshold) {
        if(area < area_threshold){
            return processBBox(detection, color);
        }
        else {
            bool check = processContour(detection, color);
            //ROS_INFO("Check result: %s", check ? "true" : "false");
            return check;
        }
    }

    bool processBBox(const robot4ws_msgs::ColorDetection3D& detection, robot4ws_mapping::ColorId color) {
        const auto& bbox = detection.detection.bbox;
        const auto& pos = bbox.center.position;
        const float conf = detection.detection.results[0].score;

        grid_map::Position min_pos(pos.x - bbox.size.x/2, pos.y - bbox.size.y/2);
        grid_map::Position max_pos(pos.x + bbox.size.x/2, pos.y + bbox.size.y/2);
        
        std::vector<grid_map::Position> corners = {
            {min_pos.x(), min_pos.y()},
            {max_pos.x(), min_pos.y()},
            {max_pos.x(), max_pos.y()},
            {min_pos.x(), max_pos.y()}
        };

        return updateGridArea(min_pos, max_pos, corners, conf, color);
    }

    bool processContour(const robot4ws_msgs::ColorDetection3D& detection, robot4ws_mapping::ColorId color) {
        if (detection.contour_points.empty()){
            //ROS_INFO("Empty contour..");
            return false;
        }

        const auto& pos = detection.detection.bbox.center.position;
        const float conf = detection.detection.results[0].score;
        
        std::vector<grid_map::Position> points;
        grid_map::Position min_pos(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        grid_map::Position max_pos(-min_pos.x(), -min_pos.y());

        points.reserve(detection.contour_points.size());
        for (const auto& p : detection.contour_points) {
            grid_map::Position point(p.x, p.y);
            points.push_back(point);
            min_pos = min_pos.cwiseMin(point);
            max_pos = max_pos.cwiseMax(point);
        }
        return updateGridArea(min_pos, max_pos, points, conf, color);
    }

    bool updateGridArea(const grid_map::Position& min_pos, const grid_map::Position& max_pos, const std::vector<grid_map::Position>& points, float confidence, robot4ws_mapping::ColorId color){
        if (points.size() < 3){
            //ROS_INFO("Two few points..");
            return false;
        } 

        const grid_map::Position center = (min_pos + max_pos) * 0.5;
        const double falloff_radius = std::max(max_pos.x() - min_pos.x(), 
                                             max_pos.y() - min_pos.y()) * 0.5;
        
        bool updated = false;
        grid_map::Position pos;
        grid_map::Index idx;

        for (pos.x() = min_pos.x(); pos.x() <= max_pos.x(); pos.x() += cell_size) {
            for (pos.y() = min_pos.y(); pos.y() <= max_pos.y(); pos.y() += cell_size) {
                if (!localMap.isInside(pos) || !isInPolygon(pos, points)) continue;
        
                const double dist = (pos - center).norm();
                const float cell_conf = confidence * std::max(0.0f, float(1.0 - (dist / falloff_radius)));
                
                localMap.getIndex(pos, idx);
                //ROS_INFO("cell_conf: %f, localMap value: %f", cell_conf,localMap.at(color_confidence_layer_name, idx));              
                if (cell_conf < localMap.at(color_confidence_layer_name, idx)){
                    localMap.at(color_layer_name, idx) = color;
                    localMap.at(color_confidence_layer_name, idx) = cell_conf;
                    updated = true;
                }
            }
        }
        return updated;
    }

    bool isInPolygon(const grid_map::Position& point, const std::vector<grid_map::Position>& polygon) const {
        bool inside = false;
        for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            if (((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())) &&
                (point.x() < (polygon[j].x() - polygon[i].x()) * 
                (point.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y()) + 
                polygon[i].x())) {
                inside = !inside;
            }
        }
        return inside;
    }

    void resetGridMap() {
        localMap[color_layer_name].setConstant(robot4ws_mapping::Unknown);
        localMap[color_confidence_layer_name].setConstant(1e6);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_detection_node");

    ColorDetector ColorDetector;

    ros::spin();
    return 0;
}
