#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <limits>

class CloudFilterNode {
public:
    CloudFilterNode(ros::NodeHandle& nh) : nh_(nh) {
        if (!nh_.getParam("gridmap/local_map_size", local_map_size_)) {
            local_map_size_ = 5.0;
            ROS_WARN_STREAM("Parameter [gridmap/local_map_size] not found. Using default value: " << local_map_size_);
        }
        if (!nh_.getParam("gridmap/lidar_topic_name",lidar_topic_name))
        {
            lidar_topic_name = "/velodyne";
            ROS_WARN_STREAM("Parameter [gridmap/lidar_topic_name] not found. Using default value: " << lidar_topic_name);
        }
        if (!nh_.getParam("gridmap/pc2_filtered_topic_name",pc2_filtered_topic_name))
        {
            pc2_filtered_topic_name = "/velodyne_filtered";
            ROS_WARN_STREAM("Parameter [gridmap/pc2_filtered_topic_name] not found. Using default value: " << pc2_filtered_topic_name);
        }

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pc2_filtered_topic_name, 10);
        cloud_sub_ = nh_.subscribe(lidar_topic_name, 10, &CloudFilterNode::filterPointCloudCallback, this);
    }

private:
    ros::NodeHandle& nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    float local_map_size_;
    std::string lidar_topic_name;
    std::string pc2_filtered_topic_name;

    void filterPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(pcl_cloud);
        float half_map_size = local_map_size_ / 2;

        crop_box_filter.setMin(Eigen::Vector4f(-half_map_size, -half_map_size, -std::numeric_limits<float>::max(), 1.0));
        crop_box_filter.setMax(Eigen::Vector4f(half_map_size, half_map_size, std::numeric_limits<float>::max(), 1.0));

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        crop_box_filter.filter(*filtered_cloud);

        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
        filtered_cloud_msg.header = cloud_msg->header;

        cloud_pub_.publish(filtered_cloud_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cloud_filter_node");
    ros::NodeHandle nh;

    CloudFilterNode cloud_filter_node(nh);

    ros::spin();

    return 0;
}
