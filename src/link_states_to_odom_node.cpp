#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>


class LinkStates2Odom {
public:
    LinkStates2Odom() {
        // Nome del link di interesse
        //TODO: eventually change to footprint
        link_name_ = "Archimede::Archimede_base_link";

        // Inizializza il nodo ROS
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gazebo_2_odom", 10);
        link_states_sub_ = nh_.subscribe("/gazebo/link_states", 10, &LinkStates2Odom::linkStatesCallback, this);

        // Inizializza il messaggio Odometry
        // odom_msg_.header.frame_id = "Archimede_base_link_start";
        odom_msg_.header.frame_id = "Archimede_base_link";
        // odom_msg_.child_frame_id = "Archimede_base_link";

       /*  tf_.header.frame_id = "Archimede_base_link_start";
        tf_.child_frame_id = "Archimede_base_link"; */

        starting_position_set_ = false;
    }

    void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
        odom_msg_.header.stamp = ros::Time::now();
        auto it = std::find(msg->name.begin(), msg->name.end(), link_name_);
        if (it == msg->name.end()) {
            ROS_WARN("Link %s not found in LinkStates", link_name_.c_str());
            return;
        }

        if (!starting_position_set_) {
            int index = std::distance(msg->name.begin(), it);
            starting_position_ = msg->pose[index].position;
            starting_orientation_ = msg->pose[index].orientation;

            odom_msg_.pose.pose.position.x = 0.0;
            odom_msg_.pose.pose.position.y = 0.0;
            odom_msg_.pose.pose.position.z = 0.0;

            odom_msg_.pose.pose.orientation.x = 0.0;
            odom_msg_.pose.pose.orientation.y = 0.0;
            odom_msg_.pose.pose.orientation.z = 0.0;
            odom_msg_.pose.pose.orientation.w = 1.0;

            odom_msg_.twist.twist.linear.x = 0.0;
            odom_msg_.twist.twist.linear.y = 0.0;
            odom_msg_.twist.twist.linear.z = 0.0;

            odom_msg_.twist.twist.angular.x = 0.0;
            odom_msg_.twist.twist.angular.y = 0.0;
            odom_msg_.twist.twist.angular.z = 0.0;

            starting_position_set_ = true;
        } else {
            int index = std::distance(msg->name.begin(), it);

            const auto& pose = msg->pose[index];
            odom_msg_.pose.pose.position.x = pose.position.x - starting_position_.x;
            odom_msg_.pose.pose.position.y = pose.position.y - starting_position_.y;
            odom_msg_.pose.pose.position.z = pose.position.z - starting_position_.z;

            tf2::Quaternion q_current(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            tf2::Quaternion q_start(starting_orientation_.x, starting_orientation_.y, starting_orientation_.z, starting_orientation_.w);
            tf2::Quaternion q_relative = q_start.inverse() * q_current;

            odom_msg_.pose.pose.orientation.x = q_relative.x();
            odom_msg_.pose.pose.orientation.y = q_relative.y();
            odom_msg_.pose.pose.orientation.z = q_relative.z();
            odom_msg_.pose.pose.orientation.w = q_relative.w();
        }

        odom_pub_.publish(odom_msg_);
        pubOdomTF(odom_msg_);
    }

private:
    void pubOdomTF(const nav_msgs::Odometry& msg) {
        tf_.header.stamp = msg.header.stamp;
        tf_.header.frame_id = "Archimede_foot_start";
        tf_.child_frame_id = "Archimede_footprint";

        // Imposta la traslazione
        tf_.transform.translation.x = msg.pose.pose.position.x;
        tf_.transform.translation.y = msg.pose.pose.position.y;
        tf_.transform.translation.z = msg.pose.pose.position.z;

        // Imposta la rotazione
        tf_.transform.rotation.x = msg.pose.pose.orientation.x;
        tf_.transform.rotation.y = msg.pose.pose.orientation.y;
        tf_.transform.rotation.z = msg.pose.pose.orientation.z;
        tf_.transform.rotation.w = msg.pose.pose.orientation.w;

        tf_broadcaster_.sendTransform(tf_);
    }

    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber link_states_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    nav_msgs::Odometry odom_msg_;
    geometry_msgs::TransformStamped tf_;

    std::string link_name_;
    geometry_msgs::Point starting_position_;
    geometry_msgs::Quaternion starting_orientation_;
    bool starting_position_set_;

    double link_height;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "link_states_to_odometry");
    LinkStates2Odom node;
    ros::spin();
    return 0;
}
