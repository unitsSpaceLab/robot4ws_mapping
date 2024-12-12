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
        link_name_ = "Archimede::Archimede_base_link";

        //ROS node init
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gazebo_2_odom", 10);
        link_states_sub_ = nh_.subscribe("/gazebo/link_states", 10, &LinkStates2Odom::linkStatesCallback, this);

        odom_msg_.header.frame_id = "Archimede_base_link";

        starting_pose_set_ = false;
    }

    void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
        odom_msg_.header.stamp = ros::Time::now();
        auto it = std::find(msg->name.begin(), msg->name.end(), link_name_);
        if (it == msg->name.end()) {
            ROS_WARN("Link %s not found in LinkStates", link_name_.c_str());
            return;
        }

        int index = std::distance(msg->name.begin(), it);
        const auto& msg_pose = msg->pose[index];
        tf2::Vector3 translation(msg_pose.position.x, msg_pose.position.y, msg_pose.position.z);
        tf2::Quaternion rotation(msg_pose.orientation.x, msg_pose.orientation.y, msg_pose.orientation.z, msg_pose.orientation.w);
            
        if (!starting_pose_set_) {
            
            //set z_axis aligned to gravity (gazebo reference frame)
            double roll, pitch, yaw;
            tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);

            pitch = 0.0;

            tf2::Quaternion aligned_rotation;
            aligned_rotation.setRPY(roll, pitch, yaw);
            aligned_rotation.normalize();

            starting_pose = tf2::Transform(aligned_rotation, translation);

            odom_msg_.pose.pose.position.x = 0.0;
            odom_msg_.pose.pose.position.y = 0.0;
            odom_msg_.pose.pose.position.z = 0.0;

            odom_msg_.pose.pose.orientation.x = aligned_rotation.x();
            odom_msg_.pose.pose.orientation.y = aligned_rotation.y();
            odom_msg_.pose.pose.orientation.z = aligned_rotation.z();
            odom_msg_.pose.pose.orientation.w = aligned_rotation.w();

            odom_msg_.twist.twist.linear.x = 0.0;
            odom_msg_.twist.twist.linear.y = 0.0;
            odom_msg_.twist.twist.linear.z = 0.0;

            odom_msg_.twist.twist.angular.x = 0.0;
            odom_msg_.twist.twist.angular.y = 0.0;
            odom_msg_.twist.twist.angular.z = 0.0;

            starting_pose_set_ = true;

           /*  starting_pose = tf2::Transform(rotation, translation);
            // starting_position_ = msg->pose[index].position;
            //starting_orientation_ = msg->pose[index].orientation; 

            tf2::Matrix3x3 rot_matrix(rotation);
            tf2::Vector3 z_axis = rot_matrix.getColumn(2); // Asse Z dell'orientamento corrente

            // Calcolo angolo tra asse Z corrente e asse Z globale
            tf2::Vector3 global_z(0.0, 0.0, 1.0);
            double angle = acos(z_axis.dot(global_z) / (z_axis.length() * global_z.length()));
            tf2::Vector3 axis = z_axis.cross(global_z).normalized();

            // Se l'asse di rotazione è valido, corregge la rotazione
            tf2::Quaternion correction;
            if (axis.length() > 0.0) {
                correction.setRotation(axis, angle);
            } else {
                correction = tf2::Quaternion(0.0, 0.0, 0.0, 1.0); 
            }

            // Applica la correzione all'orientamento originale
            tf2::Quaternion aligned_rotation = correction * rotation;
            aligned_rotation.normalize();

            // Aggiorna la starting_pose
            starting_pose.setRotation(aligned_rotation);


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

            starting_pose_set_ = true; */
        } else {
            /* odom_msg_.pose.pose.position.x = pose.position.x - starting_position_.x;
            odom_msg_.pose.pose.position.y = pose.position.y - starting_position_.y;
            odom_msg_.pose.pose.position.z = pose.position.z - starting_position_.z;

            tf2::Quaternion q_current(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            tf2::Quaternion q_start(starting_orientation_.x, starting_orientation_.y, starting_orientation_.z, starting_orientation_.w);
            tf2::Quaternion q_relative = q_start.inverse() * q_current;

            odom_msg_.pose.pose.orientation.x = q_relative.x();
            odom_msg_.pose.pose.orientation.y = q_relative.y();
            odom_msg_.pose.pose.orientation.z = q_relative.z();
            odom_msg_.pose.pose.orientation.w = q_relative.w(); */
 
            tf2::Transform pose = tf2::Transform(rotation, translation);
            tf2::Transform relative_transform = starting_pose.inverse() * pose;

            odom_msg_.pose.pose.position.x = relative_transform.getOrigin().x();
            odom_msg_.pose.pose.position.y = relative_transform.getOrigin().y();
            odom_msg_.pose.pose.position.z = relative_transform.getOrigin().z();
            odom_msg_.pose.pose.orientation.x = relative_transform.getRotation().x();
            odom_msg_.pose.pose.orientation.y = relative_transform.getRotation().y();
            odom_msg_.pose.pose.orientation.z = relative_transform.getRotation().z();
            odom_msg_.pose.pose.orientation.w = relative_transform.getRotation().w();
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
    /* geometry_msgs::Point starting_position_;
    geometry_msgs::Quaternion starting_orientation_; */
    tf2::Transform starting_pose;
    bool starting_pose_set_;

    double link_height;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "link_states_to_odometry");
    LinkStates2Odom node;
    ros::spin();
    return 0;
}
