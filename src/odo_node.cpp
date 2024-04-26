#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "PX4CtrlFSM.h"

geometry_msgs::PoseStamped  rev_pose_msg;
geometry_msgs::TwistStamped rev_twist_msg;
nav_msgs::Odometry          send_odom_msg;


void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    rev_pose_msg = *msg;
    // std::cout << "rec: Timestamp: " << rev_pose_msg.header.stamp.toNSec() << std::endl;
    // std::cout << "rev: position [ " << rev_pose_msg.pose.position.x << " ," 
    // << rev_pose_msg.pose.position.y << " ," 
    // << rev_pose_msg.pose.position.z << "]" << std::endl;
    // std::cout << "rev: rotation xyzw [ " << rev_pose_msg.pose.orientation.x << " ," 
    // << rev_pose_msg.pose.orientation.y << " ," 
    // << rev_pose_msg.pose.orientation.z << " ," 
    // << rev_pose_msg.pose.orientation.w << "]" << std::endl;
}

void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg, const ros::Publisher& pub)
{
    rev_twist_msg = *msg;
    // std::cout << "rec: Timestamp: " << rev_twist_msg.header.stamp.toNSec() << std::endl;
    // std::cout << "rev: twist linear  [ " << rev_twist_msg.twist.linear.x << " ," 
    // << rev_twist_msg.twist.linear.y << " ," 
    // << rev_twist_msg.twist.linear.z << "]" << std::endl;
    // std::cout << "rev: twist angular [ " << rev_twist_msg.twist.angular.x << " ," 
    // << rev_twist_msg.twist.angular.y << " ," 
    // << rev_twist_msg.twist.angular.z << "]" << std::endl;

    send_odom_msg.header.stamp = ros::Time::now();
    send_odom_msg.pose.pose    = rev_pose_msg.pose;
    send_odom_msg.twist.twist  = rev_twist_msg.twist;
    pub.publish(send_odom_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odo_node");
    ros::NodeHandle nh("~");

    ros::Duration(1.0).sleep();

    std::string pose_sub_topic  = "/mavros/vision_pose/pose";
    std::string twist_sub_topic = "/vrpn_client_node/Quad13_ipn/twist";
    std::string acc_sub_topic   = "/vrpn_client_node/Quad13_ipn/accel";
    std::string odom_pub_topic  = "/vicon/odom";

    ros::Subscriber pose_sub, twist_sub;
    ros::Publisher  odom_pub;


    odom_pub  = nh.advertise<nav_msgs::Odometry>         (odom_pub_topic, 10);
    pose_sub  = nh.subscribe<geometry_msgs::PoseStamped> (pose_sub_topic, 10, boost::bind(pose_callback, _1));
    twist_sub = nh.subscribe<geometry_msgs::TwistStamped>(twist_sub_topic, 10, boost::bind(twist_callback, _1, odom_pub));
    

    ros::Rate r(100);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }
}