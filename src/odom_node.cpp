#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Rot3.h>
#include <yaml-cpp/yaml.h>
#include "PX4CtrlFSM.h"


gtsam::Pose3                imu_T_vicon;
geometry_msgs::PoseStamped  rev_pose_msg;
geometry_msgs::TwistStamped rev_twist_msg;
nav_msgs::Odometry          send_odom_msg;
ros::Time                   send_last_time;

gtsam::Pose3 fromGeometryPose(const geometry_msgs::Pose& msg)
{
    gtsam::Pose3 pose(gtsam::Rot3(gtsam::Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)), 
        gtsam::Vector3(msg.position.x, msg.position.y, msg.position.z));
    return pose;
}

geometry_msgs::Pose fromGtsamPose(const gtsam::Pose3& pose)
{
    geometry_msgs::Pose msg;
    msg.orientation.w = pose.rotation().toQuaternion().w();
    msg.orientation.x = pose.rotation().toQuaternion().x();
    msg.orientation.y = pose.rotation().toQuaternion().y();
    msg.orientation.z = pose.rotation().toQuaternion().z();

    msg.position.x    = pose.translation().x();
    msg.position.y    = pose.translation().y();
    msg.position.z    = pose.translation().z();
    return msg;
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    send_odom_msg.header.stamp = msg->header.stamp; // ros::Time::now();
    rev_pose_msg = *msg;
}

void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg, const gtsam::Pose3& iTv, int odom_freq, const ros::Publisher& pub)
{
    rev_twist_msg = *msg;
    
    // need to be tested
    gtsam::Pose3 vicon = fromGeometryPose(rev_pose_msg.pose);
    gtsam::Pose3 imu   = vicon* iTv.inverse();
    send_odom_msg.pose.pose    = fromGtsamPose(imu);

    // std::cout << "vicon pose: \n";
    // vicon.print();
    // std::cout << std::endl;

    // std::cout << "imu pose: \n";
    // imu.print();
    // std::cout << std::endl;

    gtsam::Vector3 vicon_vel(rev_twist_msg.twist.linear.x, rev_twist_msg.twist.linear.y, rev_twist_msg.twist.linear.z);
    gtsam::Vector3 vicon_av(rev_twist_msg.twist.angular.x, rev_twist_msg.twist.angular.y, rev_twist_msg.twist.angular.z);
    gtsam::Vector3 imu_vel = vicon_vel + vicon.rotation().matrix()* gtsam::skewSymmetric(vicon_av)* iTv.inverse().translation();
    
    // std::cout << "vicon vel: \n";
    // std::cout << vicon_vel << std::endl;

    // std::cout << "vicon vel: \n";
    // std::cout << imu_vel << std::endl;

    // if(send_odom_msg.header.stamp.toSec() - send_last_time.toSec() >= 1.0 / odom_freq)
    // {
        send_odom_msg.twist.twist.linear.x = imu_vel.x();
        send_odom_msg.twist.twist.linear.y = imu_vel.y();
        send_odom_msg.twist.twist.linear.z = imu_vel.z();
        send_odom_msg.twist.twist.angular  = rev_twist_msg.twist.angular;
        
        // send_last_time = send_odom_msg.header.stamp;
        pub.publish(send_odom_msg);
    // }

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom");
    ros::NodeHandle nh("~");
    Parameter_t param;
    param.config_from_ros_handle(nh);

    int odom_freq = param.odom_freq;

    ros::Duration(1.0).sleep();

    std::string pose_sub_topic  = "/mavros/vision_pose/pose";
    std::string twist_sub_topic = "/vrpn_client_node/Quad13_ipn/twist";
    std::string acc_sub_topic   = "/vrpn_client_node/Quad13_ipn/accel";
    std::string odom_pub_topic  = "/vicon/odom";

    ros::Subscriber pose_sub, twist_sub;
    ros::Publisher  odom_pub;
    
    YAML::Node config = YAML::LoadFile("/home/amov/Fast250/src/px4ctrl/config/extrinsic.yaml");  
    double qw = config["qw"].as<double>();
    double qx = config["qx"].as<double>();
    double qy = config["qy"].as<double>();
    double qz = config["qz"].as<double>();
    double x  = config["x"].as<double>();
    double y  = config["y"].as<double>();
    double z  = config["z"].as<double>();

    imu_T_vicon = gtsam::Pose3(gtsam::Rot3(gtsam::Quaternion(qw, qx, qy, qz)), 
        gtsam::Point3(x, y, z)); // qw qx qy qz, x, y, z
    std::cout << "imu_T_vicon: \n";
    imu_T_vicon.print();

    odom_pub  = nh.advertise<nav_msgs::Odometry>         (odom_pub_topic, 10);
    pose_sub  = nh.subscribe<geometry_msgs::PoseStamped> (pose_sub_topic, 10, boost::bind(pose_callback, _1));
    twist_sub = nh.subscribe<geometry_msgs::TwistStamped>(twist_sub_topic, 10, boost::bind(twist_callback, _1, imu_T_vicon, odom_freq, odom_pub));

    ros::Rate r(100);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }
}