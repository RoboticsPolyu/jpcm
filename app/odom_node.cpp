#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include "basic_func.h"
#include "PX4CtrlFSM.h"

gtsam::Pose3                body_P_vicon;
geometry_msgs::PoseStamped  rev_pose_msg;
geometry_msgs::PoseStamped  send_mav_pos_msg;
geometry_msgs::TwistStamped rev_twist_msg;
nav_msgs::Odometry          send_odom_msg;
ros::Time                   send_last_time;

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    send_odom_msg.header.stamp = msg->header.stamp; // ros::Time::now();
    send_mav_pos_msg.header.stamp = msg->header.stamp;
    rev_pose_msg               = *msg;
}

void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg, const gtsam::Pose3& iTv, int odom_freq, 
    const ros::Publisher& pub, const ros::Publisher& mav_pub)
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
        send_mav_pos_msg.pose = fromGtsamPose(imu);
        mav_pub.publish(send_mav_pos_msg);
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

    std::string pose_sub_topic  = "/vrpn_client_node/Quad13_ipn/pose";
    std::string twist_sub_topic = "/vrpn_client_node/Quad13_ipn/twist";
    std::string acc_sub_topic   = "/vrpn_client_node/Quad13_ipn/accel";
    std::string odom_pub_topic  = "/vicon/odom";
    std::string mav_pub_topic   = "/mavros/vision_pose/pose";
    std::string extrinsic_name  = "extrinsic.yaml";  

    nh.getParam("vicon_pose_topic",  pose_sub_topic);
    nh.getParam("vicon_twist_topic", twist_sub_topic);
    nh.getParam("vicon_acc_topic",   acc_sub_topic);
    nh.getParam("extri_params_file", extrinsic_name);
    nh.getParam("mav_pose_topic",    mav_pub_topic);

    ros::Subscriber pose_sub, twist_sub;
    ros::Publisher  odom_pub, mav_odom_pub;
    
    YAML::Node config = YAML::LoadFile(extrinsic_name);  
    double qw = config["body_P_vicon"]["qw"].as<double>();
    double qx = config["body_P_vicon"]["qx"].as<double>();
    double qy = config["body_P_vicon"]["qy"].as<double>();
    double qz = config["body_P_vicon"]["qz"].as<double>();
    double x  = config["body_P_vicon"]["x" ].as<double>();
    double y  = config["body_P_vicon"]["y" ].as<double>();
    double z  = config["body_P_vicon"]["z" ].as<double>();

    body_P_vicon = gtsam::Pose3(gtsam::Rot3(gtsam::Quaternion(qw, qx, qy, qz)), 
        gtsam::Point3(x, y, z)); // qw qx qy qz, x, y, z
    std::cout << "body_P_vicon: \n";
    body_P_vicon.print();

    odom_pub     = nh.advertise<nav_msgs::Odometry>         (odom_pub_topic,  100);
    mav_odom_pub = nh.advertise<geometry_msgs::PoseStamped> (mav_pub_topic,   100);
    pose_sub     = nh.subscribe<geometry_msgs::PoseStamped> (pose_sub_topic,  100, boost::bind(pose_callback, _1));
    twist_sub    = nh.subscribe<geometry_msgs::TwistStamped>(twist_sub_topic, 100, boost::bind(twist_callback, _1, body_P_vicon, odom_freq, odom_pub, mav_odom_pub));

    ros::Rate r(100);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }
}