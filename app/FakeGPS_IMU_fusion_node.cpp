#include "basic_func.h"
#include <ros/ros.h>
#include "PX4CtrlFSM.h"
#include <signal.h>
#include <yaml-cpp/yaml.h>

/*
 * FakeGPS 100Hz
 * IMU
 * Rotation constraint
 */

void mySigintHandler(int sig)
{
    ROS_INFO("[FakeGPS_IMU_fusion] exit...");
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "FakeGPS_IMU_fusion");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh);

    Controller controller(param);

    std::string body_pose_topic = "odom";
    std::string imu_data_topic  = "imu";
    std::string extrinsic_name  = "extrinsic.yaml";  
    std::string body_fusion_tpc = "self_fusion";

    nh.getParam("body_pose_topic",   body_pose_topic);
    nh.getParam("imu_data_topic",    imu_data_topic);
    nh.getParam("extri_params_file", extrinsic_name);
    nh.getParam("body_pose_fusion",  body_fusion_tpc);
    
    YAML::Node config = YAML::LoadFile(extrinsic_name);  
    gtsam::Pose3 body_P_vicon; // pix_P_vicon
    double qw = config["body_P_vicon"]["qw"].as<double>();
    double qx = config["body_P_vicon"]["qx"].as<double>();
    double qy = config["body_P_vicon"]["qy"].as<double>();
    double qz = config["body_P_vicon"]["qz"].as<double>();
    double x  = config["body_P_vicon"]["x" ].as<double>();
    double y  = config["body_P_vicon"]["y" ].as<double>();
    double z  = config["body_P_vicon"]["z" ].as<double>();
    body_P_vicon = gtsam::Pose3(gtsam::Rot3(gtsam::Quaternion(qw, qx, qy, qz)), 
        gtsam::Point3(x, y, z)); 

    gtsam::Pose3 t265_P_vicon; // t265imu_P_vicon
            qw = config["t265_P_vicon"]["qw"].as<double>();
            qx = config["t265_P_vicon"]["qx"].as<double>();
            qy = config["t265_P_vicon"]["qy"].as<double>();
            qz = config["t265_P_vicon"]["qz"].as<double>();
            x  = config["t265_P_vicon"]["x" ].as<double>();
            y  = config["t265_P_vicon"]["y" ].as<double>();
            z  = config["t265_P_vicon"]["z" ].as<double>();
    t265_P_vicon = gtsam::Pose3(gtsam::Rot3(gtsam::Quaternion(qw, qx, qy, qz)), 
        gtsam::Point3(x, y, z)); 
    gtsam::Pose3 body_P_265 = body_P_vicon* t265_P_vicon.inverse();

    std::cout << "body_P_265: \n";
    body_P_265.print();

    Odom_Data_t odom_data, odom_data_noise;
	Imu_Data_t  imu_raw_data;

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>(body_pose_topic,
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_raw_sub =
        nh.subscribe<sensor_msgs::Imu>(imu_data_topic,
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &imu_raw_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
    
    nav_msgs::Odometry send_odom_msg;
    ros::Publisher odom_pub =
        nh.advertise<nav_msgs::Odometry>(body_fusion_tpc, 100);
    
    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        if(odom_data.recv_new_msg)
        {
            odom_data.recv_new_msg = false;
            odom_data_noise = controller.add_Guassian_noise(odom_data);           

            std::pair<Vector3, Vector3> corrected_imu = 
                uavfactor::correctMeasurementsBySensorPose(imu_raw_data.a, imu_raw_data.w, body_P_265, boost::none, boost::none, boost::none);
            imu_raw_data.a = corrected_imu.first;
            imu_raw_data.w = corrected_imu.second;

            gtsam::Pose3 fus_pose;
            gtsam::Vector3 fus_vel, fus_w;
            controller.fusion(odom_data_noise, imu_raw_data, odom_data, fus_pose, fus_vel, fus_w);

            send_odom_msg.header.stamp          = odom_data.rcv_stamp;
            send_odom_msg.pose.pose             = fromGtsamPose(fus_pose);
            send_odom_msg.twist.twist.linear.x  = fus_vel.x();
            send_odom_msg.twist.twist.linear.y  = fus_vel.y();
            send_odom_msg.twist.twist.linear.z  = fus_vel.z();
            send_odom_msg.twist.twist.angular.x = fus_w.x();
            send_odom_msg.twist.twist.angular.y = fus_w.y();
            send_odom_msg.twist.twist.angular.z = fus_w.z();
            odom_pub.publish(send_odom_msg);
        }
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
