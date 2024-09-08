#include <ros/ros.h>
#include "PX4CtrlFSM.h"
#include <signal.h>
#include <yaml-cpp/yaml.h>

#define PI 3.14159

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}


void hover_thrust_cb(const mavros_msgs::TrustMoments::ConstPtr& msg, float *hover_thrust)
{
    *hover_thrust = msg->trust_x;
    // std::cout << "hover thrust : [ " << *hover_thrust << " ]" << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_fusion");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh);

    DFBControl controller(param);
    // buildJPCMFG controller(param);
    std::string body_pose_topic = "odom";
    std::string imu_data_topic  = "imu";
    std::string extrinsic_name  = "extrinsic.yaml";  

    nh.getParam("body_pose_topic",   body_pose_topic);
    nh.getParam("imu_data_topic",    imu_data_topic);
    nh.getParam("extri_params_file", extrinsic_name);
    
    ros::Subscriber pose_sub, twist_sub;
    ros::Publisher  odom_pub, mav_odom_pub;
    
    YAML::Node config = YAML::LoadFile(extrinsic_name);  
    gtsam::Pose3 body_P_vicon;
    double qw = config["body_P_vicon"]["qw"].as<double>();
    double qx = config["body_P_vicon"]["qx"].as<double>();
    double qy = config["body_P_vicon"]["qy"].as<double>();
    double qz = config["body_P_vicon"]["qz"].as<double>();
    double x  = config["body_P_vicon"]["x" ].as<double>();
    double y  = config["body_P_vicon"]["y" ].as<double>();
    double z  = config["body_P_vicon"]["z" ].as<double>();
    body_P_vicon = gtsam::Pose3(gtsam::Rot3(gtsam::Quaternion(qw, qx, qy, qz)), 
        gtsam::Point3(x, y, z)); // qw qx qy qz, x, y, z

    gtsam::Pose3 t265_P_vicon;
    qw = config["t265_P_vicon"]["qw"].as<double>();
    qx = config["t265_P_vicon"]["qx"].as<double>();
    qy = config["t265_P_vicon"]["qy"].as<double>();
    qz = config["t265_P_vicon"]["qz"].as<double>();
    x  = config["t265_P_vicon"]["x" ].as<double>();
    y  = config["t265_P_vicon"]["y" ].as<double>();
    z  = config["t265_P_vicon"]["z" ].as<double>();
    t265_P_vicon = gtsam::Pose3(gtsam::Rot3(gtsam::Quaternion(qw, qx, qy, qz)), 
        gtsam::Point3(x, y, z)); // qw qx qy qz, x, y, z
    gtsam::Pose3 body_P_265 = body_P_vicon* t265_P_vicon.inverse();

    std::cout << "body_P_265: \n";
    body_P_265.print();


    Odom_Data_t odom_data, odom_data_noise;
	Imu_Data_t  imu_raw_data, imu_raw_data_b;

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>(body_pose_topic,
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_raw_sub =
        // nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw",
        nh.subscribe<sensor_msgs::Imu>(imu_data_topic,
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &imu_raw_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());


    ros::Duration(0.5).sleep();

    Controller_Output_t ctrl_cmd;
    bool first_data = true;

    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        if(odom_data.recv_new_msg)
        {
            if(first_data)
            {
                odom_data_noise = odom_data;
                first_data = false;
            }
            else
            {
                odom_data.recv_new_msg = false;
                std::cout << "-------------------------------------  Odom adding noise   ------------------------------------" << std::endl;
                std::cout << "odom_position: " << odom_data.p.transpose()    << std::endl;
                std::cout << "odom_vel: "      << odom_data.v.transpose()    << std::endl;
                std::cout << "acc: "           << imu_raw_data.a.transpose() << std::endl;
                std::cout << "gyro: "          << imu_raw_data.w.transpose() << std::endl;
                odom_data_noise = controller.add_Guassian_noise(odom_data);
                std::cout << "odom_noise_pos: " << odom_data_noise.p.transpose() << std::endl;
                std::cout << "odom_noise_vel: " << odom_data_noise.v.transpose() << std::endl;
            }            

            std::pair<Vector3, Vector3> corrected_imu = UAVFactor::correctMeasurementsBySensorPose(imu_raw_data.a, imu_raw_data.w, body_P_265, boost::none, boost::none, boost::none);
            imu_raw_data_b.a = corrected_imu.first;
            imu_raw_data_b.w = corrected_imu.second;

            controller.fusion(odom_data_noise, imu_raw_data_b, odom_data);
        }
    }

    return 0;
}
