#include <ros/ros.h>
#include "PX4CtrlFSM.h"
#include <signal.h>

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

    Odom_Data_t odom_data, odom_data_noise;
    Imu_Data_t  imu;
	Imu_Data_t  imu_raw_data;
    Imu_Data_t  imu_raw_data_b;

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_raw_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw",
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
                std::cout << "odom_position: " << odom_data.p.transpose() << std::endl;
                std::cout << "odom_vel: "      << odom_data.v.transpose() << std::endl;
                std::cout << "acc: "           << imu_raw_data.a.transpose() << std::endl;
                std::cout << "gyro: "          << imu_raw_data.w.transpose() << std::endl;
                odom_data_noise = controller.add_Guassian_noise(odom_data);
                std::cout << "odom_noise_position: " << odom_data_noise.p.transpose() << std::endl;
                std::cout << "odom_noise_vel: "      << odom_data_noise.v.transpose() << std::endl;
            }

            // std::cout << "Rx PI: " << gtsam::Rot3::Rx(-90.0/180.0*3.14159) *gtsam::Rot3::Ry(-90.0/180.0*3.14159) << std::endl;
            
            imu_raw_data_b = imu_raw_data;

            // imu_raw_data_b.w = gtsam::Rot3::Rx(-90.0/180.0*3.14159) *gtsam::Rot3::Ry(-90.0/180.0*3.14159).rotate(imu_raw_data.w);
            // imu_raw_data_b.a = gtsam::Rot3::Rx(-90.0/180.0*3.14159) *gtsam::Rot3::Ry(-90.0/180.0*3.14159).rotate(imu_raw_data.a);
            controller.fusion(odom_data_noise, imu_raw_data_b, odom_data);
        }
    }

    return 0;
}
