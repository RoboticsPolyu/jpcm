/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file IMUKittiExampleGPS
 * @brief Example of application of ISAM2 for GPS-aided navigation on the KITTI
 * VISION BENCHMARK SUITE
 * @author Ported by Thomas Jespersen (thomasj@tkjelectronics.dk), TKJ
 * Electronics
 */

// GTSAM related includes.
#include "gps_imu.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gps_imu");
    ros::NodeHandle nh("~");
    Parameter_t param;
    param.config_from_ros_handle(nh);

    GIO_Params gio_params;
    GIO gio(gio_params);

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &gio.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
    
    ros::Subscriber imu_raw_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw",
                                       100,
                                       boost::bind(&Imu_Datas_t::feed, &gio.imu_raw_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
    
    ros::Rate r(100);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }
}