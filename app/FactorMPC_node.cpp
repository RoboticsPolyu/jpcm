#include <ros/ros.h>
#include "basic_func.h"
#include "PX4CtrlFSM.h"
#include <signal.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

gtsam::Vector3 stick_xyz;

void hover_thrust_cb(const mavros_msgs::TrustMoments::ConstPtr& msg, float *hover_thrust)
{
    *hover_thrust = msg->trust_x;
    // std::cout << "hover thrust : [ " << *hover_thrust << " ]" << std::endl;
    
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg, PX4CtrlFSM* fsm)
{
    stick_xyz[0] = msg->pose.position.x;
    stick_xyz[1] = msg->pose.position.y;
    stick_xyz[2] = msg->pose.position.z;
}

void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg, PX4CtrlFSM* fsm)
{
    Obstacle obs;
    obs.obs_pos = stick_xyz;
    obs.obs_vel[0] = msg->twist.linear.x;
    obs.obs_vel[1] = msg->twist.linear.y;
    obs.obs_vel[2] = msg->twist.linear.z;
    obs.obs_type = ObsType::sphere;
    obs.obs_size = 0.20;
    obs.timestamp = msg->header.stamp.nsec;

    // Output the obstacle information
    ROS_INFO("=== Obstacle Information ===");
    ROS_INFO("Position: [%.3f, %.3f, %.3f]", 
             obs.obs_pos[0], obs.obs_pos[1], obs.obs_pos[2]);
    ROS_INFO("Velocity: [%.3f, %.3f, %.3f] m/s", 
             obs.obs_vel[0], obs.obs_vel[1], obs.obs_vel[2]);
    ROS_INFO("Type: %d", obs.obs_type);
    ROS_INFO("Size: %.3f m", obs.obs_size);
    ROS_INFO("Timestamp: &u ns", obs.timestamp);
    ROS_INFO("===========================");

    std::lock_guard<std::mutex> lock(fsm->obs_data_mutex);
    fsm->obs_data.clear();
    fsm->obs_data.push_back(obs);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jpcm");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh);

    Controller controller(param);
    // buildJPCMFG controller(param);

    PX4CtrlFSM fsm(param, controller);

    ros::Subscriber pose_sub, twist_sub;
    std::string obs_name = "STICK";
    std::string obs_pose_sub_topic  = "/vrpn_client_node/"; 
    obs_pose_sub_topic.append(obs_name); obs_pose_sub_topic.append("/pose");
    std::string obs_twist_sub_topic = "/vrpn_client_node/"; 
    obs_twist_sub_topic.append(obs_name); obs_twist_sub_topic.append("/twist");

    // pose_sub = nh.subscribe<geometry_msgs::PoseStamped> (obs_pose_sub_topic,  
    //                                                         100, 
    //                                                         boost::bind(pose_callback, _1, &fsm));
    // twist_sub = nh.subscribe<geometry_msgs::TwistStamped>(obs_twist_sub_topic, 
    //                                                         100, 
    //                                                         boost::bind(twist_callback, _1, &fsm));


    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &fsm.state_data, _1));

    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                 10,
                                                 boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data, _1));

    ros::Subscriber gt_sub   =
        nh.subscribe<nav_msgs::Odometry>("GT",
                                         10,
                                         boost::bind(&Odom_Data_t::feed, &fsm.GT, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         10,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      10,
                                                      boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       10,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_raw_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw",
                                       10,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_raw_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber acc_sub =
        nh.subscribe<geometry_msgs::AccelStamped>("acc", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       10,
                                       boost::bind(&Acc_Data_t::feed, &fsm.acc_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber hover_thrust_sub = 
        nh.subscribe<mavros_msgs::TrustMoments>("/mavros/trust_moments_px4", 10, boost::bind(&hover_thrust_cb, _1, &fsm.hover_thrust));

    ros::Subscriber rc_sub;
    if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1));
    }

    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                10,
                                                boost::bind(&Battery_Data_t::feed, &fsm.bat_data, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                                                  100,
                                                  boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());

    fsm.ctrl_FCU_pub        = nh.advertise<mavros_msgs::AttitudeTarget> ("/mavros/setpoint_raw/attitude", 10);
    fsm.traj_start_trig_pub = nh.advertise<geometry_msgs::PoseStamped>  ("/traj_start_trigger", 10);
    fsm.debug_pub           = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug
    fsm.set_FCU_mode_srv    = nh.serviceClient<mavros_msgs::SetMode>    ("/mavros/set_mode");
    fsm.arming_client_srv   = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    fsm.reboot_FCU_srv      = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    
    ros::Duration(0.5).sleep();

    if (param.takeoff_land.no_RC)
    {
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    int trials = 0;
    while (ros::ok() && !fsm.state_data.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }

    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}
