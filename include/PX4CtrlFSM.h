#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <mutex>
#include "TrustMoments.h"

#include "input.h"
// #include "ThrustCurve.h"
#include "controller.h"
#include "JPCM.h"


struct AutoTakeoffLand_t
{
	bool landed{true};
	ros::Time toggle_takeoff_land_time;
	std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};
	Eigen::Vector4d start_pose;
	
	static constexpr double MOTORS_SPEEDUP_TIME = 3.0; // motors idle running for 3 seconds before takeoff
	static constexpr double DELAY_TRIGGER_TIME = 2.0;  // Time to be delayed when reach at target height
};

class PX4CtrlFSM
{
public:
	Parameter_t &param;

	RC_Data_t rc_data;
	State_Data_t state_data;
	ExtendedState_Data_t  extended_state_data;

    // Mutexes for data protection
    std::mutex odom_data_mutex;
    std::mutex gt_mutex;
    std::mutex imu_data_mutex;
    std::mutex imu_raw_data_mutex;
    std::mutex cmd_data_mutex;
    std::mutex bat_data_mutex;
	std::mutex obs_data_mutex; // already exists
	std::mutex rc_data_mutex;
	
	std::vector<Obstacle> obs_data;

	Odom_Data_t    odom_data;
	Odom_Data_t    GT;
	Imu_Data_t     imu_data;
	Imu_Data_t     imu_raw_data;
	// Acc_Data_t     acc_data; // linear acc
	Command_Data_t cmd_data;
	Battery_Data_t bat_data;

	float          hover_thrust;

	Takeoff_Land_Data_t takeoff_land_data;

	Controller &controller;
	// buildJPCMFG &controller;

	ros::Publisher traj_start_trig_pub;
	ros::Publisher ctrl_FCU_pub;
	ros::Publisher debug_pub; //debug
	ros::ServiceClient set_FCU_mode_srv;
	ros::ServiceClient arming_client_srv;
	ros::ServiceClient reboot_FCU_srv;

	quadrotor_msgs::Px4ctrlDebug debug_msg; //debug

	Eigen::Vector4d hover_pose;
	ros::Time last_set_hover_pose_time;

	enum State_t
	{
		MANUAL_CTRL = 1, // JPCM is deactived. FCU is controled by the remote controller only
		AUTO_HOVER, // JPCM is actived, it will keep the drone hover from odom measurments while waiting for commands from PositionCommand topic.
		CMD_CTRL,	// JPCM is actived, and controling the drone.
		AUTO_TAKEOFF,
		AUTO_LAND
	};

	PX4CtrlFSM(Parameter_t &, Controller &);
	
	void process();
	bool rc_is_received(const ros::Time &now_time);
	bool cmd_is_received(const ros::Time &now_time);
	bool odom_is_received(const ros::Time &now_time, const Odom_Data_t &odom);
	bool imu_is_received(const ros::Time &now_time);
	bool bat_is_received(const ros::Time &now_time);
	bool recv_new_odom();
	State_t get_state() { return state; }
	bool get_landed() { return takeoff_land.landed; }

private:
	State_t state; // Should only be changed in PX4CtrlFSM::process() function!
	AutoTakeoffLand_t takeoff_land;

	// ---- control related ----
	Desired_State_t get_hover_des();
	Desired_State_t get_cmd_des();

	// ---- auto takeoff/land ----
	void motors_idling(const Imu_Data_t &imu, Controller_Output_t &thr_bodyrate_u);
	void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom); // Detect landing 
	void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
	Desired_State_t get_rotor_speed_up_des(const ros::Time now);
	Desired_State_t get_takeoff_land_des(const double speed);

	std::vector<Obstacle> get_obs_data_copy() 
    {
        std::lock_guard<std::mutex> lock(obs_data_mutex);
        return obs_data;
    }
    
    void clear_obs_data()
    {
        std::lock_guard<std::mutex> lock(obs_data_mutex);
        obs_data.clear();
    }

	// ---- tools ----
	void set_hov_with_odom(const Odom_Data_t &odom);
	void set_hov_with_rc();

	bool toggle_offboard_mode(bool on_off); // It will only try to toggle once, so not blocked.
	bool toggle_arm_disarm(bool arm); // It will only try to toggle once, so not blocked.
	void reboot_FCU();

	void publish_bodyrate_ctrl(const Controller_Output_t &thr_bodyrate_u, const ros::Time &stamp);
	void publish_attitude_ctrl(const Controller_Output_t &thr_bodyrate_u, const ros::Time &stamp);
	void publish_trigger(const nav_msgs::Odometry &odom_msg);

	CTRL_MODE cvt_ctrl_mode(uint8_t ctrl_mode);
};

#endif