#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>

class Parameter_t
{
public:
	struct Gain
	{
		double Kp0, Kp1, Kp2;
		double Kv0, Kv1, Kv2;
		double Kvi0, Kvi1, Kvi2;
		double Kvd0, Kvd1, Kvd2;
		double KAngR, KAngP, KAngY;
		double PErrMax, VErrMax;
	};

	struct RotorDrag
	{
		double x, y, z;
		double k_thrust_horz;
	};

	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
		double imu;
		double bat;
	};

	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
		double thrust_upper_bound;
		double thrust_lower_bound;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	struct AutoTakeoffLand
	{
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
	};

	// factor graph params
	struct FactorGraph
	{
		std::string LOG_NAME;

		double PRI_VICON_POS_COV;
		double PRI_VICON_VEL_COV; 

		double CONTROL_P_COV_X;
		double CONTROL_P_COV_Y;
		double CONTROL_P_COV_Z;
		double CONTROL_PF_COV_X;
		double CONTROL_PF_COV_Y;
		double CONTROL_PF_COV_Z;
		double CONTROL_V_COV;
		double DYNAMIC_P_COV; 
		double DYNAMIC_R_COV;
		double DYNAMIC_V_COV;
		double CONTROL_R1_COV;
		double CONTROL_R2_COV;
		double CONTROL_R3_COV;

		double INPUT_JERK_T; 
		double INPUT_JERK_M; 
		double INPUT_JERK_M3; 
		
		int OPT_LENS_TRAJ;
		int WINDOW_SIZE;

		double high; 
		double low; 
		double thr; 

		double ghigh; 
		double glow; 
		double gthr; 

		double alpha; 

		double POS_MEAS_MEAN;
		double POS_MEAS_COV;
		double VEL_MEAS_COV;
		double ROT_MEAS_COV;

		double PRIOR_POS_MEAS_COV;
		double PRIOR_VEL_MEAS_COV;
		double PRIOR_ROT_MEAS_COV;

		double acc_sigma_x;
		double acc_bias_imu_x;
		double acc_sigma_y;
		double acc_bias_imu_y;
		double acc_sigma_z;
		double acc_bias_imu_z;

		double gyro_sigma_x;
		double gyro_bias_sigma_x;
		double gyro_sigma_y;
		double gyro_bias_sigma_y;
		double gyro_sigma_z;
		double gyro_bias_sigma_z;

		double prior_acc_sigma;
		double prior_gyro_sigma;
	};

	Gain gain;
	RotorDrag rt_drag;
	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	ThrustMapping thr_map;
	AutoTakeoffLand takeoff_land;
	FactorGraph factor_graph;
	int ctrl_mode;

	int pose_solver;
	double mass;
	double gra;
	double max_angle;
	double ctrl_freq_max;
	double max_manual_vel;
	double low_voltage;
	int    odom_freq;
	
	double qw, qx, qy, qz, x, y, z;
	
	bool use_bodyrate_ctrl;
	// bool print_dbg;

	Parameter_t();
	void config_from_ros_handle(const ros::NodeHandle &nh);
	void config_full_thrust(double hov);

private:
	template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};

#endif