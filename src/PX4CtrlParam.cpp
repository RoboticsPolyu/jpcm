#include "PX4CtrlParam.h"

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
	read_essential_param(nh, "gain/Kp0", gain.Kp0);
	read_essential_param(nh, "gain/Kp1", gain.Kp1);
	read_essential_param(nh, "gain/Kp2", gain.Kp2);
	read_essential_param(nh, "gain/Kv0", gain.Kv0);
	read_essential_param(nh, "gain/Kv1", gain.Kv1);
	read_essential_param(nh, "gain/Kv2", gain.Kv2);
	read_essential_param(nh, "gain/Kvi0", gain.Kvi0);
	read_essential_param(nh, "gain/Kvi1", gain.Kvi1);
	read_essential_param(nh, "gain/Kvi2", gain.Kvi2);
	read_essential_param(nh, "gain/KAngR", gain.KAngR);
	read_essential_param(nh, "gain/KAngP", gain.KAngP);
	read_essential_param(nh, "gain/KAngY", gain.KAngY);
	read_essential_param(nh, "gain/PErrMax", gain.PErrMax);
	read_essential_param(nh, "gain/VErrMax", gain.VErrMax);

	read_essential_param(nh, "rotor_drag/x", rt_drag.x);
	read_essential_param(nh, "rotor_drag/y", rt_drag.y);
	read_essential_param(nh, "rotor_drag/z", rt_drag.z);
	read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
	read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
	read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

	read_essential_param(nh, "pose_solver", pose_solver);
	read_essential_param(nh, "mass", mass);
	read_essential_param(nh, "gra", gra);
	read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
	read_essential_param(nh, "max_manual_vel", max_manual_vel);
	read_essential_param(nh, "max_angle", max_angle);
	read_essential_param(nh, "low_voltage", low_voltage);
	read_essential_param(nh, "odom_freq", odom_freq);

	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

	read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
	read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
	read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

	read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
	read_essential_param(nh, "thrust_model/K1", thr_map.K1);
	read_essential_param(nh, "thrust_model/K2", thr_map.K2);
	read_essential_param(nh, "thrust_model/K3", thr_map.K3);
	read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
	read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);
	read_essential_param(nh, "thrust_model/thrust_upper_bound", thr_map.thrust_upper_bound);
	read_essential_param(nh, "thrust_model/thrust_lower_bound", thr_map.thrust_lower_bound);
	
	read_essential_param(nh, "qw", qw);
	read_essential_param(nh, "qw", qx);
	read_essential_param(nh, "qw", qy);
	read_essential_param(nh, "qw", qz);
	read_essential_param(nh, "qw", x);
	read_essential_param(nh, "qw", y);
	read_essential_param(nh, "qw", z);

	read_essential_param(nh, "Factor_graph/LOG_NAME", factor_graph.LOG_NAME);

	read_essential_param(nh, "Factor_graph/PRI_VICON_COV", factor_graph.PRI_VICON_COV);
	read_essential_param(nh, "Factor_graph/PRI_VICON_VEL_COV", factor_graph.PRI_VICON_VEL_COV);

	read_essential_param(nh, "Factor_graph/CONTROL_P_COV_X", factor_graph.CONTROL_P_COV_X);
	read_essential_param(nh, "Factor_graph/CONTROL_PF_COV_X", factor_graph.CONTROL_PF_COV_X);
	read_essential_param(nh, "Factor_graph/CONTROL_P_COV_Y", factor_graph.CONTROL_P_COV_Y);
	read_essential_param(nh, "Factor_graph/CONTROL_PF_COV_Y", factor_graph.CONTROL_PF_COV_Y);
	read_essential_param(nh, "Factor_graph/CONTROL_P_COV_Z", factor_graph.CONTROL_P_COV_Z);
	read_essential_param(nh, "Factor_graph/CONTROL_PF_COV_Z", factor_graph.CONTROL_PF_COV_Z);
	read_essential_param(nh, "Factor_graph/CONTROL_R1_COV", factor_graph.CONTROL_R1_COV);
	read_essential_param(nh, "Factor_graph/CONTROL_R2_COV", factor_graph.CONTROL_R2_COV);
	read_essential_param(nh, "Factor_graph/CONTROL_R3_COV", factor_graph.CONTROL_R3_COV);
	read_essential_param(nh, "Factor_graph/CONTROL_V_COV", factor_graph.CONTROL_V_COV);
	read_essential_param(nh, "Factor_graph/DYNAMIC_P_COV", factor_graph.DYNAMIC_P_COV);
	read_essential_param(nh, "Factor_graph/DYNAMIC_R_COV", factor_graph.DYNAMIC_R_COV);
	read_essential_param(nh, "Factor_graph/DYNAMIC_V_COV", factor_graph.DYNAMIC_V_COV);
	
	read_essential_param(nh, "Factor_graph/INPUT_JERK_T", factor_graph.INPUT_JERK_T);
	read_essential_param(nh, "Factor_graph/INPUT_JERK_M", factor_graph.INPUT_JERK_M);
	read_essential_param(nh, "Factor_graph/INPUT_JERK_M3", factor_graph.INPUT_JERK_M3);
	
	read_essential_param(nh, "Factor_graph/OPT_LENS_TRAJ", factor_graph.OPT_LENS_TRAJ);
	read_essential_param(nh, "Factor_graph/WINDOW_SIZE", factor_graph.WINDOW_SIZE);

	read_essential_param(nh, "Factor_graph/CLF_HIGH", factor_graph.high);
	read_essential_param(nh, "Factor_graph/CLF_LOW", factor_graph.low);
	read_essential_param(nh, "Factor_graph/CLF_THR", factor_graph.thr);

	read_essential_param(nh, "Factor_graph/G_CLF_HIGH", factor_graph.ghigh);
	read_essential_param(nh, "Factor_graph/G_CLF_LOW", factor_graph.glow);
	read_essential_param(nh, "Factor_graph/G_CLF_THR", factor_graph.gthr);

	read_essential_param(nh, "Factor_graph/CLF_ALPHA", factor_graph.alpha);

	read_essential_param(nh, "Factor_graph/POS_MEAS_MEAN", factor_graph.POS_MEAS_MEAN);
	read_essential_param(nh, "Factor_graph/POS_MEAS_COV", factor_graph.POS_MEAS_COV);
	read_essential_param(nh, "Factor_graph/VEL_MEAS_COV", factor_graph.VEL_MEAS_COV);
	read_essential_param(nh, "Factor_graph/ROT_MEAS_COV", factor_graph.ROT_MEAS_COV);

	max_angle /= (180.0 / M_PI);

	if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
	{
		takeoff_land.enable_auto_arm = false;
		ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
	}
	if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
	{
		takeoff_land.no_RC = false;
		ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
	}

	if ( thr_map.print_val )
	{
		ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
	}
};

// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };
