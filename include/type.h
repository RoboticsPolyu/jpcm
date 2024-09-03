/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __TYPE_H__
#define __TYPE_H__

#include <Eigen/Dense>
#include "input.h"
#include <ros/ros.h>

struct Desired_State_t
{
    ros::Time rcv_stamp;
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	Eigen::Vector3d w;
	double yaw;
	double yaw_rate;

	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: rcv_stamp(ros::Time(0)),
          p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

  // Body rates in body frame
	Eigen::Vector3d mpc_bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double mpc_thrust;

	//Eigen::Vector3d des_v_real;
};

enum CTRL_MODE
{
  DFBC = 0x01,
  MPC,
  JPCM,
  MAX
};


#endif // __TYPE_H__