/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <fstream>
#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>

#include "input.h"
#include <Eigen/Dense>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Rot3.h>


struct Desired_State_t
{
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
		: p(odom.p),
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

	//Eigen::Vector3d des_v_real;
};


class SE3Control
{
public:
  SE3Control(Parameter_t &);
  ~SE3Control();
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu, 
      Controller_Output_t &thr_bodyrate_u);
  bool estimateThrustModel(const Eigen::Vector3d &est_v,
      const Parameter_t &param);
  void resetThrustMapping(void);

  void set_hover_thrust(float hover_thrust) { thr2acc_ = param_.gra / hover_thrust; }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

  // Thrust-accel mapping params
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc, const Eigen::Vector3d &v);
  double fromQuaternion2yaw(Eigen::Quaterniond q);
  double limit_value(double upper_bound,  double input, double lower_bound);
  Eigen::Vector3d limit_err(const Eigen::Vector3d err, const double p_err_max);

  std::ofstream log_;
  
};


#endif
