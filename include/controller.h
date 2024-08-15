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
#include "factors.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Rot3.h>


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


class DFBControl
{
public:
  enum CTRL_MODE
  {
    DFBC = 0x01,
    MPC,
    JPCM,
    MAX
  };

  DFBControl(Parameter_t &);
  
  ~DFBControl();

  quadrotor_msgs::Px4ctrlDebug fusion(const Odom_Data_t &odom, const Imu_Data_t &imu_raw, const Odom_Data_t &GT);

  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu, Controller_Output_t &thr_bodyrate_u);

  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu, 
    Controller_Output_t &thr_bodyrate_u, CTRL_MODE mode_switch);

  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu, const Imu_Data_t &imu_raw, 
    Controller_Output_t &thr_bodyrate_u, CTRL_MODE mode_switch);
  
  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc, const Eigen::Vector3d &v);

  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
  
  bool estimateThrustModel(const Eigen::Vector3d &est_v, const Parameter_t &param);
  
  void resetThrustMapping(void);

  void set_hover_thrust(float hover_thrust) { thr2acc_ = param_.gra / hover_thrust; }

  Odom_Data_t addNoise(const Odom_Data_t &odom);

  const Parameter_t & get_param() { return param_; };
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const uint16_t IDX_START = 100;
  
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

  // Thrust-accel mapping params
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  uint64_t state_idx_ = 0;
  
  std::vector<Odom_Data_t> odom_data_v_;
  std::vector<Odom_Data_t> odom_data_noise_;
  std::vector<Imu_Data_t>  imu_data_v_;

  double fromQuaternion2yaw(Eigen::Quaterniond q);
  double limit_value(double upper_bound,  double input, double lower_bound);
  Eigen::Vector3d limit_err(const Eigen::Vector3d err, const double p_err_max);

  void buildFactorGraph(gtsam::NonlinearFactorGraph& _graph, gtsam::Values& _initial_value, 
                        const std::vector<Desired_State_t> &des_v, const Odom_Data_t &odom, double dt);

  void buildFactorGraph(gtsam::NonlinearFactorGraph& _graph, gtsam::Values& _initial_value, 
                        const std::vector<Desired_State_t> &des_v, const std::vector<Odom_Data_t> &odom_v, const std::vector<Imu_Data_t> &imu_v, double dt);

  void  buildFactorGraph(gtsam::NonlinearFactorGraph& _graph, gtsam::Values& _initial_value, 
                        const std::vector<Odom_Data_t> &odom_v, 
                        const std::vector<Imu_Data_t> &imu_v, double dt);

  std::vector<Desired_State_t> des_data_v_;
  gtsam::NonlinearFactorGraph  graph_positioning_;
  gtsam::Values                initial_value_positioning_;

  gtsam::NonlinearFactorGraph  graph_;
  gtsam::Values                initial_value_;

  double                       dt_;
  uint16_t                     opt_traj_lens_;
  uint16_t                     window_lens_;

  std::default_random_engine meas_x_gen;
  std::default_random_engine meas_y_gen;
  std::default_random_engine meas_z_gen;
  
  std::default_random_engine meas_rx_gen;
  std::default_random_engine meas_ry_gen;
  std::default_random_engine meas_rz_gen;

  std::default_random_engine meas_vx_gen;
  std::default_random_engine meas_vy_gen;
  std::default_random_engine meas_vz_gen;

  std::normal_distribution<double> position_noise_x;
  std::normal_distribution<double> rotation_noise_x;
  std::normal_distribution<double> velocity_noise_x;

  std::normal_distribution<double> position_noise_y;
  std::normal_distribution<double> rotation_noise_y;
  std::normal_distribution<double> velocity_noise_y;

  std::normal_distribution<double> position_noise_z;
  std::normal_distribution<double> rotation_noise_z;
  std::normal_distribution<double> velocity_noise_z;

protected:
  std::ofstream log_;
  
};


#endif
