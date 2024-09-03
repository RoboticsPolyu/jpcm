/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <fstream>
#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>

#include "JPCM.h"
#include "input.h"
#include <Eigen/Dense>
#include "factors.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Rot3.h>
#include "type.h"


class DFBControl
{
public:

  using Dist_Dou = std::normal_distribution<double>;

  DFBControl(Parameter_t &);
  
  ~DFBControl();

  quadrotor_msgs::Px4ctrlDebug fusion(const Odom_Data_t &odom, const Imu_Data_t &imu_raw, const Odom_Data_t &GT);
  
  bool initializeState(const std::vector<Imu_Data_t> &imu_raw, const std::vector<Odom_Data_t> &fakeGPS, gtsam::Vector3 &init_vel, gtsam::Vector6 &init_bias);

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

  Odom_Data_t add_Guassian_noise(const Odom_Data_t &odom);

  const Parameter_t & get_param() { return param_; };
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const uint16_t IDX_P_START = 100;
  
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

  // Thrust-accel mapping params
  const double rho2_ = 0.998; // do not change
  double       thr2acc_;
  double       P_;
  
  std::vector<Odom_Data_t> odom_data_v_;
  std::vector<Odom_Data_t> odom_data_noise_;
  std::vector<Imu_Data_t>  imu_data_v_;
  std::vector<Desired_State_t> des_data_v_;

  double fromQuaternion2yaw(Eigen::Quaterniond q);
  double limit_value(double upper_bound,  double input, double lower_bound);
  Eigen::Vector3d limit_err(const Eigen::Vector3d err, const double p_err_max);

  std::shared_ptr<buildJPCMFG> FGbuilder;
  uint64_t   state_idx_ = 0;
  gtsam_fg   graph_;
  gtsam_sols initial_value_;
  bool       init_state_flag_ = false;
  
  gtsam::Vector3 init_vel_;
  gtsam::Vector6 init_bias_;

  double   dt_;
  uint16_t opt_traj_lens_;
  uint16_t window_lens_;

  Dist_Dou position_noise_x;
  Dist_Dou rotation_noise_x;
  Dist_Dou velocity_noise_x;

  Dist_Dou position_noise_y;
  Dist_Dou rotation_noise_y;
  Dist_Dou velocity_noise_y;

  Dist_Dou position_noise_z;
  Dist_Dou rotation_noise_z;
  Dist_Dou velocity_noise_z;

protected:
  std::ofstream log_;
  
};


#endif
