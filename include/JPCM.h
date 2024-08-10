#ifndef __JPCM_H__
#define __JPCM_H__

#include "controller.h"
#include "factors.h"
#include "GTSAMUtils.h"
#include "Marginalization.h"

class JCPM_TGyro
{
public:

  JCPM_TGyro(Parameter_t &param);

  bool JPCM_control(const std::vector<Desired_State_t> &des, 
                    const std::vector<Odom_Data_t> &odom, 
                    const std::vector<Imu_Data_t> &imu, 
                    gtsam::Vector4 &input);
      
private:
  Parameter_t param_;
  uint64_t state_idx_ = 0;
  
  std::vector<Odom_Data_t> odom_data_v_;
  std::vector<Odom_Data_t> odom_data_noise_;
  std::vector<Imu_Data_t>  imu_data_v_;

  void buildFactorGraph(gtsam::NonlinearFactorGraph& _graph, gtsam::Values& _initial_value, 
                        const std::vector<Desired_State_t> &des_v, const Odom_Data_t &odom, double dt);

  void buildFactorGraph(gtsam::NonlinearFactorGraph& _graph, gtsam::Values& _initial_value, 
                        const std::vector<Desired_State_t> &des_v, const std::vector<Odom_Data_t> &odom_v, 
                        const std::vector<Imu_Data_t> &imu_v, double dt);

  std::vector<Desired_State_t> des_data_v_;
  gtsam::NonlinearFactorGraph  graph_positioning_;
  gtsam::Values                initial_value_positioning_;

  gtsam::NonlinearFactorGraph  graph_;
  gtsam::Values                initial_value_;

  double                       dt_;
  uint16_t                     opt_traj_lens_;
  uint16_t                     window_lens_;

};

#endif // __JPCM_H__

