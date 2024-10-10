#ifndef __JPCM_H__
#define __JPCM_H__

#include "type.h"
#include "factors.h"
#include "GTSAMUtils.h"
#include "Marginalization.h"

class buildJPCMFG
{
public:

  buildJPCMFG(Parameter_t &param);
  
  // Single-point JPCM
  void buildFactorGraph(gtsam_fg& _graph, gtsam_sols& _initial_value, 
                        const std::vector<Desired_State_t> &des_seq, const Odom_Data_t &odom, double dt);
  // JPCM
  void buildFactorGraph(gtsam_fg& _graph, gtsam_sols& _initial_value, 
                        const std::vector<Desired_State_t> &des_seq, const std::vector<Odom_Data_t> &odom_v, 
                        const std::vector<Imu_Data_t> &imu_v, double dt, uint64_t& state_idx);
  
  // Fake-GPS and IMU Fusion
  void buildFusionFG(gtsam_fg& _graph, gtsam_sols& _initial_value, 
                     const std::vector<Odom_Data_t> &odom_v, 
                     const std::vector<Imu_Data_t> &imu_v, double dt, uint64_t& state_idx);
  // Add Control factors into factor graph
  void buildJoinedFG(gtsam_fg& _graph, gtsam_sols& _initial_value, 
                     const std::vector<Desired_State_t> &des_seq, double dt, uint64_t& state_idx);
  
  gtsam_fg      graph_positioning_;
  gtsam::Values initial_value_positioning_;

  double         dt_;
  uint16_t       opt_traj_lens_;
  uint16_t       window_lens_;
  const uint16_t IDX_P_START = 100;
  Parameter_t    param_;

  bool           first_add_crf_ = true; // first add control-related factors into factor graph

};

#endif // __JPCM_H__

