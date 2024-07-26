#ifndef __JPCM_H__
#define __JPCM_H__

#include "controller.h"
#include "factors.h"

class JCPM_TGyro: public DFBControl
{
public:

  JCPM_TGyro(Parameter_t &param): DFBControl(param), opt_lens_traj_(20)
  { 
    graph_.empty(); dt_ = 0.01f; 
    const Parameter_t params = get_param();
    opt_lens_traj_ = params.factor_graph.OPT_LENS_TRAJ;
  }

  /* mode_switch: DFBC  */
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu, 
    Controller_Output_t &thr_bodyrate_u, CTRL_MODE mode_switch);
      
private:
  void buildFactorGraph(gtsam::NonlinearFactorGraph& graph, gtsam::Values& initial_value, const std::vector<Desired_State_t> &des_v, const Odom_Data_t &odom, double dt);

  std::vector<Desired_State_t> des_vec_;
  gtsam::NonlinearFactorGraph  graph_;
  gtsam::Values                initial_value_;
  double                       dt_;
  uint16_t                     opt_lens_traj_;

};

#endif // __JPCM_H__

