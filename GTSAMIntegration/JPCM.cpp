#include "factors.h"
#include "JPCM.h"
#include "Marginalization.h"


using namespace gtsam;
using namespace std;
using namespace dmvio;
using namespace UAVFactor;


using symbol_shorthand::B;
using symbol_shorthand::S;
using symbol_shorthand::U;
using symbol_shorthand::V;
using symbol_shorthand::X;


buildJPCMFG::buildJPCMFG(Parameter_t &param) : param_(param)
{
    dt_            = 0.01f; 
    opt_traj_lens_ = param_.factor_graph.OPT_LENS_TRAJ;
    window_lens_   = param_.factor_graph.WINDOW_SIZE;
}


/* Fusion*/
void buildJPCMFG::buildFusionFG(gtsam_fg&  _graph, 
                               gtsam_sols& _initial_value, 
                               const std::vector<Odom_Data_t>& odom_v, 
                               const std::vector<Imu_Data_t>&  imu_v, 
                               double dt,
                               uint64_t& state_idx)
{
  // IMU noise
  auto imu_factor_noise  = noiseModel::Diagonal::Sigmas
    ((Vector(9) << Vector3::Constant(param_.factor_graph.acc_sigma_x* dt * dt * 0.5f + param_.factor_graph.acc_sigma_x* dt * dt), 
      Vector3::Constant(param_.factor_graph.gyro_sigma_x* dt), Vector3::Constant(param_.factor_graph.acc_sigma_x* dt)).finished());  
  
  // Bias noise
  auto bias_noise        = noiseModel::Diagonal::Sigmas(
    (Vector(6) << Vector3(param_.factor_graph.acc_bias_imu_x, param_.factor_graph.acc_bias_imu_x, param_.factor_graph.acc_bias_imu_x), 
    Vector3(param_.factor_graph.gyro_bias_sigma_x, param_.factor_graph.gyro_bias_sigma_x, param_.factor_graph.gyro_bias_sigma_x)).finished());
  
  // Prior noise
  auto prior_bias_noise  = noiseModel::Diagonal::Sigmas(
    (Vector(6) << Vector3::Constant(param_.factor_graph.prior_acc_sigma), Vector3::Constant(param_.factor_graph.prior_gyro_sigma)).finished());
  auto prior_vicon_noise = noiseModel::Diagonal::Sigmas(
    (Vector(6) << Vector3::Constant(param_.factor_graph.PRIOR_ROT_MEAS_COV), Vector3::Constant(param_.factor_graph.PRIOR_POS_MEAS_COV)).finished());
  auto prior_vel_noise   = noiseModel::Diagonal::Sigmas(
    Vector3(param_.factor_graph.PRIOR_VEL_MEAS_COV, param_.factor_graph.PRIOR_VEL_MEAS_COV, param_.factor_graph.PRIOR_VEL_MEAS_COV));

  // GPS noise
  auto noise_model_gps   = noiseModel::Isotropic::Sigma(3, param_.factor_graph.POS_MEAS_COV);
  auto vel_noise         = noiseModel::Isotropic::Sigma(3, param_.factor_graph.VEL_MEAS_COV);
  // gtsam::GPSFactor gps_factor(X(correction_count), Point3(gps(0), gps(1), gps(2)), noise_model_gps);
  // gtsam_imuBi prior_bias(init_bias_);
  gtsam_imuBi prior_bias(gtsam::Vector3(0.12, -0.07, 0), gtsam::Vector3(0, 0, -0.004)); //!!!
  if(state_idx == 0) 
  {
    for(uint16_t idx =  state_idx; idx < window_lens_ + state_idx; idx++)
    {
      gtsam::Rot3 rot  = gtsam::Rot3(odom_v[idx - state_idx].q);
      gtsam::Pose3 pose(rot, odom_v[idx - state_idx].p);
      gtsam::Vector3 v = odom_v[idx - state_idx].v;

      if(idx != state_idx)
      {
        // float __dt = (odom_v[idx - state_idx].rcv_stamp - odom_v[idx - state_idx - 1].rcv_stamp).toSec();
        graph_positioning_.add(IMUFactor(X(idx-1+IDX_P_START), V(idx-1+IDX_P_START), B(idx-1+IDX_P_START), X(idx+IDX_P_START), V(idx+IDX_P_START), dt, 
          imu_v[idx - state_idx].a, imu_v[idx - state_idx].w, imu_factor_noise));
        _initial_value.insert(B(idx-1+IDX_P_START), prior_bias);
        gtsam_imuBi zero_bias(gtsam::Vector3(0,0,0), gtsam::Vector3(0,0,0));
        graph_positioning_.add(BetweenFactor<gtsam_imuBi>(B(idx-1+IDX_P_START), B(idx+IDX_P_START), zero_bias, bias_noise));
      }
      
      if(idx == state_idx)
      {
        graph_positioning_.add(gtsam::PriorFactor<gtsam_imuBi>   (B(idx+IDX_P_START), prior_bias, prior_bias_noise));
        graph_positioning_.add(gtsam::PriorFactor<gtsam::Pose3>  (X(idx+IDX_P_START), pose,       prior_vicon_noise)); 
        graph_positioning_.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx+IDX_P_START), v,          prior_vel_noise)); 
      }
      else
      {
        // graph_positioning_.add(gtsam::GPSFactor(X(idx+IDX_P_START), odom_v[idx - state_idx].p, noise_model_gps)); 
        graph_positioning_.add(gtsam::PriorFactor<gtsam::Pose3>  (X(idx+IDX_P_START), pose, prior_vicon_noise)); 
        // graph_positioning_.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx+IDX_P_START), v, prior_vel_noise)); 
      }

      _initial_value.insert(X(idx+IDX_P_START), pose);
      _initial_value.insert(V(idx+IDX_P_START), v);

    }

    _initial_value.insert(B(window_lens_-1+IDX_P_START), gtsam_imuBi());
    _graph = graph_positioning_;

    std::cout << "Build first factor graph" << std::endl;
    _graph.print();

  }
  else
  {
    gtsam::FastVector<gtsam::Key> keysToMarginalize;
    keysToMarginalize.push_back(X(state_idx-1+IDX_P_START));
    keysToMarginalize.push_back(V(state_idx-1+IDX_P_START));
    keysToMarginalize.push_back(B(state_idx-1+IDX_P_START));
    // keysToMarginalize.push_back(U(state_idx));
    boost::shared_ptr<gtsam_fg> margGraph;
    margGraph = marginalizeOut(graph_positioning_, _initial_value, keysToMarginalize, nullptr, true);

    graph_positioning_ = *margGraph;
    // add new measurements factor
    uint16_t    idx =  window_lens_ + state_idx - 1 + IDX_P_START;
    gtsam::Rot3 rot = gtsam::Rot3(odom_v[window_lens_-1].q);
    gtsam::Pose3 pose(rot, odom_v[window_lens_-1].p);

    // graph_positioning_.add(gtsam::PriorFactor<gtsam::Pose3>  (X(idx), pose, vicon_noise));
    graph_positioning_.add(gtsam::GPSFactor(X(idx), odom_v[window_lens_-1].p, noise_model_gps)); 
    float __dt = (odom_v[window_lens_-1].rcv_stamp - odom_v[window_lens_-2].rcv_stamp).toSec();
    std::cout << "__dt is : " << __dt << std::endl;
    graph_positioning_.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), odom_v[window_lens_-1].v, vel_noise)); 
    graph_positioning_.add(IMUFactor(X(idx-1), V(idx-1), B(idx-1), X(idx), V(idx), __dt, imu_v[window_lens_-1].a, imu_v[window_lens_-1].w, imu_factor_noise));
    
    gtsam_imuBi zero_bias(gtsam::Vector3(0,0,0), gtsam::Vector3(0,0,0));
    graph_positioning_.add(BetweenFactor<gtsam_imuBi>(B(idx-1), B(idx), zero_bias, bias_noise));
    _initial_value.insert(B(idx), gtsam_imuBi());

    _initial_value.insert(X(idx), pose);
    _initial_value.insert(V(idx), odom_v[window_lens_-1].v);

    _graph = graph_positioning_;
  
    std::cout << "Updated graph" << std::endl;
    _graph.print();

  }
  state_idx++;
}

// Build Factor Graph

/* 
 * JCPM 
 */
void buildJPCMFG::buildFactorGraph(gtsam_fg& _graph, gtsam_sols& _initial_value, 
                        const std::vector<Desired_State_t> &des_seq, const std::vector<Odom_Data_t> &odom_v, 
                        const std::vector<Imu_Data_t> &imu_v, double dt, uint64_t& state_idx)
{
  buildFusionFG(_graph, _initial_value, odom_v, imu_v, dt, state_idx);
  buildJoinedFG(_graph, _initial_value, des_seq, dt, state_idx);
}

void buildJPCMFG::buildJoinedFG(gtsam_fg& _graph, gtsam_sols& _initial_value, 
                        const std::vector<Desired_State_t> &des_seq, double dt, uint64_t& state_idx)
{
  auto input_jerk  = noiseModel::Diagonal::Sigmas(Vector4(param_.factor_graph.INPUT_JERK_T, 
      param_.factor_graph.INPUT_JERK_M, param_.factor_graph.INPUT_JERK_M, param_.factor_graph.INPUT_JERK_M3));

  auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(9) << Vector3::Constant(param_.factor_graph.DYNAMIC_P_COV), 
      Vector3::Constant(param_.factor_graph.DYNAMIC_R_COV), Vector3::Constant(param_.factor_graph.DYNAMIC_V_COV)).finished());
  
  auto ref_predict_vel_noise = noiseModel::Diagonal::Sigmas(
    Vector3(param_.factor_graph.CONTROL_V_COV, param_.factor_graph.CONTROL_V_COV, param_.factor_graph.CONTROL_V_COV));
  
  // Configure noise models
  // Replace pose noise of gtsam::Pose3(gtsam::Rot3(), gps_position);

  auto noise_model_gps = noiseModel::Isotropic::Sigma(3, param_.factor_graph.POS_MEAS_COV);
  // gtsam::GPSFactor gps_factor(X(correction_count), Point3(gps(0), gps(1), gps(2)), noise_model_gps);

  auto clf_sigma = noiseModel::Diagonal::Sigmas(Vector4(1.0, 1.0, 1.0, 1.0));

  gtsam::Vector3 drag_k(-param_.rt_drag.x, -param_.rt_drag.y, -param_.rt_drag.z);
  
  _graph = graph_positioning_;

  uint16_t begin_u = 0;
  uint16_t end_u   = opt_traj_lens_;

  uint16_t latest_state_idx = state_idx + window_lens_ + IDX_P_START - 1;
  
  for (uint16_t idx = begin_u; idx < end_u; idx++)
  {
    if(idx == begin_u)
    {
      DynamicsFactorTGyro dynamics_factor(X(latest_state_idx), V(latest_state_idx), U(idx), X(idx + 1), V(idx + 1), dt, param_.mass, drag_k, dynamics_noise);
      _graph.add(dynamics_factor); 
    }
    else
    {
      DynamicsFactorTGyro dynamics_factor(X(idx), V(idx), U(idx), X(idx + 1), V(idx + 1), dt, param_.mass, drag_k, dynamics_noise);
      _graph.add(dynamics_factor); 
    }
    
    gtsam::Pose3   pose_idx(gtsam::Rot3(des_seq[idx].q), des_seq[idx].p);
    gtsam::Vector3 vel_idx   = des_seq[idx].v;
    // gtsam::Vector3 omega_idx = des_seq[idx].w;
    
    // std::cout << "Idx: " << idx << ", ref vel: " << vel_idx.transpose() << std::endl;
    // std::cout << "ref pos: " << des_seq[idx].p.transpose() << std::endl;

    _initial_value.update(X(idx + 1), pose_idx);
    _initial_value.update(V(idx + 1), vel_idx);

    if(idx != begin_u)
    {
      BetForceMoments bet_FM_factor(U(idx - 1), U(idx), input_jerk);
      _graph.add(bet_FM_factor); 
    }
    
    gtsam::Vector4 init_input(10, 0, 0, 0);
    _initial_value.update(U(idx), init_input);

    gtsam::Vector3 control_r_cov(param_.factor_graph.CONTROL_R1_COV, param_.factor_graph.CONTROL_R2_COV, param_.factor_graph.CONTROL_R3_COV);
    if(idx == end_u - 1)
    {   
      gtsam::Vector3 final_position_ref(param_.factor_graph.CONTROL_PF_COV_X, param_.factor_graph.CONTROL_PF_COV_Y, param_.factor_graph.CONTROL_PF_COV_Z);
      auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << control_r_cov, final_position_ref).finished()); 
      _graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise)); 
      _graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise)); 
    }
    else
    {
      gtsam::Vector3 _position_ref(param_.factor_graph.CONTROL_P_COV_X, param_.factor_graph.CONTROL_P_COV_Y, param_.factor_graph.CONTROL_P_COV_Z);
      auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << control_r_cov, _position_ref).finished());
      _graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise)); 
      _graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise)); 
    }

    ControlLimitTGyroFactor cntrolLimitTGyroFactor(U(0), clf_sigma, param_.factor_graph.low, param_.factor_graph.high,
    param_.factor_graph.glow, param_.factor_graph.ghigh, param_.factor_graph.thr, param_.factor_graph.gthr, param_.factor_graph.alpha);
    _graph.add(cntrolLimitTGyroFactor); 

  }

}

/* 
 * Single-positioning JPCM 
 */
void buildJPCMFG::buildFactorGraph(gtsam_fg& _graph, gtsam_sols& _initial_value, 
  const std::vector<Desired_State_t> &des_seq, const Odom_Data_t &odom, double dt)
{
  auto input_jerk  = noiseModel::Diagonal::Sigmas(Vector4(param_.factor_graph.INPUT_JERK_T, 
      param_.factor_graph.INPUT_JERK_M, param_.factor_graph.INPUT_JERK_M, param_.factor_graph.INPUT_JERK_M3));

  auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(9) << Vector3::Constant(param_.factor_graph.DYNAMIC_P_COV), 
      Vector3::Constant(param_.factor_graph.DYNAMIC_R_COV), Vector3::Constant(param_.factor_graph.DYNAMIC_V_COV)).finished());
  
  // Initial state noise
  auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(param_.factor_graph.ROT_MEAS_COV), Vector3::Constant(param_.factor_graph.PRI_VICON_POS_COV)).finished());
  auto vel_noise   = noiseModel::Diagonal::Sigmas(Vector3(param_.factor_graph.PRI_VICON_VEL_COV, param_.factor_graph.PRI_VICON_VEL_COV, param_.factor_graph.PRI_VICON_VEL_COV));

  auto ref_predict_vel_noise = noiseModel::Diagonal::Sigmas(Vector3(param_.factor_graph.CONTROL_V_COV, param_.factor_graph.CONTROL_V_COV, param_.factor_graph.CONTROL_V_COV));
  // auto ref_predict_omega_noise = noiseModel::Diagonal::Sigmas(Vector3(param_.factor_graph.CONTROL_O_COV, param_.factor_graph.CONTROL_O_COV, param_.factor_graph.CONTROL_O_COV));
  gtsam_fg  graph;
  gtsam_sols initial_value;

  graph.empty();
  initial_value.empty();

  auto clf_sigma = noiseModel::Diagonal::Sigmas(Vector4(1.0, 1.0, 1.0, 1.0));
  ControlLimitTGyroFactor cntrolLimitTGyroFactor(U(0), clf_sigma, param_.factor_graph.low, param_.factor_graph.high,
      param_.factor_graph.glow, param_.factor_graph.ghigh, param_.factor_graph.thr, param_.factor_graph.gthr, param_.factor_graph.alpha);
  graph.add(cntrolLimitTGyroFactor);

  gtsam::Vector3 drag_k(-param_.rt_drag.x, -param_.rt_drag.y, -param_.rt_drag.z);

  for (uint16_t idx = 0; idx < param_.factor_graph.OPT_LENS_TRAJ; idx++)
  {
    DynamicsFactorTGyro dynamics_factor(X(idx), V(idx), U(idx), X(idx + 1), V(idx + 1), dt, param_.mass, drag_k, dynamics_noise);
    graph.add(dynamics_factor);
    
    gtsam::Pose3   pose_idx(gtsam::Rot3(des_seq[idx].q), des_seq[idx].p);
    gtsam::Vector3 vel_idx   = des_seq[idx].v;
    // gtsam::Vector3 omega_idx = des_seq[idx].w;
    
    // std::cout << "Idx: " << idx << ", ref vel: " << vel_idx.transpose() << std::endl;
    // std::cout << "ref pos: " << des_seq[idx].p.transpose() << std::endl;

    initial_value.insert(X(idx + 1), pose_idx);
    initial_value.insert(V(idx + 1), vel_idx);

    if(idx != 0)
    {
      BetForceMoments bet_FM_factor(U(idx - 1), U(idx), input_jerk);
      graph.add(bet_FM_factor);
    }
    
    gtsam::Vector4 init_input(10, 0, 0, 0);
    initial_value.insert(U(idx), init_input);

    gtsam::Vector3 control_r_cov(param_.factor_graph.CONTROL_R1_COV, param_.factor_graph.CONTROL_R2_COV, param_.factor_graph.CONTROL_R3_COV);
    if(idx == param_.factor_graph.OPT_LENS_TRAJ - 1)
    {   
      gtsam::Vector3 final_position_ref(param_.factor_graph.CONTROL_PF_COV_X, param_.factor_graph.CONTROL_PF_COV_Y, param_.factor_graph.CONTROL_PF_COV_Z);
      auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << control_r_cov, final_position_ref).finished()); 
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
      graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
    }
    else
    {
      gtsam::Vector3 _position_ref(param_.factor_graph.CONTROL_P_COV_X, param_.factor_graph.CONTROL_P_COV_Y, param_.factor_graph.CONTROL_P_COV_Z);
      auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << control_r_cov, _position_ref).finished());
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
      graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
    }

    if (idx == 0)
    {              
      gtsam::Rot3 rot = gtsam::Rot3(odom.q);
      gtsam::Pose3 pose(rot, odom.p);

      // std::cout << "Odom: " << idx << ", vel: " << vel_add.transpose() << std::endl;
      // std::cout << "pos: " << pos_add.transpose() << std::endl;
      
      graph.add(gtsam::PriorFactor<gtsam::Pose3>  (X(idx), pose, vicon_noise));
      graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), odom.v, vel_noise));
      
      initial_value.insert(X(idx), pose);
      initial_value.insert(V(idx), odom.v);

      // float distance = (des_seq[0].p - odom.p).norm();
      // std::cout << "distance: [ " << distance << " ]" << endl;
    }
  }

  _graph = graph;
  _initial_value = initial_value;
}
