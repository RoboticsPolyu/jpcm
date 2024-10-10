#include "controller.h"
#include "Marginalization.h"

using namespace gtsam;
using namespace std;
using namespace dmvio;

using namespace uavfactor;

using symbol_shorthand::S;
using symbol_shorthand::U;
using symbol_shorthand::V;
using symbol_shorthand::X;
using symbol_shorthand::B;
using symbol_shorthand::R;

static std::random_device __randomDevice;
static std::mt19937 __randomGen(__randomDevice());

bool Controller::initializeState(const std::vector<Imu_Data_t> &imu_raw, const std::vector<Odom_Data_t> &fakeGPS, 
                                 gtsam::Vector3 &init_vel, gtsam::Vector6 &init_bias)
{
  double opt_cost = 0.0f;
  clock_t start, end;

  gtsam_fg graph_init;
  gtsam_sols initial_value;

  // IMU noise
  auto imu_factor_noise = noiseModel::Diagonal::Sigmas
    ((Vector(9) << Vector3::Constant(param_.factor_graph.acc_sigma_x* dt_ * dt_ * 0.5f + param_.factor_graph.acc_sigma_x* dt_ * dt_), 
      Vector3::Constant(param_.factor_graph.gyro_sigma_x* dt_), Vector3::Constant(param_.factor_graph.acc_sigma_x*dt_)).finished());  
  
  // Bias noise
  auto bias_noise = noiseModel::Diagonal::Sigmas(
    (Vector(6) << Vector3(param_.factor_graph.acc_bias_imu_x, param_.factor_graph.acc_bias_imu_x, param_.factor_graph.acc_bias_imu_x), 
    Vector3(param_.factor_graph.gyro_bias_sigma_x, param_.factor_graph.gyro_bias_sigma_x, param_.factor_graph.gyro_bias_sigma_x)).finished());
  
  // Prior noise
  auto prior_bias_noise = noiseModel::Diagonal::Sigmas(
    (Vector(6) << Vector3::Constant(param_.factor_graph.prior_acc_sigma), Vector3::Constant(param_.factor_graph.prior_gyro_sigma)).finished());
  auto prior_vicon_noise = noiseModel::Diagonal::Sigmas(
    (Vector(6) << Vector3::Constant(param_.factor_graph.PRIOR_ROT_MEAS_COV), Vector3::Constant(param_.factor_graph.PRIOR_POS_MEAS_COV)).finished());
  auto prior_vel_noise   = noiseModel::Diagonal::Sigmas(
    Vector3(param_.factor_graph.PRIOR_VEL_MEAS_COV, param_.factor_graph.PRIOR_VEL_MEAS_COV, param_.factor_graph.PRIOR_VEL_MEAS_COV));

  // GPS noise
  auto noise_model_gps = noiseModel::Isotropic::Sigma(3, param_.factor_graph.POS_MEAS_COV);
  // gtsam::GPSFactor gps_factor(X(correction_count), Point3(gps(0), gps(1), gps(2)), noise_model_gps);

  for(uint16_t idx = 0; idx < window_lens_; idx++)
  {
    gtsam::Pose3 pose  = gtsam::Pose3(gtsam::Rot3(fakeGPS[idx].q), fakeGPS[idx].p);
    gtsam::Vector3 vel = fakeGPS[idx].v;

    if(idx != 0)
    {
      float __dt = (fakeGPS[idx].rcv_stamp - fakeGPS[idx - 1].rcv_stamp).toSec();
      graph_init.add(IMUFactor(X(idx-1), V(idx-1), B(idx-1), X(idx), V(idx), __dt, imu_raw[idx].a, imu_raw[idx].w, imu_factor_noise));
      gtsam_imuBi zero_bias(gtsam::Vector3(0,0,0), gtsam::Vector3(0,0,0));
      graph_init.add(BetweenFactor<gtsam_imuBi>(B(idx-1), B(idx), zero_bias, bias_noise));
    }

    // graph_init.add(gtsam::GPSFactor(X(idx), odom_v[idx].p, noise_model_gps)); 
    graph_init.add(gtsam::PriorFactor<gtsam::Pose3>  (X(idx), pose, prior_vicon_noise)); 
    // graph_init.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), vel,  prior_vel_noise)); 

    initial_value.insert(B(idx), gtsam_imuBi());
    initial_value.insert(X(idx), pose);
    initial_value.insert(V(idx), vel);
  }

  gtsam::LevenbergMarquardtParams parameters;
  parameters.absoluteErrorTol = 100;
  parameters.relativeErrorTol = 1e-2;
  parameters.maxIterations    = 10;
  parameters.verbosity        = gtsam::NonlinearOptimizerParams::SILENT;
  parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SILENT;

  std::cout << " -- <  Initialization Test > -- " << std::endl;
  LevenbergMarquardtOptimizer optimizer(graph_init, initial_value, parameters);
  start = clock();
  Values result = optimizer.optimize();
  end   = clock();
  std::cout << " ---------------------------------------------------- Result ----------------------------------------------------" << std::endl;
  // result.print();
  opt_cost = (double)(end - start) / CLOCKS_PER_SEC;
  std::cout << " ---------- Initialization Time: [ " << opt_cost << " ] " << endl;

  gtsam::Vector3 vel;
  gtsam_imuBi imu_bias;
  imu_bias  = result.at<gtsam_imuBi>(B(0));
  init_vel  = result.at<Vector3>(V(0));
  init_bias = imu_bias.vector();

  std::cout << "Initialization Vel:  [ " << init_vel.transpose()  << " ] " << endl;
  std::cout << "Initialization Bias: [ " << init_bias.transpose() << " ] " << endl;

  return true;
}

/* Fusion */
quadrotor_msgs::Px4ctrlDebug Controller::fusion(const Odom_Data_t &odom, const Imu_Data_t &imu_raw, const Odom_Data_t &GT, gtsam::Pose3& fus_pose, gtsam::Vector3& fus_vel, gtsam::Vector3& fus_w)
{
  odom_data_v_.push_back(GT);
  gtsam::Vector3 gt_rxyz = gtsam::Rot3(GT.q).rpy();
  
  odom_data_noise_.push_back(odom);
  imu_data_v_.push_back(imu_raw);

  double opt_cost = 0.0f;

  clock_t start, end;
  gtsam::Vector3 init_vel;
  gtsam::Vector6 init_bias;

  if(odom_data_v_.size() == window_lens_)
  {
    if(!init_state_flag_)
    {
      initializeState(imu_data_v_, odom_data_v_, init_vel_, init_bias_);
      init_state_flag_ = true;
    }

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 100;
    parameters.relativeErrorTol = 1e-2;
    parameters.maxIterations    = 10;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::SILENT;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SILENT;

    FGbuilder->buildFusionFG(graph_, initial_value_, odom_data_noise_, imu_data_v_, dt_, state_idx_);
    LevenbergMarquardtOptimizer optimizer(graph_, initial_value_, parameters);
    start = clock();
    Values result = optimizer.optimize();
    end   = clock();
    initial_value_ = result; 
    std::cout << " ---------------------------------------------------- Result ----------------------------------------------------" << std::endl;
    result.print();
    opt_cost = (double)(end - start) / CLOCKS_PER_SEC;
    std::cout << " ---------- Optimize Time: [ " << opt_cost << " ] " << endl;

    uint16_t idx = 0;
    if(state_idx_ == 1)
    {
      idx = window_lens_ - 1 + IDX_P_START;
    }
    else
    {
      idx = state_idx_ - 1 + IDX_P_START;
    }
    
    gtsam::Pose3 pose;
    gtsam::Vector3 vel;
    gtsam_imuBi imu_bias;
    gtsam::Rot3 Rg;

    pose = result.at<Pose3>(X(idx));
    vel  = result.at<Vector3>(V(idx));
    imu_bias = result.at<gtsam_imuBi>(B(idx));

    fus_pose = pose;
    fus_vel  = vel;
    fus_w    = imu_bias.correctGyroscope(imu_data_v_[window_lens_-1].w); // Guassian noise

    if(param_.factor_graph.opt_gravity_rot)
    {
      Rg = result.at<gtsam::Rot3>(R(0));
      std::cout << " --- Gravity rotation:" << Rg.rpy().transpose() << std::endl;
    }
    gtsam::Vector3 fusion_rxyz = pose.rotation().rpy();

    log_ << std::setprecision(19)
        // GT
        << GT.p.x() << " " << GT.p.y() << " " << GT.p.z() << " "
        << gt_rxyz.x() << " " << gt_rxyz.y() << " " << gt_rxyz.z() << " "
        << GT.v.x() << " " << GT.v.y() << " " << GT.v.z() << " "

        // Measurement
        << odom.p.x() << " " << odom.p.y() << " " << odom.p.z() << " " 
        << odom.v.x() << " " << odom.v.y() << " " << odom.v.z() << " "

        // Estimation
        << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z() << " "
        << vel.x() << " " << vel.y() << " " << vel.z() << " "
        << fusion_rxyz.x() << " " << fusion_rxyz.y() << " " << fusion_rxyz.z() << " "
        // Time Cost
        << opt_cost  << " "
        
        // IMU Raw Data
        << imu_raw.w.x() << " " << imu_raw.w.y() << " " << imu_raw.w.z() << " "
        << imu_raw.a.x() << " " << imu_raw.a.y() << " " << imu_raw.a.z() << " "
        
        // IMU Bias
        << imu_bias.accelerometer().x() << " " << imu_bias.accelerometer().y() << " " << imu_bias.accelerometer().z() << " "
        << imu_bias.gyroscope().x() << " " << imu_bias.gyroscope().y() << " " << imu_bias.gyroscope().z() << " "
        
        << std::endl;
  }

  if(odom_data_v_.size() >= window_lens_)
  {
    odom_data_v_.erase(odom_data_v_.begin());
  }

  if(odom_data_noise_.size() >= window_lens_)
  {
    odom_data_noise_.erase(odom_data_noise_.begin());
  }

  if(imu_data_v_.size() >= window_lens_)
  {
    imu_data_v_.erase(imu_data_v_.begin());
  }

  return debug_msg_;

}

/* JPCM */
quadrotor_msgs::Px4ctrlDebug Controller::calculateControl(const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu, 
  const Imu_Data_t &imu_raw, 
  Controller_Output_t &thr_bodyrate_u, CTRL_MODE mode_switch)
{
  odom_data_v_.push_back(odom);
  Odom_Data_t odom_noise = add_Guassian_noise(odom);

  odom_data_noise_.push_back(odom_noise);
  des_data_v_.push_back(des);
  imu_data_v_.push_back(imu_raw);

  bool   timeout  = false;
  double opt_cost = 0.0f;

  double thrust2   = 0;
  gtsam::Vector3 bodyrates2(0,0,0);

  if(timeout || mode_switch == DFBC || des_data_v_.size() < opt_traj_lens_)
  {
    Controller::calculateControl(des_data_v_[0], odom_noise, imu, thr_bodyrate_u);
  }
  else if(mode_switch == JPCM && des_data_v_.size() == opt_traj_lens_ ) // && odom.p.z() > hight_thr)
  {
    clock_t start, end;

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 100;
    parameters.relativeErrorTol = 1e-2;
    parameters.maxIterations    = 10;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::SILENT;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SILENT;

    std::cout << " - JPCM Opt - " << std::endl;
    FGbuilder->buildFactorGraph(graph_, initial_value_, des_data_v_, odom_data_noise_, imu_data_v_, dt_, state_idx_);
    LevenbergMarquardtOptimizer optimizer(graph_, initial_value_, parameters);
    start = clock();
    Values result = optimizer.optimize();
    end   = clock();
    initial_value_ = result; 
    std::cout << " ---------------------------------------------------- Result ----------------------------------------------------" << std::endl;
    // result.print();
    opt_cost = (double)(end - start) / CLOCKS_PER_SEC;
    float distance = (des_data_v_[0].p - odom_noise.p).norm();
    
    gtsam::Vector4 input;
    input = result.at<gtsam::Vector4>(U(0));
    Eigen::Vector3d des_acc(0, 0, input[0]);
    thrust2    = Controller::computeDesiredCollectiveThrustSignal(des_acc);
    bodyrates2 = Eigen::Vector3d(input[1], input[2], input[3]);

    thr_bodyrate_u.thrust    = thrust2;
    thr_bodyrate_u.bodyrates = bodyrates2;

    uint16_t idx = 0;
    if(state_idx_ == 1)
    {
      idx = window_lens_ - 1 + IDX_P_START;
    }
    else
    {
      idx = state_idx_ - 1 + IDX_P_START;
    }
    
    gtsam::Pose3   pose;
    gtsam::Vector3 vel;
    gtsam_imuBi imu_bias;

    pose     = result.at<Pose3>      (X(idx));
    vel      = result.at<Vector3>    (V(idx));
    imu_bias = result.at<gtsam_imuBi>(B(idx));

    gtsam::Vector3 eular_xyz     = pose.rotation().rpy();
    gtsam::Vector3 gt_eular_xyz  = gtsam::Rot3(odom.q).rpy();
    gtsam::Vector3 des_eular_xyz = gtsam::Rot3(des_data_v_[0].q).rpy();
    float          distance_est  = (des_data_v_[0].p - pose.translation()).norm();

    std::cout << " ---------- Optimize Time: [ " << opt_cost << " ], " << "ori distance: [ " << distance << " ], est distance: [" << distance_est << " ]" << endl;

    log_ << std::setprecision(19) 
      // Des info 
      << des_data_v_[0].rcv_stamp.toSec() <<  " "
      << des_data_v_[0].p.x() << " " << des_data_v_[0].p.y() << " " << des_data_v_[0].p.z() << " "
      << des_data_v_[0].v.x() << " " << des_data_v_[0].v.y() << " " << des_data_v_[0].v.z() << " "
      << des_eular_xyz.x()    << " " << des_eular_xyz.y()    << " " << des_eular_xyz.z()    << " "
      
      // Positioning GT Info
      << odom.p.x()       << " " << odom.p.y()       << " " << odom.p.z()       << " "
      << gt_eular_xyz.x() << " " << gt_eular_xyz.y() << " " << gt_eular_xyz.z() << " "
      << odom.v.x()       << " " << odom.v.y()       << " " << odom.v.z()       << " "
      
      // Positioning with Noise
      << odom_noise.p.x() << " " << odom_noise.p.y() << " " << odom_noise.p.z() << " " 
      << odom_noise.v.x() << " " << odom_noise.v.y() << " " << odom_noise.v.z() << " "
      
      // Positioning Estimation
      << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z() << " "
      << vel.x()                << " " << vel.y()                << " " << vel.z()                << " "
      << eular_xyz.x()          << " " << eular_xyz.y()          << " " << eular_xyz.z()          << " "
      
      // Time cost
      << opt_cost << " "

      // IMU Raw Data
      << imu_raw.w.x() << " " << imu_raw.w.y() << " " << imu_raw.w.z() << " "
      << imu_raw.a.x() << " " << imu_raw.a.y() << " " << imu_raw.a.z() << " "

      // Bias
      << imu_bias.accelerometer().x() << " " << imu_bias.accelerometer().y() << " " << imu_bias.accelerometer().z() << " "
      << imu_bias.gyroscope().x()     << " " << imu_bias.gyroscope().y()     << " " << imu_bias.gyroscope().z()     << " "
      
      // Control
      << thrust2 << " " 
      << bodyrates2.x() << " " << bodyrates2.y() << " " << bodyrates2.z() << " " // MPC

      << std::endl;
  }

  if(des_data_v_.size() >= opt_traj_lens_)
  {
    des_data_v_.erase(des_data_v_.begin());
  }

  if(odom_data_v_.size() >= window_lens_)
  {
    odom_data_v_.erase(odom_data_v_.begin());
  }

  if(odom_data_noise_.size() >= window_lens_)
  {
    odom_data_noise_.erase(odom_data_noise_.begin());
  }

  if(imu_data_v_.size() >= window_lens_)
  {
    imu_data_v_.erase(imu_data_v_.begin());
  }

  return debug_msg_;

}

/* 
 * Single-point (SP) JPCM 
 */
quadrotor_msgs::Px4ctrlDebug Controller::calculateControl(const Desired_State_t &des, const Odom_Data_t &GT, const Odom_Data_t &odom, const Imu_Data_t &imu, 
  Controller_Output_t &thr_bodyrate_u, CTRL_MODE mode_switch)
{
  // Odom_Data_t odom_noise = add_Guassian_noise(odom);
  Odom_Data_t odom_noise = odom;
  
  gtsam::Vector3 gt_rxyz = gtsam::Rot3(GT.q).rpy();

  bool   timeout  = false;
  double opt_cost = 0.0f;
  double thrust   = 0.0f;
  double thrust2  = 0.0f;
  gtsam::Vector3 bodyrates(0,0,0);
  gtsam::Vector3 bodyrates2(0,0,0);

  des_data_v_.push_back(des);

  if(timeout || mode_switch == DFBC || des_data_v_.size() < opt_traj_lens_)
  {
    Controller::calculateControl(des_data_v_[0], odom_noise, imu, thr_bodyrate_u);
    thrust    = thr_bodyrate_u.thrust;
    bodyrates = thr_bodyrate_u.bodyrates;
  }
  else if(mode_switch == MPC && des_data_v_.size() == opt_traj_lens_ ) 
  {
    clock_t start, end;

    gtsam::LevenbergMarquardtParams parameters;
    parameters.absoluteErrorTol = 100;
    parameters.relativeErrorTol = 1e-2;
    parameters.maxIterations    = 10;
    parameters.verbosity        = gtsam::NonlinearOptimizerParams::SILENT;
    parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SILENT;
    graph_.empty();

    // std::cout << " --------------------- MPC -------------------- " << std::endl;
    FGbuilder->buildFactorGraph(graph_, initial_value_, des_data_v_, odom_noise, dt_);
    LevenbergMarquardtOptimizer optimizer(graph_, initial_value_, parameters);
    start = clock();
    Values result = optimizer.optimize();
    end = clock();
    opt_cost = (double)(end-start)/CLOCKS_PER_SEC;
    float distance = (des_data_v_[0].p - odom_noise.p).norm();
    std::cout << " ---------- Optimize Time: [ " << opt_cost << " ], " << "distance: [ " << distance << " ]" << endl;
    
    gtsam::Vector4 input;
    gtsam::Pose3   pose;
    gtsam::Vector3 vel;
    gtsam_imuBi imu_bias;

    input = result.at<gtsam::Vector4>(U(0));
    Eigen::Vector3d des_acc(0, 0, input[0]);

    pose = result.at<Pose3>(X(0));
    vel = result.at<Vector3>(V(0));

    gtsam::Vector3 fusion_rxyz = pose.rotation().rpy();

    thrust2    = Controller::computeDesiredCollectiveThrustSignal(des_acc);
    bodyrates2 = Eigen::Vector3d(input[1], input[2], input[3]);

    thr_bodyrate_u.thrust    = thrust2;
    thr_bodyrate_u.bodyrates = bodyrates2;

    gtsam::Vector3 des_eular_xyz = gtsam::Rot3(des_data_v_[0].q).rpy();

    log_ << std::setprecision(19)
      // Des info
      << des_data_v_[0].rcv_stamp.toSec() <<  " " 
      << des_data_v_[0].p.x() << " " << des_data_v_[0].p.y() << " " << des_data_v_[0].p.z() << " "
      << des_data_v_[0].v.x() << " " << des_data_v_[0].v.y() << " " << des_data_v_[0].v.z() << " "
      << des_eular_xyz.x()    << " " << des_eular_xyz.y()    << " " << des_eular_xyz.z()    << " "

      // Positioning GT Info
      << GT.p.x()    << " " << GT.p.y()    << " " << GT.p.z()    << " "
      << gt_rxyz.x() << " " << gt_rxyz.y() << " " << gt_rxyz.z() << " "
      << GT.v.x()    << " " << GT.v.y()    << " " << GT.v.z()    << " "

      // Positioning with Noise
      << odom_noise.p.x() << " " << odom_noise.p.y() << " " << odom_noise.p.z() << " " 
      << odom_noise.v.x() << " " << odom_noise.v.y() << " " << odom_noise.v.z() << " "

      // Positioning Estimation
      << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z() << " "
      << vel.x()                << " " << vel.y()                << " " << vel.z()                << " "
      << fusion_rxyz.x()        << " " << fusion_rxyz.y()        << " " << fusion_rxyz.z()        << " "
      
      // Time cost
      << opt_cost << " "
      
      // IMU_raw
      << 0 << " " << 0 << " " << 0 << " "
      << 0 << " " << 0 << " " << 0 << " "

      // Bias
      << imu_bias.accelerometer().x() << " " << imu_bias.accelerometer().y() << " " << imu_bias.accelerometer().z() << " "
      << imu_bias.gyroscope().x()     << " " << imu_bias.gyroscope().y()     << " " << imu_bias.gyroscope().z()     << " "
      
      // Control
      << thrust2 << " " 
      << bodyrates2.x() << " " << bodyrates2.y() << " " << bodyrates2.z() << " " // MPC

      << std::endl;
  }

  if(des_data_v_.size() >= opt_traj_lens_)
  {
    des_data_v_.erase(des_data_v_.begin());
  }
  return debug_msg_;
}

double Controller::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

Controller::Controller(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
  time_t now = time(NULL);
  tm* t = localtime(&now);
  FGbuilder = std::make_shared<buildJPCMFG>(param);

  graph_.empty(); 
  dt_ = 0.01f; 
  opt_traj_lens_ = param_.factor_graph.OPT_LENS_TRAJ;
  window_lens_   = param_.factor_graph.WINDOW_SIZE;

  stringstream ss; ss << "/home/amov/output/controller_log_";
  ss << t->tm_year + 1900 << "-" << t->tm_mon + 1 << "-" << t->tm_mday << "-" << t->tm_hour << "-" << t->tm_min << "-" << t->tm_sec << ".txt";
  std::cout << " -- log file:" << ss.str() << std::endl;
  log_.open(ss.str(), std::ios::out);

  position_noise_x = Dist_Dou(param_.factor_graph.POS_MEAS_MEAN, param_.factor_graph.POS_MEAS_COV);
  rotation_noise_x = Dist_Dou(0, param_.factor_graph.ROT_MEAS_COV);
  velocity_noise_x = Dist_Dou(0, param_.factor_graph.VEL_MEAS_COV);

  position_noise_y = Dist_Dou(param_.factor_graph.POS_MEAS_MEAN, param_.factor_graph.POS_MEAS_COV);
  rotation_noise_y = Dist_Dou(0, param_.factor_graph.ROT_MEAS_COV);
  velocity_noise_y = Dist_Dou(0, param_.factor_graph.VEL_MEAS_COV);

  position_noise_z = Dist_Dou(param_.factor_graph.POS_MEAS_MEAN, param_.factor_graph.POS_MEAS_COV);
  rotation_noise_z = Dist_Dou(0, param_.factor_graph.ROT_MEAS_COV);
  velocity_noise_z = Dist_Dou(0, param_.factor_graph.VEL_MEAS_COV);
}

/* 
 * compute thr_bodyrate_u.thrust and thr_bodyrate_u.q, controller gains and other parameters are in param_ 
 * Differential-Flatness Based Controller (DFBC) Subject to Aerodynamics Drag Force
 */
quadrotor_msgs::Px4ctrlDebug Controller::calculateControl(const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu, 
  Controller_Output_t &thr_bodyrate_u)
{
  /* WRITE YOUR CODE HERE */
  //compute disired acceleration
  Eigen::Vector3d subtract(0,0,0);
  if(des.p[2] >= 2.3f)
  {
    subtract = des.p - Eigen::Vector3d(0, 0, 2.3f);
    ROS_WARN("Des.p >= 2.3f");
  }
  gtsam::Rot3 Rc(odom.q);
  Eigen::Vector3d des_acc(0.0, 0.0, 0.0); // des_acc corresponding to collective thrust in the world coordinate system
  Eigen::Vector3d Kp, Kv, KR, KDrag;
  Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
  Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
  KR << param_.gain.KAngR, param_.gain.KAngP, param_.gain.KAngY;
  KDrag << param_.rt_drag.x, param_.rt_drag.y, param_.rt_drag.z;
  float mass = param_.mass;
  des_acc = des.a + Kv.asDiagonal() * limit_err(des.v - odom.v, param_.gain.VErrMax) + Kp.asDiagonal() * limit_err(des.p - subtract - odom.p, param_.gain.PErrMax);
  des_acc += Eigen::Vector3d(0, 0, param_.gra); // * odom.q * e3
  des_acc += Rc.matrix() * KDrag.asDiagonal() * Rc.inverse().matrix() * odom.v / mass;
  
  // check thrust 
  // if((Rc.inverse().matrix() * des_acc)[2] < 0)
  // {
  //   thr_bodyrate_u.thrust = 0.01f;
  // }
  // else
  // {
  thr_bodyrate_u.thrust = computeDesiredCollectiveThrustSignal(des_acc, odom.v);
  // }

  Eigen::Vector3d force = des_acc * param_.mass;

  // Limit control angle to 80 degree
  double          theta = param_.max_angle;
  double          c     = cos(theta);
  Eigen::Vector3d f;
  f.noalias() = force - param_.mass * param_.gra * Eigen::Vector3d(0, 0, 1);
  if (Eigen::Vector3d(0, 0, 1).dot(force / force.norm()) < c)
  {
    double nf        = f.norm();
    double A         = c * c * nf * nf - f(2) * f(2);
    double B         = 2 * (c * c - 1) * f(2) * param_.mass * param_.gra;
    double C         = (c * c - 1) * param_.mass * param_.mass * param_.gra * param_.gra;
    double s         = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
    force.noalias() = s * f + param_.mass * param_.gra * Eigen::Vector3d(0, 0, 1);
  }
  // Limit control angle to 80 degree

  Eigen::Vector3d b1c, b2c, b3c;
  Eigen::Vector3d b1d(cos(des.yaw), sin(des.yaw), 0);

  if (force.norm() > 1e-6)
    b3c.noalias() = force.normalized();
  else
    b3c.noalias() = Eigen::Vector3d(0, 0, 1);

  b2c.noalias() = b3c.cross(b1d).normalized();
  b1c.noalias() = b2c.cross(b3c).normalized();

  Eigen::Matrix3d R;
  R << b1c, b2c, b3c;

  thr_bodyrate_u.q = Eigen::Quaterniond(R);
  gtsam::Rot3 Rd(thr_bodyrate_u.q);
  thr_bodyrate_u.bodyrates = KR.asDiagonal()* gtsam::Rot3::Logmap(Rc.inverse() * Rd) + des.w;
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = thr_bodyrate_u.q.x();
  debug_msg_.des_q_y = thr_bodyrate_u.q.y();
  debug_msg_.des_q_z = thr_bodyrate_u.q.z();
  debug_msg_.des_q_w = thr_bodyrate_u.q.w();
  
  debug_msg_.des_thr = thr_bodyrate_u.thrust;

  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), thr_bodyrate_u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage 
*/
double Controller::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc, const Eigen::Vector3d &v)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = (des_acc.norm() - param_.rt_drag.k_thrust_horz * (pow(v.x(), 2.0) + pow(v.y(), 2.0)) / param_.mass) / thr2acc_;
  throttle_percentage = limit_value(param_.thr_map.thrust_upper_bound, throttle_percentage, param_.thr_map.thrust_lower_bound);
  return throttle_percentage;
}

/*
  compute throttle percentage 
*/
double Controller::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc.norm() / thr2acc_;
  throttle_percentage = limit_value(param_.thr_map.thrust_upper_bound, throttle_percentage, param_.thr_map.thrust_lower_bound);
  return throttle_percentage;
}

bool Controller::estimateThrustModel(const Eigen::Vector3d &est_a, const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose thrust data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();

    /***********************************/
    /* Model: est_a(2) = thr2acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    printf("Thrust debug [ thr2acc: %6.3f, gamma: %6.3f, K: %6.3f, P: %6.3f, thrust: %6.3f, est_a(2): %6.3f ]\n", thr2acc_, gamma, K, P_, thr, est_a(2));
    fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void Controller::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}


double Controller::limit_value(double upper_bound, double input, double lower_bound)
{
  if(upper_bound <= lower_bound)
  {
    log_ << "Warning: upper_bound <= lower_bound\n";
  }
  if(input > upper_bound)
  {
    input = upper_bound;
  }
  if(input < lower_bound)
  {
    input = lower_bound;
  }
  return input;
}

Controller::~Controller()
{
  log_.close();
}

Eigen::Vector3d Controller::limit_err(const Eigen::Vector3d err, const double p_err_max)
{
  Eigen::Vector3d r_err(0, 0, 0);
  for(uint i = 0; i < 3; i++)
  {
    r_err[i] = limit_value(std::abs(p_err_max), err[i], -std::abs(p_err_max));
  }
  return r_err;
}

Odom_Data_t Controller::add_Guassian_noise(const Odom_Data_t &odom)
{
  gtsam::Vector3 pos_noise = gtsam::Vector3(position_noise_x(__randomGen), position_noise_y(__randomGen), position_noise_z(__randomGen));
  gtsam::Vector3 vel_noise = gtsam::Vector3(velocity_noise_x(__randomGen), velocity_noise_y(__randomGen), velocity_noise_z(__randomGen));
  gtsam::Vector3 rot_noise = gtsam::Vector3(rotation_noise_x(__randomGen), rotation_noise_y(__randomGen), rotation_noise_z(__randomGen));
  gtsam::Vector3 rot_add   = gtsam::Rot3::Logmap(gtsam::Rot3(odom.q)) + rot_noise;
  gtsam::Rot3    rot3_add  = gtsam::Rot3::Expmap(rot_add);

  Odom_Data_t odom_noise;
  odom_noise.rcv_stamp = odom.rcv_stamp;
  odom_noise.p         = odom.p + pos_noise;
  odom_noise.v         = odom.v + vel_noise;
  odom_noise.q         = Eigen::Quaterniond(rot3_add.toQuaternion().w(), rot3_add.toQuaternion().x(), rot3_add.toQuaternion().y(), rot3_add.toQuaternion().z());

  return odom_noise;
}







