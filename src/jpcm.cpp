#include "jpcm.h"
#include <time.h>

using namespace UAVFactor;

using symbol_shorthand::S;
using symbol_shorthand::U;
using symbol_shorthand::V;
using symbol_shorthand::X;

quadrotor_msgs::Px4ctrlDebug JCPM_TGyro::calculateControl(const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu, 
    Controller_Output_t &thr_bodyrate_u, CTRL_MODE mode_switch)
{
    DFBControl::calculateControl(des, odom, imu, thr_bodyrate_u);

    bool timeout = false;
    double opt_cost = 0.0f;
    double thrust = 0;
    gtsam::Vector3 bodyrates;
    if(des_vec_.size() > 0)
    {
        if((des.rcv_stamp - des_vec_[des_vec_.size()-1].rcv_stamp).toSec() > 0.015f 
            || (des.rcv_stamp - des_vec_[des_vec_.size()-1].rcv_stamp).toSec() < 0.001f)
        {
            des_vec_.clear();
            timeout = true;
        }
    }

    des_vec_.push_back(des);
    if(timeout || mode_switch == DFBC || des_vec_.size() < opt_lens_traj_)
    {
        // DFBControl::calculateControl(des, odom, imu, thr_bodyrate_u);
    }
    else if(mode_switch == MPC && des_vec_.size() == opt_lens_traj_)
    {
        clock_t start, end;

        gtsam::LevenbergMarquardtParams parameters;
        parameters.absoluteErrorTol = 1e-2;
        parameters.relativeErrorTol = 1e-3;
        parameters.maxIterations    = 10;
        parameters.verbosity        = gtsam::NonlinearOptimizerParams::ERROR;
        parameters.verbosityLM      = gtsam::LevenbergMarquardtParams::SUMMARY;
        graph_.empty();

        buildFactorGraph(graph_, initial_value_, des_vec_, odom, dt_);

        std::cout << "###################### Init optimizer ######################" << std::endl;
        LevenbergMarquardtOptimizer optimizer(graph_, initial_value_, parameters);

        std::cout << "###################### Begin optimize ######################" << std::endl;
        start = clock();
        Values result = optimizer.optimize();
	    end = clock();
        opt_cost = (double)(end-start)/CLOCKS_PER_SEC;
	    std::cout << " ---------- Optimize Time: " << opt_cost << endl;
        
        gtsam::Vector4 input;
        input = result.at<gtsam::Vector4>(U(0));
        Eigen::Vector3d des_acc(0, 0, input[0]);

        thrust = DFBControl::computeDesiredCollectiveThrustSignal(des_acc, odom.v);
        bodyrates = Eigen::Vector3d(input[1], input[2], input[3]);

        // thr_bodyrate_u.thrust    = DFBControl::computeDesiredCollectiveThrustSignal(des_acc, odom.v);
        // thr_bodyrate_u.bodyrates = Eigen::Vector3d(input[1], input[2], input[3]);

    }

    log_ << " -- cur_p: [ " << odom.p.transpose() << "], des_p: [ " << des.p.transpose() 
         << " cur_v: [ " << odom.v.transpose() << " ], des_v: [ " << des.v.transpose() << std::endl;
    // log_ << " -- des_acc: [ " << des_acc.transpose() << " ], des_a: [ " << des.a.transpose() << std::endl;
    log_ << des.rcv_stamp.toSec() << " " << timeout << " " << des_vec_.size() << " - JPCM: " << thrust << " " << bodyrates.x() << " " << bodyrates.y() << " " 
         << bodyrates.z() << " " << opt_cost << " - DFBC: " << thr_bodyrate_u.thrust << " " << thr_bodyrate_u.bodyrates.x() << " "
         << thr_bodyrate_u.bodyrates.y() << " " << thr_bodyrate_u.bodyrates.z() << std::endl;

    if(des_vec_.size() >= opt_lens_traj_)
    {
        des_vec_.erase(des_vec_.begin());
    }
}

void JCPM_TGyro::buildFactorGraph(gtsam::NonlinearFactorGraph& graph, gtsam::Values& initial_value, const std::vector<Desired_State_t> &des_v, const Odom_Data_t &odom, double dt)
{
    const Parameter_t params = get_param();
    
    std::default_random_engine meas_x_gen;
    std::default_random_engine meas_y_gen;
    std::default_random_engine meas_z_gen;
    
    std::default_random_engine meas_rx_gen;
    std::default_random_engine meas_ry_gen;
    std::default_random_engine meas_rz_gen;

    std::default_random_engine meas_vx_gen;
    std::default_random_engine meas_vy_gen;
    std::default_random_engine meas_vz_gen;

    std::normal_distribution<double> position_noise(params.factor_graph.POS_MEAS_MEAN, params.factor_graph.POS_MEAS_COV);
    std::normal_distribution<double> rot_noise     (0, params.factor_graph.POS_MEAS_COV);
    std::normal_distribution<double> velocity_noise(0, params.factor_graph.ROT_MEAS_COV);

    auto input_jerk  = noiseModel::Diagonal::Sigmas(Vector4(params.factor_graph.INPUT_JERK_T, 
        params.factor_graph.INPUT_JERK_M, params.factor_graph.INPUT_JERK_M, params.factor_graph.INPUT_JERK_M3));

    auto dynamics_noise = noiseModel::Diagonal::Sigmas((Vector(9) << Vector3::Constant(params.factor_graph.DYNAMIC_P_COV), 
        Vector3::Constant(0.0001), Vector3::Constant(0.001)).finished());
    
    // Initial state noise
    auto vicon_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(params.factor_graph.ROT_MEAS_COV), Vector3::Constant(params.factor_graph.PRI_VICON_COV)).finished());
    auto vel_noise   = noiseModel::Diagonal::Sigmas(Vector3(params.factor_graph.PRI_VICON_VEL_COV, params.factor_graph.PRI_VICON_VEL_COV, params.factor_graph.PRI_VICON_VEL_COV));

    auto ref_predict_vel_noise = noiseModel::Diagonal::Sigmas(Vector3(params.factor_graph.CONTROL_V_COV, params.factor_graph.CONTROL_V_COV, params.factor_graph.CONTROL_V_COV));
    // auto ref_predict_omega_noise = noiseModel::Diagonal::Sigmas(Vector3(params.factor_graph.CONTROL_O_COV, params.factor_graph.CONTROL_O_COV, params.factor_graph.CONTROL_O_COV));

    graph.empty();

    auto clf_sigma = noiseModel::Diagonal::Sigmas(Vector4(1.0, 1.0, 1.0, 1.0));
    ControlLimitTGyroFactor cntrolLimitTGyroFactor(U(0), clf_sigma, params.factor_graph.low, params.factor_graph.high, params.factor_graph.thr,
        params.factor_graph.glow, params.factor_graph.ghigh, params.factor_graph.gthr, params.factor_graph.alpha);
    graph.add(cntrolLimitTGyroFactor);

    gtsam::Vector3 drag_k(params.rt_drag.x, params.rt_drag.y, params.rt_drag.z);

    for (uint16_t idx = 0; idx < params.factor_graph.OPT_LENS_TRAJ; idx++)
    {
        DynamicsFactorTGyro dynamics_factor(X(idx), V(idx), U(idx), X(idx + 1), V(idx + 1), dt, params.mass, drag_k, dynamics_noise);
        graph.add(dynamics_factor);
        
        gtsam::Pose3   pose_idx(gtsam::Rot3(des_v[idx].q), des_v[idx].p);
        gtsam::Vector3 vel_idx   = des_v[idx].v;
        gtsam::Vector3 omega_idx = des_v[idx].w;
        
        initial_value.insert(X(idx + 1), pose_idx);
        initial_value.insert(V(idx + 1), vel_idx);

        if(idx != 0)
        {
            BetForceMoments bet_FM_factor(U(idx - 1), U(idx), input_jerk);
            graph.add(bet_FM_factor);
        }
        
        gtsam::Vector4 init_input(10, 0, 0, 0);
        initial_value.insert(U(idx), init_input);

        gtsam::Vector3 control_r_cov(params.factor_graph.CONTROL_R1_COV, params.factor_graph.CONTROL_R2_COV, params.factor_graph.CONTROL_R3_COV);
        if(idx == params.factor_graph.OPT_LENS_TRAJ - 1)
        {   
            gtsam::Vector3 final_position_ref(params.factor_graph.CONTROL_PF_COV_X, params.factor_graph.CONTROL_PF_COV_Y, params.factor_graph.CONTROL_PF_COV_Z);
            auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << control_r_cov, final_position_ref).finished()); 
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
        }
        else
        {
            gtsam::Vector3 _position_ref(params.factor_graph.CONTROL_P_COV_X, params.factor_graph.CONTROL_P_COV_Y, params.factor_graph.CONTROL_P_COV_Z);
            auto ref_predict_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << control_r_cov, _position_ref).finished());
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(idx + 1), pose_idx, ref_predict_pose_noise));
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx + 1), vel_idx, ref_predict_vel_noise));
        }

        if (idx == 0)
        {                
            gtsam::Vector3 pos_noise     = gtsam::Vector3(position_noise(meas_x_gen), position_noise(meas_y_gen), position_noise(meas_z_gen));
            gtsam::Vector3 vel_noise_add = gtsam::Vector3(velocity_noise(meas_vx_gen), velocity_noise(meas_vy_gen), velocity_noise(meas_vz_gen));
            gtsam::Vector3 rot_noise_add = gtsam::Vector3(rot_noise(meas_rx_gen), rot_noise(meas_ry_gen), rot_noise(meas_rz_gen));
            
            gtsam::Vector3 pos_add = odom.p + pos_noise;
            gtsam::Vector3 vel_add = odom.v + vel_noise_add;
            gtsam::Vector3 rot_add = gtsam::Rot3::Logmap(gtsam::Rot3(odom.q)) + rot_noise_add;
            gtsam::Rot3   rot3_add = gtsam::Rot3::Expmap(rot_add);
            gtsam::Pose3  pose_add(rot3_add, pos_add);

            graph.add(gtsam::PriorFactor<gtsam::Pose3>  (X(idx), pose_add, vicon_noise));
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(idx), vel_add, vel_noise));
            
            initial_value.insert(X(idx), pose_add);
            initial_value.insert(V(idx), vel_add);
        }

    }
}
