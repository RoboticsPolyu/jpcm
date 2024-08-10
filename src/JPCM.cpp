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


JCPM_TGyro::JCPM_TGyro(Parameter_t &param) : param_(param)
{
    graph_.empty(); 
    dt_            = 0.01f; 
    opt_traj_lens_ = param_.factor_graph.OPT_LENS_TRAJ;
    window_lens_   = param_.factor_graph.WINDOW_SIZE;
}


bool JCPM_TGyro::JPCM_control(const std::vector<Desired_State_t> &des, 
    const std::vector<Odom_Data_t> &odom, const std::vector<Imu_Data_t> &imu, gtsam::Vector4 &input)
{

    return true;
}


void JCPM_TGyro::buildFactorGraph(gtsam::NonlinearFactorGraph& _graph, gtsam::Values& _initial_value, 
    const std::vector<Desired_State_t> &des_v, const std::vector<Odom_Data_t> &odom_v, 
    const std::vector<Imu_Data_t> &imu_v, double dt)
{

}

void JCPM_TGyro::buildFactorGraph(gtsam::NonlinearFactorGraph& _graph, gtsam::Values& _initial_value, 
    const std::vector<Desired_State_t> &des_v, const Odom_Data_t &odom, double dt)
{
  
}
