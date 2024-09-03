#include <ros/ros.h>
#include "controller.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "test_jpcm");
  ros::NodeHandle nh("~");

  Parameter_t param;
  param.config_from_ros_handle(nh);

  Desired_State_t des;
  Odom_Data_t     odom;
  Imu_Data_t      imu;
  Imu_Data_t      imu_raw;
  imu_raw.a = Eigen::Vector3d(0, 0, 9.81);
  imu_raw.w = Eigen::Vector3d(0, 0, 0);

  float v  = 0.5;
  float dt = 0.01;

  for(int test_idx = 0; test_idx < 1; test_idx++)
  {
    // int test_idx = 60;
    // odom.p = Eigen::Vector3d(0.005*(test_idx - 50), 0.005*(test_idx - 50), 0.00);
    odom.p = Eigen::Vector3d(0.0, 0.0, 0.0);
    odom.v = Eigen::Vector3d(0.0, 0.0, 0.0);

    // gtsam::Vector3 rzyx(0, 0, 10.0/180.0*3.14159);
    // gtsam::Rot3 rot = gtsam::Rot3::RzRyRx(rzyx);
    gtsam::Rot3 rot = gtsam::Rot3::identity();
    odom.q          = rot.toQuaternion();

    des.p = Eigen::Vector3d(0,0,0);
    des.v = Eigen::Vector3d(0,0,v);
    rot   = gtsam::Rot3::identity();
    des.q = rot.toQuaternion();

    DFBControl controller(param);
    Controller_Output_t ctrl_cmd;
    CTRL_MODE MPC = CTRL_MODE::MPC;

    for(int i = 0; i < param.factor_graph.OPT_LENS_TRAJ+2; i++)
    {
      des.p = Eigen::Vector3d(0,0,v*(i+1)*dt);
      des.v = Eigen::Vector3d(0,0,v);
      std::cout << "calculateControl" << i << std::endl;
      controller.calculateControl(des, odom, imu, imu_raw, ctrl_cmd, MPC);
    }

    std::cout << "MPC ctrl thrust: " << ctrl_cmd.mpc_thrust << std::endl;
    std::cout << "MPC ctrl bodyrate: [" << ctrl_cmd.mpc_bodyrates.x() << ", " << ctrl_cmd.mpc_bodyrates.y() << ", " << ctrl_cmd.mpc_bodyrates.z() << "]" << std::endl;

    std::cout << "DFBC ctrl thrust: " << ctrl_cmd.thrust << std::endl;
    std::cout << "DFBC ctrl bodyrate: [" << ctrl_cmd.bodyrates.x() << ", " << ctrl_cmd.bodyrates.y() << ", " << ctrl_cmd.bodyrates.z() << "]" << std::endl;
  
  }
  
  return 0;
}
