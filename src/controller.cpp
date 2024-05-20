#include "controller.h"

using namespace gtsam;
using namespace std;



double SE3Control::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

SE3Control::SE3Control(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
  time_t now = time(NULL);
	tm* t = localtime(&now);

	// 将信息输出到字符串流
	stringstream ss; ss << "/home/amov/output/controller_log_";
	ss << t->tm_year + 1900 << "." << t->tm_mon + 1 << "." << t->tm_mday << "." << t->tm_hour << "." << t->tm_min << "." << t->tm_sec << ".txt";
  std::cout << " -- log file:" << ss.str() << std::endl;
  log_.open(ss.str(), std::ios::out);
}

/* 
 * compute u.thrust and u.q, controller gains and other parameters are in param_ 
 * Differential-Flatness Based Controller (DFBC) Subject to Aerodynamics Drag Force
 */
quadrotor_msgs::Px4ctrlDebug SE3Control::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
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
  des_acc = des.a + Kv.asDiagonal() * limit_err(des.v - odom.v, param_.gain.PErrMax) + Kp.asDiagonal() * limit_err(des.p - subtract - odom.p, param_.gain.VErrMax);
  des_acc += Eigen::Vector3d(0, 0, param_.gra); // * odom.q * e3
  des_acc += Rc.matrix() * KDrag.asDiagonal() * Rc.inverse().matrix() * odom.v / mass;
  
  // check thrust 
  // if((Rc.inverse().matrix() * des_acc)[2] < 0)
  // {
  //   u.thrust = 0.01f;
  // }
  // else
  // {
  u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
  // }

  Eigen::Vector3d force = des_acc * param_.mass;

  // Limit control angle to 90 degree
  double          theta = 80.0f / 180.0f * M_PI;
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
  // Limit control angle to 90 degree

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

  u.q = Eigen::Quaterniond(R);
  gtsam::Rot3 Rd(u.q);
  u.bodyrates = KR.asDiagonal()* gtsam::Rot3::Logmap(Rc.inverse() * Rd) + des.w;
  log_ << " -- cur_p [ " << odom.p.transpose() << " ], cur_v: [ " << odom.v.transpose() << std::endl;
  log_ << " -- des_acc: [ " << des_acc.transpose() << " ], des_a: [ " << des.a.transpose() << " ], des_v: [ " << des.v.transpose() << " ], des_p: [ " << des.p.transpose() << std::endl;
  log_ << " -- control u: [ " << u.thrust << " ], body_rate: [ " << u.bodyrates.transpose() << std::endl;
  /* WRITE YOUR CODE HERE */

  //used for debug
  // debug_msg_.des_p_x = des.p(0);
  // debug_msg_.des_p_y = des.p(1);
  // debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;

  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage 
*/
double SE3Control::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc.norm() / thr2acc_;
  throttle_percentage = limit_value(param_.thr_map.thrust_upper_bound, throttle_percentage, param_.thr_map.thrust_lower_bound);
  return throttle_percentage;
}

bool  SE3Control::estimateThrustModel(const Eigen::Vector3d &est_a, const Parameter_t &param)
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

void SE3Control::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}


double SE3Control::limit_value(double upper_bound, double input, double lower_bound)
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

SE3Control::~SE3Control()
{
  log_.close();
}

Eigen::Vector3d SE3Control::limit_err(const Eigen::Vector3d err, const double p_err_max)
{
  Eigen::Vector3d r_err(0, 0, 0);
  for(uint i = 0; i < 3; i++)
  {
    r_err[i] = limit_value(std::abs(p_err_max), err[i], -std::abs(p_err_max));
  }
  return r_err;
}







