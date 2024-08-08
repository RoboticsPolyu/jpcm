#ifndef __GPS_IMU__
#define __GPS_IMU__

#include "input.h"
#include "PX4CtrlParam.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <cstring>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)


struct ImuMeasurement {
  double time;
  double dt;
  Vector3 accelerometer;
  Vector3 gyroscope;  // omega
};

struct GpsMeasurement {
  double time;
  Vector3 position;  // x,y,z
};

struct GIO_Params
{
    double body_ptx;
    double body_pty;
    double body_ptz;
    double body_prx;
    double body_pry;
    double body_prz;
    double accelerometer_sigma;
    double gyroscope_sigma;
    double integration_sigma;
    double accelerometer_bias_sigma;
    double gyroscope_bias_sigma;
    double average_delta_t;
    double gps_sigma;
    double gps_v_sigma;
    
    GIO_Params()
    {
        body_ptx = 0;

    }
    GIO_Params(double _body_ptx)
    {
        body_ptx = _body_ptx;

    };

};

class GIO
{
    public:
        GIO(GIO_Params gio_params)
            :gio_params_(gio_params)
            {

            }

        Odom_Data_t odom_data;
	    Imu_Datas_t imu_raw_data;

        void Process();

    private:
        GIO_Params gio_params_;
};

#endif