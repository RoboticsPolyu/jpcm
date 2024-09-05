#ifndef __DYNAMICS_PLANNING_FACTOR_H__
#define __DYNAMICS_PLANNING_FACTOR_H__

#include "gtsam_wrapper.h"
#include <vector>

using namespace gtsam_wrapper;

namespace UAVFactor
{

//------------------------------------------------------------------------------
std::pair<Vector3, Vector3> correctMeasurementsBySensorPose(
    const Vector3& unbiasedAcc, const Vector3& unbiasedOmega, const gtsam::Pose3& body_P_sensor,
    OptionalJacobian<3, 3> correctedAcc_H_unbiasedAcc,
    OptionalJacobian<3, 3> correctedAcc_H_unbiasedOmega,
    OptionalJacobian<3, 3> correctedOmega_H_unbiasedOmega) 
{

    // Compensate for sensor-body displacement if needed: we express the quantities
    // (originally in the IMU frame) into the body frame
    // Equations below assume the "body" frame is the CG

    // Get sensor to body rotation matrix
    const Matrix3 bRs = body_P_sensor.rotation().matrix();

    // Convert angular velocity and acceleration from sensor to body frame
    Vector3 correctedAcc = bRs * unbiasedAcc;
    const Vector3 correctedOmega = bRs * unbiasedOmega;

    // Jacobians
    if (correctedAcc_H_unbiasedAcc) *correctedAcc_H_unbiasedAcc = bRs;
    if (correctedAcc_H_unbiasedOmega) *correctedAcc_H_unbiasedOmega = Z_3x3;
    if (correctedOmega_H_unbiasedOmega) *correctedOmega_H_unbiasedOmega = bRs;

    // Centrifugal acceleration
    const Vector3 b_arm = body_P_sensor.translation();
    if (!b_arm.isZero()) {
        // Subtract out the the centripetal acceleration from the unbiased one
        // to get linear acceleration vector in the body frame:
        const Matrix3 body_Omega_body = skewSymmetric(correctedOmega);
        const Vector3 b_velocity_bs = body_Omega_body * b_arm; // magnitude: omega * arm
        correctedAcc -= body_Omega_body * b_velocity_bs;

        // Update derivative: centrifugal causes the correlation between acc and omega!!!
        if (correctedAcc_H_unbiasedOmega) {
        double wdp = correctedOmega.dot(b_arm);
        const Matrix3 diag_wdp = Vector3::Constant(wdp).asDiagonal();
        *correctedAcc_H_unbiasedOmega = -( diag_wdp
            + correctedOmega * b_arm.transpose()) * bRs.matrix()
            + 2 * b_arm * unbiasedOmega.transpose();
        }
    }

    return std::make_pair(correctedAcc, correctedOmega);
}
/*
* MPC based FGO, generating Thrust and Gyro
*/
class GTSAM_EXPORT DynamicsFactorTGyro : public NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, gtsam::Pose3, gtsam::Vector3>
{
public:
    typedef boost::shared_ptr<DynamicsFactorTGyro> shared_ptr;

    DynamicsFactorTGyro() {}
    DynamicsFactorTGyro(Key p_i, Key vel_i, Key input_i, Key p_j, Key vel_j, float dt, double mass, gtsam::Vector3 drag_k, const SharedNoiseModel &model);

    virtual ~DynamicsFactorTGyro()
    {
    }

    Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam::Vector4 &input_i,
                            const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, 
                            boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                            boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                            boost::optional<Matrix &> H5 = boost::none) const;

private:
    typedef DynamicsFactorTGyro This;
    typedef NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3, gtsam::Vector4, gtsam::Pose3, gtsam::Vector3> Base;
    
    float           dt_    = 0.01f;
    double         mass_   = 1.0f;
    gtsam::Vector3 drag_k_ = gtsam::Vector3(0, 0, 0);
    gtsam::Vector3 gI_     = gtsam::Vector3(0, 0, 9.81); // gravity

};

class GTSAM_EXPORT IMUFactor : public NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3, gtsam_imuBi, gtsam::Pose3, gtsam::Vector3>
{
public:
    typedef boost::shared_ptr<IMUFactor> shared_ptr;

    IMUFactor() {}
    IMUFactor(Key p_i, Key vel_i, Key bias_i, Key p_j, Key vel_j, 
        float dt, gtsam::Vector3 acc, gtsam::Vector3 gyro, const SharedNoiseModel &model);

    virtual ~IMUFactor()
    {
    }

    Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam_imuBi &bias_i,
                            const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, 
                            boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                            boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                            boost::optional<Matrix &> H5 = boost::none) const;

private:
    typedef IMUFactor This;
    typedef NoiseModelFactor5<gtsam::Pose3, gtsam::Vector3, gtsam_imuBi, gtsam::Pose3, gtsam::Vector3>
        Base;
    
    float           dt_    = 0.01f;
    double         mass_   = 1.0f;
    gtsam::Vector3 acc_;
    gtsam::Vector3 gyro_;
    gtsam::Vector3 gI_     = gtsam::Vector3(0, 0, 9.81); // gravity

};

class GTSAM_EXPORT IMUFactorRg : public NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam_imuBi, gtsam::Pose3, gtsam::Vector3, gtsam::Rot3>
{
public:
    typedef boost::shared_ptr<IMUFactorRg> shared_ptr;

    IMUFactorRg() {}
    IMUFactorRg(Key p_i, Key vel_i, Key bias_i, Key p_j, Key vel_j, Key Rg,
        float dt, gtsam::Vector3 acc, gtsam::Vector3 gyro, const SharedNoiseModel &model);

    virtual ~IMUFactorRg()
    {
    }

    Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam_imuBi &bias_i,
                            const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Rot3 &Rg,
                            boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                            boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                            boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none) const;

private:
    typedef IMUFactorRg This;
    typedef NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam_imuBi, gtsam::Pose3, gtsam::Vector3, gtsam::Rot3>
        Base;
    
    float           dt_    = 0.01f;
    double         mass_   = 1.0f;
    gtsam::Vector3 acc_;
    gtsam::Vector3 gyro_;
    gtsam::Vector3 gI_     = gtsam::Vector3(0, 0, 9.81); // gravity

};

class GTSAM_EXPORT IMUFactorRa : public NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam_imuBi, gtsam::Pose3, gtsam::Vector3, gtsam::Rot3>
{
public:
    typedef boost::shared_ptr<IMUFactorRa> shared_ptr;

    IMUFactorRa() {}
    IMUFactorRa(Key p_i, Key vel_i, Key bias_i, Key p_j, Key vel_j, Key Ra,
        float dt, gtsam::Vector3 acc, gtsam::Vector3 gyro, const SharedNoiseModel &model);

    virtual ~IMUFactorRa()
    {
    }

    Vector evaluateError(const gtsam::Pose3 &pos_i, const gtsam::Vector3 &vel_i, const gtsam_imuBi &bias_i,
                            const gtsam::Pose3 &pos_j, const gtsam::Vector3 &vel_j, const gtsam::Rot3 &Ra,
                            boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none,
                            boost::optional<Matrix &> H3 = boost::none, boost::optional<Matrix &> H4 = boost::none,
                            boost::optional<Matrix &> H5 = boost::none, boost::optional<Matrix &> H6 = boost::none) const;

private:
    typedef IMUFactorRa This;
    typedef NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam_imuBi, gtsam::Pose3, gtsam::Vector3, gtsam::Rot3>
        Base;
    
    float           dt_    = 0.01f;
    double         mass_   = 1.0f;
    gtsam::Vector3 acc_;
    gtsam::Vector3 gyro_;
    gtsam::Vector3 gI_     = gtsam::Vector3(0, 0, 9.81); // gravity

};

class GTSAM_EXPORT ControlLimitTGyroFactor : public NoiseModelFactor1<gtsam::Vector4>
{
public:
    typedef boost::shared_ptr<ControlLimitTGyroFactor> shared_ptr;

    ControlLimitTGyroFactor() {}
    ControlLimitTGyroFactor(Key input, const SharedNoiseModel &model, 
        double T_low, double T_high, double Gyro_low, double Gyro_high,
        double T_thr, double Gyro_thr, double alpha)
        : Base(model, input)
        , T_high_(T_high)
        , T_low_(T_low)
        , Gyro_high_(Gyro_high)
        , Gyro_low_(Gyro_low)
        , T_thr_(T_thr)
        , Gyro_thr_(Gyro_thr)
        , alpha_(alpha){};

    virtual ~ControlLimitTGyroFactor()
    {
    }

    Vector evaluateError(const gtsam::Vector4 &input, boost::optional<Matrix &> H1 = boost::none) const;

private:
    typedef ControlLimitTGyroFactor This;
    typedef NoiseModelFactor1<gtsam::Vector4>
        Base;

    double T_high_;
    double T_low_;

    double Gyro_high_;
    double Gyro_low_;

    double T_thr_;
    double Gyro_thr_;

    double alpha_;
};


/* Force and Moments Between factor */
class GTSAM_EXPORT BetForceMoments : public NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4>
{
public:
    typedef boost::shared_ptr<BetForceMoments> shared_ptr;

    BetForceMoments() {}
    BetForceMoments(Key input_i, Key input_j, const SharedNoiseModel &model);

    virtual ~BetForceMoments()
    {
    }

    Vector evaluateError(const gtsam::Vector4 &input_i, const gtsam::Vector4 &input_j,
                            boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none
                            ) const;

private:
    typedef BetForceMoments This;
    typedef NoiseModelFactor2<gtsam::Vector4, gtsam::Vector4>
        Base;
};

}

#endif // __DYNAMICS_PLANNING_FACTOR_H__