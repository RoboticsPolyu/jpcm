#include "CBF_factor.h"

namespace uavfactor
{
   Vector VeCBFPdFactor::evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, const gtsam::Vector4 &ui,
         boost::optional<Matrix &> H1, boost::optional<Matrix &> H2, boost::optional<Matrix &> H3) const
   {
      Vector err;
      Matrix36 jac_ti;
      // err = (- Vector1(1.* safe_d_ * safe_d_) 
      //      + (pi.translation(jac_t_posei) - obs_).transpose() * (pi.translation(jac_t_posei) - obs_)); 
      gtsam::Vector3 pi_t = pi.translation(jac_ti);
      gtsam::Vector3 p_p0 = pi_t - obs_;
      double d = p_p0.norm();
      gtsam::Vector1 hi = Vector1(1/safe_d_ - 1/d) + beta_* p_p0.transpose()/d* (vi - obs_vel_);
      
      gtsam::Vector3 _e2(0., 0., 1.);
      gtsam::Vector3 _g(0., 0., 9.81);
      gtsam::Vector3 _ai = _e2 * ui(0);
      
      gtsam::Matrix36 J_ri;
      gtsam::Matrix3  J_r_ri;
      gtsam::Matrix3  J_r_ai;

      gtsam::Vector3 x_dot_0 = vi;
      gtsam::Vector3 x_dot_2 = -_g + pi.rotation(J_ri).rotate(_ai, J_r_ri, J_r_ai);
      err = hi + alpha_* evaluateH_ti(pi_t, vi)* x_dot_0 + alpha_* evaluateH_vi(pi_t)* x_dot_2; // h + alpha* dot h

      if(err(0) > 0)
      {
            err(0) = 0;
            if(H1)
            {
               *H1 = gtsam::Matrix16::Zero();
            }

            if(H2)
            {
               *H2 = gtsam::Matrix13::Zero();
            }
            
            if(H3)
            {
               *H3 = gtsam::Matrix14::Zero();
            }
      }
      else
      {
            if(H1) 
            {
               *H1 = evaluateH_ti_err(pi_t, pi.rotation(), vi, ui)* jac_ti 
                  + alpha_* evaluateH_vi(pi_t)* J_r_ri* J_ri ;
            }

            if(H2)
            {
               *H2 = evaluateH_vi_err(pi_t, vi);
            }

            if(H3)
            {
               gtsam::Matrix14 h3; h3.setZero();
               gtsam::Matrix13 J_ai = alpha_* evaluateH_vi(pi_t)* J_r_ai;
               h3.block<1, 1>(0, 0) = (J_ai * _e2); 
               *H3 = h3;
            }
      }

      return err;
   }


   gtsam::Vector VeCBFPdFactor1::evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, const gtsam::Vector4 &ui,
                                          boost::optional<gtsam::Matrix &> H1, boost::optional<gtsam::Matrix &> H2, 
                                          boost::optional<gtsam::Matrix &> H3) const
   {
      double epsilon = 1e-6;

      gtsam::Matrix36 jac_ti;
      gtsam::Vector3 p_p0 = pi.translation(jac_ti) - obs_;
      double d = p_p0.norm();
      double d_eps = std::max(d, epsilon);
      gtsam::Matrix13 p_p0_t = p_p0.transpose();
      gtsam::Vector3 v_diff = vi - obs_vel_;
      double p_dot_vdiff = p_p0.dot(v_diff);

      gtsam::Matrix13 H_ti = evaluateH_ti(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff);
      gtsam::Matrix13 H_vi = evaluateH_vi(p_p0, d_eps, p_p0_t);

      gtsam::Vector3 _e2(0., 0., 1.);  // Note: Assumes z-axis thrust; ui(1:3) unused.
      gtsam::Vector3 _g(0., 0., 9.81);
      gtsam::Vector3 _ai = _e2 * ui(0);

      gtsam::Matrix36 J_ri;
      gtsam::Matrix3 J_r_ri, J_r_ai;
      gtsam::Vector3 R_ai = pi.rotation(J_ri).rotate(_ai, J_r_ri, J_r_ai);
      gtsam::Vector3 term3_vec = -_g + R_ai;  // a_i

      gtsam::Vector1 hi = gtsam::Vector1(1.0 / safe_d_ - 1.0 / d_eps) + beta_  / d_eps * p_p0_t * v_diff;
      gtsam::Vector1 x_dot_0_term = H_ti * vi;
      gtsam::Vector1 x_dot_2_term = H_vi * term3_vec;
      gtsam::Vector1 err = hi + alpha_ * x_dot_0_term + alpha_ * x_dot_2_term;

      if (err(0) > 0.0)
      {
         err(0) = 0.0;
         if (H1) *H1 = gtsam::Matrix::Zero(1, 6);
         if (H2) *H2 = gtsam::Matrix::Zero(1, 3);
         if (H3) *H3 = gtsam::Matrix::Zero(1, 4);
      }
      else
      {
         if (H1) 
         {
               gtsam::Matrix13 H_ti_err = evaluateH_ti_err(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff, term3_vec, H_ti);
               *H1 = H_ti_err * jac_ti + alpha_ * H_vi * J_r_ri * J_ri;
         }

         if (H2)
         {
               gtsam::Matrix13 H_vi_err = evaluateH_vi_err(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff, H_ti, H_vi);
               *H2 = H_vi_err;
         }
         
         if (H3)
         {
               gtsam::Matrix14 h3 = gtsam::Matrix14::Zero();
               gtsam::Matrix13 J_ai = alpha_ * H_vi * J_r_ai;
               h3.block<1, 1>(0, 0) = J_ai * _e2;  // Scalar, but matrix form
               *H3 = h3;
         }
      }

      return err;
   }


   Vector VeCBFPdFactorcCylinder::evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, const gtsam::Vector4 &ui,
         boost::optional<Matrix &> H1, boost::optional<Matrix &> H2, boost::optional<Matrix &> H3) const
   {
      Vector err;
      Matrix36 jac_ti;

      // err = (- Vector1(1.* safe_d_ * safe_d_) 
      //      + (pi.translation(jac_t_posei) - obs_).transpose() * (pi.translation(jac_t_posei) - obs_)); 
      gtsam::Vector3 pi_t = (_E12 * pi.translation(jac_ti));
      gtsam::Vector3 p_p0 = pi_t - obs_;

      gtsam::Vector3 _vi = (_E12 * vi);

      double d = p_p0.norm();
      gtsam::Vector1 hi = Vector1(1/safe_d_ - 1/d) + beta_* p_p0.transpose()/d* (_vi - obs_vel_);
      
      gtsam::Vector3 _e2(0., 0., 1.);
      gtsam::Vector3 _g(0., 0., 9.81);
      gtsam::Vector3 _ai = _e2 * ui(0);
      
      gtsam::Matrix36 J_ri;
      gtsam::Matrix3  J_r_ri;
      gtsam::Matrix3  J_r_ai;

      gtsam::Vector3 x_dot_0 = _vi;
      gtsam::Vector3 x_dot_2 = -_g + pi.rotation(J_ri).rotate(_ai, J_r_ri, J_r_ai);
      err = hi + alpha_* evaluateH_ti(pi_t, _vi)* x_dot_0 + alpha_* evaluateH_vi(pi_t)* x_dot_2; // h + alpha* dot h

      if(err(0) > 0)
      {
            err(0) = 0;
            if(H1)
            {
               *H1 = gtsam::Matrix16::Zero();
            }

            if(H2)
            {
               *H2 = gtsam::Matrix13::Zero();
            }
            
            if(H3)
            {
               *H3 = gtsam::Matrix14::Zero();
            }
      }
      else
      {
            if(H1) 
            {
               *H1 = evaluateH_ti_err(pi_t, pi.rotation(), _vi, ui)* jac_ti 
                  + alpha_* evaluateH_vi(pi_t)* _E12 * J_r_ri * J_ri ;
            }

            if(H2)
            {
               *H2 = evaluateH_vi_err(pi_t, _vi)* _E12;
            }

            if(H3)
            {
               gtsam::Matrix14 h3; h3.setZero();
               gtsam::Matrix13 J_ai = alpha_* evaluateH_vi(pi_t)* J_r_ai;
               h3.block(0, 0, 0, 0) = J_ai* _e2;
               *H3 = h3;
            }
      }

      return err;
   }


   gtsam::Vector VeCBFPdFactorCylinder1::evaluateError(const gtsam::Pose3 &pi, const gtsam::Vector3 &vi, const gtsam::Vector4 &ui,
                                            boost::optional<gtsam::Matrix &> H1, boost::optional<gtsam::Matrix &> H2, 
                                            boost::optional<gtsam::Matrix &> H3) const
   {
      double epsilon = 1e-6;

      gtsam::Matrix36 jac_ti;
      gtsam::Vector3 p_p0 = (_E12 * pi.translation(jac_ti) - obs_);
      double d = p_p0.norm();
      double d_eps = std::max(d, epsilon);
      gtsam::Matrix13 p_p0_t = p_p0.transpose();
      gtsam::Vector3 _vi = _E12 * vi;
      gtsam::Vector3 v_diff = _vi - obs_vel_;
      double p_dot_vdiff = p_p0.dot(v_diff);

      gtsam::Matrix13 H_ti = evaluateH_ti(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff);
      gtsam::Matrix13 H_vi = evaluateH_vi(p_p0, d_eps, p_p0_t);

      gtsam::Vector3 _e2(0., 0., 1.);  // Note: Assumes z-axis thrust; ui(1:3) unused.
      gtsam::Vector3 _g(0., 0., 9.81);
      gtsam::Vector3 _ai = _e2 * ui(0);

      gtsam::Matrix36 J_ri;
      gtsam::Matrix3 J_r_ri, J_r_ai;
      gtsam::Vector3 R_ai = pi.rotation(J_ri).rotate(_ai, J_r_ri, J_r_ai);
      gtsam::Vector3 term3_vec = -_g + R_ai;  // a_i

      gtsam::Vector1 hi = gtsam::Vector1(1.0 / safe_d_ - 1.0 / d_eps) + beta_  / d_eps * p_p0_t * v_diff;
      gtsam::Vector1 x_dot_0_term = H_ti * _vi;
      gtsam::Vector1 x_dot_2_term = H_vi * term3_vec;
      gtsam::Vector1 err = hi + alpha_ * x_dot_0_term + alpha_ * x_dot_2_term;

      if (err(0) > 0.0)
      {
         err(0) = 0.0;
         if (H1) *H1 = gtsam::Matrix::Zero(1, 6);
         if (H2) *H2 = gtsam::Matrix::Zero(1, 3);
         if (H3) *H3 = gtsam::Matrix::Zero(1, 4);
      }
      else
      {
         if (H1) 
         {
               gtsam::Matrix13 H_ti_err = evaluateH_ti_err(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff, term3_vec, H_ti);
               *H1 = H_ti_err * _E12 * jac_ti + alpha_ * H_vi * J_r_ri * J_ri;
         }

         if (H2)
         {
               gtsam::Matrix13 H_vi_err = evaluateH_vi_err(p_p0, d_eps, p_p0_t, v_diff, p_dot_vdiff, H_ti, H_vi) * _E12;
               *H2 = H_vi_err;
         }
         
         if (H3)
         {
               gtsam::Matrix14 h3 = gtsam::Matrix14::Zero();
               gtsam::Matrix13 J_ai = alpha_ * H_vi * J_r_ai;
               h3.block<1, 1>(0, 0) = J_ai * _e2;  // Scalar, but matrix form
               *H3 = h3;
         }
      }

      return err;
   }
    
}