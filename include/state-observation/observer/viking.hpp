/**
 * \file      viking.hpp
 * \author    Arnaud Demont, Mehdi Benallegue, Abdelaziz Benallegue
 * \date       2025
 *
 * \details
 *
 *
 */

#ifndef VikingHPP
#define VikingHPP

#include "state-observation/observer/zero-delay-observer.hpp"
#include <state-observation/observer/delayed-measurements-complem-filter.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace stateObservation
{
class STATE_OBSERVATION_DLLAPI Viking : public ZeroDelayObserver
{
public:
  /// Constructor with gains and sampling time (default sizes n=m=21)
  Viking(double alpha, double beta, double gamma, double mu, double rho, double dt);

  /// Constructor that allows to initialize the estimator's parameters afterwards. Handle with care.
  Viking();

protected:
  /// Internal constructor for custom state/measurement sizes.
  Viking(double alpha, double beta, double gamma, double mu, double rho, int n, int m, double dt);

public:
  /// Initialize full state vector directly
  void initEstimator(Vector & x);

  /// Initialize structured components
  /// x1: IMU local linear velocity
  /// x2: tilt vector (R^T e_z)
  /// b : gyro bias
  /// R : body orientation
  /// p_l: landmark position (in IMU/local frame)
  void initEstimator(const Vector3 & x1, const Vector3 & x2, const Vector3 & b, const Matrix3 & R, const Vector3 & p_l);

  /// Set measurement: yv, ya, yg, R_y, p_l,y at time k
  void setMeasurement(const Vector3 & yv_k,
                      const Vector3 & ya_k,
                      const Vector3 & yg_k,
                      const Matrix3 & Ry_k,
                      const Vector3 & p_l_y_k,
                      TimeIndex k);

  // --- Parameter setters / getters ---
  void setAlpha(double v)
  {
    alpha_ = v;
  }
  double getAlpha() const
  {
    return alpha_;
  }

  void setBeta(double v)
  {
    beta_ = v;
  }
  double getBeta() const
  {
    return beta_;
  }

  void setGamma(double v)
  {
    gamma_ = v;
  }
  double getGamma() const
  {
    return gamma_;
  }

  void setMu(double v)
  {
    mu_ = v;
  }
  double getMu() const
  {
    return mu_;
  }

  void setRho(double v)
  {
    rho_ = v;
  }
  double getRho() const
  {
    return rho_;
  }

  void setSamplingTime(double dt)
  {
    dt_ = dt;
  }
  double getSamplingTime() const
  {
    return dt_;
  }
  inline const kine::Orientation & getEstOrientation()
  {
    return R_hat_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  virtual StateVector oneStepEstimation_();

protected:
  // Parameters
  double alpha_, beta_, gamma_, mu_, rho_;
  double dt_; // sampling time

  // Cached variables for readability
  Vector3 x1_hat_, x2_hat_, b_hat_, p_l_hat_;
  kine::Orientation R_hat_;
};

} // namespace stateObservation

#endif // VikingHPP
