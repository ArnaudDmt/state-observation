

#ifndef TiltVisualHumanoidHPP
#define TiltVisualHumanoidHPP

#include "state-observation/observer/tilt-estimator-humanoid.hpp"
#include "state-observation/tools/rigid-body-kinematics.hpp"
#include <state-observation/api.h>
#include <state-observation/observer/tilt-estimator.hpp>
#include <state-observation/observer/zero-delay-observer.hpp>

namespace stateObservation
{

/**
 * \class  TiltVisualHumanoid
 * \brief  Version of the Tilt Estimator for humanoid robots.
 *
 */
class STATE_OBSERVATION_DLLAPI TiltVisualHumanoid : public TiltEstimatorHumanoid
{
  typedef kine::Orientation Orientation;

public:
  /// The constructor
  ///  \li alpha : parameter related to the convergence of the linear velocity
  ///              of the IMU expressed in the control frame
  ///  \li beta  : parameter related to the fast convergence of the tilt
  TiltVisualHumanoid(double alpha, double beta, double dt);

  /// @brief initializes the state vector.
  /// @param x1 The initial local linear velocity of the IMU.
  /// @param x2_p The initial value of the intermediate estimate of the IMU's tilt.
  /// @param x2 The initial tilt of the IMU.
  void initEstimator(Vector3 x1 = Vector3::Zero(),
                     Vector3 x2_prime = Vector3::UnitZ(),
                     Vector4 R = Vector4(0, 0, 0, 1));

  Vector3 getVirtualLocalVelocityMeasurement()
  {
    return x1_;
  }

/// prevent c++ overloaded virtual function warning
#if defined(__clang__)
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Woverloaded-virtual"
#else
#  if defined(__GNUC__)
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Woverloaded-virtual"
#  endif
#endif

  /// set rho1
  void setRho1(const double rho1)
  {
    rho1_ = rho1;
  }
  double getRho1() const
  {
    return rho1_;
  }

  /// set rho1
  void setRho2(const double rho2)
  {
    rho2_ = rho2;
  }
  double getRho2() const
  {
    return rho2_;
  }

  /// set mu
  void setMu(const double mu)
  {
    mu_ = mu;
  }
  double getMu() const
  {
    return mu_;
  }

  inline stateObservation::Vector3 getSigmaPart1() const
  {
    return sigma_part1_;
  }
  inline stateObservation::Vector3 getSigmaPart2() const
  {
    return sigma_part2_;
  }
  inline stateObservation::Vector3 getSigmaPart3() const
  {
    return sigma_part3_;
  }

  /// sets ths measurement (accelero and gyro stacked in one vector)
  void setMeasurement(const Vector3 & imuControlPos,
                      const Vector3 & imuControlLinVel,
                      const Vector3 & ya_k,
                      const Vector3 & yg_k,
                      const Vector4 & yR_k,
                      TimeIndex k);

  /// sets ths measurement (accelero and gyro stacked in one vector)
  void setMeasurement(const Vector3 & yv_k,
                      const Vector3 & ya_k,
                      const Vector3 & yg_k,
                      const Vector4 & yR_k,
                      TimeIndex k);

#if defined(__clang__)
#  pragma clang diagnostic pop
#else
#  if defined(__GNUC__)
#    pragma GCC diagnostic pop
#  endif
#endif

public:
protected:
  double rho1_ = 0.0;
  double rho2_ = 0.0;
  double mu_ = 0.0;

  Vector3 sigma_part1_;
  Vector3 sigma_part2_;
  Vector3 sigma_part3_;

  /// Estimated orientation of the IMU
  kine::Orientation R_hat_;

  /// Orientation estimator loop
  StateVector oneStepEstimation_();
};

} // namespace stateObservation

#endif // TiltVisualHumanoidHPP
