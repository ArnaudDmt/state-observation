/**
 * \file      tilt-estimator-humanoid.hpp
 * \author    Arnaud Demont, Mehdi Benallegue
 * \date       2024
 * \brief      Version of the Tilt Estimator that implements all the necessary functions to perform the estimation for
 * humanoid robots.
 *
 * \details
 *
 *
 */

#ifndef TILTESTIMATORHUMANOIDHPP
#define TILTESTIMATORHUMANOIDHPP

#include "state-observation/tools/rigid-body-kinematics.hpp"
#include <state-observation/api.h>
#include <state-observation/observer/tilt-estimator.hpp>
#include <state-observation/observer/zero-delay-observer.hpp>

namespace stateObservation
{

/**
 * \class  TiltEstimatorHumanoid
 * \brief  Version of the Tilt Estimator for humanoid robots.
 *
 */
class STATE_OBSERVATION_DLLAPI TiltEstimatorHumanoid : public TiltEstimator
{
public:
  /// The constructor
  ///  \li alpha : parameter related to the convergence of the linear velocity
  ///              of the IMU expressed in the control frame
  ///  \li beta  : parameter related to the fast convergence of the tilt
  ///  \li gamma : parameter related to the orthogonality
  TiltEstimatorHumanoid(double alpha, double beta, double gamma, double dt);

  /// Constructor that allows to initialize the estimator's parameters afterwards. Handle with care.
  TiltEstimatorHumanoid();

protected:
  // constructor that allows to use custom sizes for the state and measurement vectors. Might be useful for other
  // estimators inheriting from this one.
  TiltEstimatorHumanoid(double alpha, double beta, double gamma, int n, int m, double dt);

public:
  /// @brief Resets x1hat (the estimate of the local linear velocity of the IMU in the world)
  /// @details Avoid discontinuities when the computation mode of the anchor point changes
  void resetImuLocVelHat();

  // we also want to use the function setMeasurement from the TiltEstimator class, that is hidden by the following
  using TiltEstimator::setMeasurement;
  /// @brief sets the measurement (accelero and gyro stacked in one vector)
  /// @details the local linear velocity of the IMU in the world is obtained from \p imuControlPos, \p imuControlLinVel
  /// and \p yg_k. The control frame is a frame whose velocity is considered zero in the world frame.
  /// @param imuControlPos position of the IMU frame in the control frame
  /// @param imuControlLinVel linear velocity of the IMU frame in the control frame
  /// @param ya_k accelerometer measurement
  /// @param yg_k gyrometer measurement
  void setMeasurement(const Vector3 & imuControlPos,
                      const Vector3 & imuControlLinVel,
                      const Vector3 & ya_k,
                      const Vector3 & yg_k,
                      TimeIndex k);

protected:
  Vector3 x1_;
};
} // namespace stateObservation

#endif // TILTESTIMATORHUMANOIDHPP
