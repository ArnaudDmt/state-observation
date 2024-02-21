#include <state-observation/observer/tilt-estimator-humanoid.hpp>

namespace stateObservation
{
TiltEstimatorHumanoid::TiltEstimatorHumanoid(double alpha, double beta, double gamma)
: TiltEstimator(alpha, beta, gamma), p_S_C_(Vector3::Zero()), R_S_C_(Matrix3::Identity()), v_S_C_(Vector3::Zero()),
  w_S_C_(Vector3::Zero()), v_C_(Vector3::Zero())
{
}

TiltEstimatorHumanoid::TiltEstimatorHumanoid(double alpha, double beta, double gamma, int n, int m)
: TiltEstimator(alpha, beta, gamma, n, m), p_S_C_(Vector3::Zero()), R_S_C_(Matrix3::Identity()),
  v_S_C_(Vector3::Zero()), w_S_C_(Vector3::Zero()), v_C_(Vector3::Zero())
{
}

void TiltEstimatorHumanoid::setMeasurement(const Vector3 & ya_k, const Vector3 & yg_k, TimeIndex k)
{
  x1_ = R_S_C_.transpose() * (v_C_ + v_S_C_) + (yg_k - R_S_C_.transpose() * w_S_C_).cross(R_S_C_.transpose() * p_S_C_);

  TiltEstimator::setMeasurement(x1_, ya_k, yg_k, k);
}

void TiltEstimatorHumanoid::resetImuLocVelHat()
{
  x_().segment<3>(0) = x1_;
}

} // namespace stateObservation
