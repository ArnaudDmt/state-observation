#include <state-observation/observer/tilt-estimator.hpp>

namespace stateObservation
{

TiltEstimator::TiltEstimator() : ZeroDelayObserver(9, 9, std::make_shared<IndexedInputArrayT<>>()) {}

TiltEstimator::TiltEstimator(double alpha, double beta, double gamma, double dt)
: TiltEstimator(alpha, beta, gamma, 9, 9, dt)
{
}

TiltEstimator::TiltEstimator(double alpha, double beta, double gamma, int n, int m, double dt)
: ZeroDelayObserver(n, m, std::make_shared<IndexedInputArrayT<>>()), alpha_(alpha), beta_(beta), gamma_(gamma), dt_(dt)
{
}

void TiltEstimator::initEstimator(Vector & x)
{
  setState(x, 0);
}

void TiltEstimator::initEstimator(Vector3 & x1, Vector3 & x2_prime, Vector3 & x2)
{
  Eigen::VectorXd initStateVector = Eigen::VectorXd::Zero(getStateSize());

  initStateVector.segment<3>(0) = x1;
  initStateVector.segment<3>(3) = x2_prime;
  initStateVector.segment<3>(6) = x2;

  setState(initStateVector, 0);
}

void TiltEstimator::setMeasurement(const Vector3 & yv_k, const Vector3 & ya_k, const Vector3 & yg_k, TimeIndex k)
{
  ObserverBase::MeasureVector y_k(9);
  y_k << yv_k, ya_k, yg_k;

  ZeroDelayObserver::setMeasurement(y_k, k);
}

ObserverBase::StateVector TiltEstimator::oneStepEstimation_()
{
  TimeIndex k = this->x_.getTime();

  BOOST_ASSERT(this->y_.size() > 0 && this->y_.checkIndex(k + 1) && "ERROR: The measurement vector is not set");

  Vector3 yv = getMeasurement(k + 1).head<3>();
  Vector3 ya = getMeasurement(k + 1).segment<3>(3);
  Vector3 yg = getMeasurement(k + 1).tail<3>();

  ObserverBase::StateVector x_hat = getCurrentEstimatedState();
  x1_hat_ = x_hat.segment<3>(0);
  x2_hat_prime_ = x_hat.segment<3>(3);
  x2_hat_ = x_hat.segment<3>(6);

  Vector dx_hat(9);
  dx_hat.segment<3>(0) = x1_hat_.cross(yg) - cst::gravityConstant * x2_hat_prime_ + ya + alpha_ * (yv - x1_hat_);
  dx_hat.segment<3>(3) = x2_hat_prime_.cross(yg) - beta_ * (yv - x1_hat_);
  dx_hat.segment<3>(6) = x2_hat_.cross(yg - gamma_ * x2_hat_.cross(x2_hat_prime_));

  x_hat += dx_hat * dt_;

  x_hat.tail<3>() /= x_hat.tail<3>().norm();

  setState(x_hat, k + 1);

  return x_hat;
}

} // namespace stateObservation
