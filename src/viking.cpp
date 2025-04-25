#include <state-observation/observer/viking.hpp>
#include <state-observation/tools/definitions.hpp>

namespace stateObservation
{

Viking::Viking(double dt, double alpha, double beta, double rho, unsigned long bufferCapacity)
: DelayedMeasurementComplemFilter(dt,
                                  16,
                                  15,
                                  9,
                                  bufferCapacity,
                                  std::make_shared<IndexedInputArrayT<VectorInput>>(),
                                  std::make_shared<AsynchronousDataMapT<AsynchronousInputViking>>())
{
  setAlpha(alpha);
  setBeta(beta);
  setRho(rho);
}

Viking::~Viking() {}

void Viking::setMeasurement(const Vector3 & yv_k,
                            const Vector3 & ya_k,
                            const Vector3 & yg_k,
                            TimeIndex k,
                            bool resetImuLocVelHat)
{
  ObserverBase::MeasureVector y_k(getMeasureSize());
  y_k << yv_k, ya_k, yg_k;

  setMeasurement(y_k, k);

  if(resetImuLocVelHat)
  {
    xBuffer_.front()().segment<3>(0) = yv_k;
  }
}

void Viking::addCorrectionTerms(StateIterator it)
{
  // we fetch the state and input from the previous iteration
  StateIterator prevIter = it + 1;
  if(!u_asynchronous_->checkIndex(prevIter->getTime()))
  {
    return;
  }

  // we fetch the estimated state from the previous iteration
  ObserverBase::StateVector & x_hat = (*prevIter)();
  Eigen::Ref<Vector3> x2_hat = x_hat.segment<3>(3);
  Eigen::Ref<Vector3> pl_hat = x_hat.tail(3);

  // we add the correction terms compute the state dynamics
  Eigen::Ref<Vector3> x1_hat_dot = dx_hat_.segment<3>(0);
  Eigen::Ref<Vector3> x2_hat_dot = dx_hat_.segment<3>(3);
  Eigen::Ref<Vector3> b_hat_dot = dx_hat_.segment<3>(6);
  Eigen::Ref<Vector3> w_l = dx_hat_.segment<3>(9);
  Eigen::Ref<Vector3> v_l = dx_hat_.segment<3>(12);

  AsynchronousInputViking & async_input =
      convert_async_data<AsynchronousInputViking>(u_asynchronous_->getElement(prevIter->getTime()));

  for(auto & [posMeas, oriMeas, mu, lambda] : async_input.pos_ori_measurements_)
  {
    Vector3 meas_pl = oriMeas.transpose() * posMeas;
    Vector3 meas_tilt = oriMeas.transpose() * Vector3::UnitZ();
    Matrix3 R_tilde = oriMeas * state_kine_.orientation.toMatrix3();
    Vector3 R_tilde_vec = kine::skewSymmetricToRotationVector(R_tilde - R_tilde.transpose()) / 2.0;

    x1_hat_dot += pow(cst::gravityConstant, 2) / beta_ * (meas_pl - pl_hat);
    x2_hat_dot += gamma_ * (meas_tilt - x2_hat);
    b_hat_dot += x2_hat.cross(meas_tilt)
                 - gamma_ * std::min(rho_, mu) / (4 * pow(rho_, 2)) * state_kine_.orientation.toMatrix3().transpose()
                       * R_tilde_vec
                 + pl_hat.cross(meas_pl);
    w_l += mu * state_kine_.orientation.toMatrix3().transpose() * Vector3::UnitZ() * Vector3::UnitZ().transpose()
           * R_tilde_vec;
    v_l += lambda * (meas_pl - pl_hat);
  }
}

void Viking::startNewIteration_()
{
  u_asynchronous_->pushValue(AsynchronousInputViking(), getCurrentTime());
}

void Viking::addDelayedPosOriMeasurement(const Vector3 & pos,
                                         const Matrix3 & ori,
                                         double mu,
                                         double lambda,
                                         TimeIndex delay)
{
  startNewIteration_();
  TimeIndex measTime = getCurrentTime() - delay;

  u_asynchronous_->pushValue(AsynchronousInputViking(pos, ori, mu, lambda), measTime);
}

void Viking::addPosOriMeasurement(const Vector3 & pos, const Matrix3 & ori, double mu, double lambda)
{
  startNewIteration_();

  u_asynchronous_->pushValue(AsynchronousInputViking(pos, ori, mu, lambda), getCurrentTime());
}

ObserverBase::StateVector & Viking::computeStateDynamics_(StateIterator it)
{
  dx_hat_.setZero();
  StateIterator prevIter = it + 1;
  // we fetch the estimated state from the previous iteration
  ObserverBase::StateVector & x_hat = (*prevIter)();
  Eigen::Ref<Vector3> x1_hat = x_hat.segment<3>(0);
  Eigen::Ref<Vector3> x2_hat = x_hat.segment<3>(3);
  Eigen::Ref<Vector3> b_hat = x_hat.segment<3>(6);
  Eigen::Ref<Vector3> pl_hat = x_hat.tail(3);
  state_kine_.fromVector(x_hat.tail(7), kine::Kinematics::Flags::pose);

  // we fetch the input from the previous iteration
  VectorInput & synced_Input = convert_input<VectorInput>((*u_)[prevIter->getTime()]);
  Eigen::Ref<Vector3> yv = synced_Input.head<3>();
  Eigen::Ref<Vector3> ya = synced_Input.segment<3>(3);
  Eigen::Ref<Vector3> yg = synced_Input.segment<3>(6);

  Vector3 unbiased_yg = yg - b_hat;

  // we compute the state dynamics
  Eigen::Ref<Vector3> x1_hat_dot = dx_hat_.segment<3>(0);
  Eigen::Ref<Vector3> x2_hat_dot = dx_hat_.segment<3>(3);
  Eigen::Ref<Vector3> b_hat_dot = dx_hat_.segment<3>(6);
  Eigen::Ref<Vector3> w_l = dx_hat_.segment<3>(9); // using R_dot = RS(w_l * dt)
  Eigen::Ref<Vector3> v_l = dx_hat_.segment<3>(12);

  x1_hat_dot = x1_hat.cross(unbiased_yg) - cst::gravityConstant * x2_hat + ya + alpha_ * (yv - x1_hat); // x1
  x2_hat_dot = x2_hat.cross(unbiased_yg) - beta_ / cst::gravityConstant * (yv - x1_hat); // x2
  b_hat_dot = beta_ / pow(cst::gravityConstant, 2) * x1_hat.cross(yv); // using b_dot = gain * S(x1_hat) * yv
  // using R_dot = RS(w_l * dt) and w_l = yg-rho*S(R_hat^T ez))x2_hat
  w_l = unbiased_yg + rho_ * x2_hat.cross(state_kine_.orientation.toMatrix3().transpose() * Vector3::UnitZ());
  // using pl_dot = -S(yg) pl + v_l
  v_l = x1_hat + pl_hat.cross(unbiased_yg);

  return dx_hat_;
}

void Viking::integrateState_(StateIterator it)
{
  ObserverBase::StateVector & initState = (*(it + 1))();
  ObserverBase::StateVector & newState = (*(it))();

  Eigen::Ref<Vector3> x1_hat = initState.segment<3>(0);
  Eigen::Ref<Vector3> x2_hat = initState.segment<3>(3);
  Eigen::Ref<Vector3> b_hat = initState.segment<3>(6);

  // we add the correction terms compute the state dynamics
  const auto & x1_hat_dot = dx_hat_.segment<3>(0);
  const auto & x2_hat_dot = dx_hat_.segment<3>(3);
  const auto & b_hat_dot = dx_hat_.segment<3>(6);
  const auto & w_l = dx_hat_.segment<3>(9);
  const auto & v_l = dx_hat_.segment<3>(12);

  // discrete-time integration of x1_hat, x2_hat and b_hat
  x1_hat += x1_hat_dot * dt_;
  x2_hat += x2_hat_dot * dt_;
  b_hat += b_hat_dot * dt_;

  // discrete-time integration of p and R
  state_kine_.linVel = v_l;
  state_kine_.angVel = w_l;
  state_kine_.SE3_integration(dt_);

  newState.segment<4>(9) = state_kine_.orientation.toVector4();
  newState.tail(3) = state_kine_.position();
}

void Viking::initEstimator(const Vector3 & x1, const Vector3 & x2_prime, const Vector3 & pos, const Vector4 & R)
{
  Eigen::VectorXd initStateVector = Eigen::VectorXd::Zero(getStateSize());

  initStateVector.segment<3>(0) = x1;
  initStateVector.segment<3>(3) = x2_prime;
  initStateVector.segment<3>(6) = pos;
  initStateVector.tail(4) = R;

  initEstimator(initStateVector);
}

ObserverBase::StateVector Viking::oneStepEstimation_(StateIterator it)
{
  BOOST_ASSERT(this->y_.size() > 0 && this->y_.checkIndex(it->getTime()) && "ERROR: The measurement vector is not set");

  computeStateDynamics_(it);
  addCorrectionTerms(it);
  integrateState_(it);

  return (*(it))();
}

} // namespace stateObservation
