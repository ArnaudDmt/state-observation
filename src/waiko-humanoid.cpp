#include "state-observation/tools/rigid-body-kinematics.hpp"
#include <state-observation/observer/waiko-humanoid.hpp>
#include <state-observation/tools/definitions.hpp>
namespace stateObservation
{

WaikoHumanoid::WaikoHumanoid(double dt, double alpha, double beta, double gamma, double rho, bool withGyroBias)
: ZeroDelayObserver(16, 0, std::make_shared<IndexedInputArrayT<InputWaiko>>()), alpha_(alpha), beta_(beta),
  gamma_(gamma), rho_(rho), withGyroBias_(withGyroBias), dt_(dt)
{
  dx_hat_.resize(15);
}

WaikoHumanoid::~WaikoHumanoid() {}

void WaikoHumanoid::setInput(const Vector3 & imuAnchorPos,
                             const Vector3 & imuAnchorLinVel,
                             const Vector3 & ya_k,
                             const Vector3 & yg_k,
                             TimeIndex k,
                             bool resetImuLocVelHat)
{
  Vector3 yv = -yg_k.cross(imuAnchorPos) - imuAnchorLinVel;

  WaikoHumanoid::setInput(yv, ya_k, yg_k, k, resetImuLocVelHat);
}

void WaikoHumanoid::setInput(const Vector3 & yv_k,
                             const Vector3 & ya_k,
                             const Vector3 & yg_k,
                             TimeIndex k,
                             bool resetImuLocVelHat)
{
  setInput(InputWaiko(yv_k, ya_k, yg_k), k);

  if(resetImuLocVelHat)
  {
    x_().segment<3>(0) = yv_k;
  }
}

void WaikoHumanoid::addContactInput(const InputWaiko::ContactInput & contactInput, TimeIndex k)
{
  InputWaiko & input = convert_input<InputWaiko>(getInput(k));
  input.contact_inputs_.push_back(contactInput);
}

void WaikoHumanoid::startNewIteration_() {}

ObserverBase::StateVector & WaikoHumanoid::computeStateDynamics_()
{
  dx_hat_.setZero();
  TimeIndex k = this->x_.getTime();
  BOOST_ASSERT(u_ && u_->checkIndex(k) && "ERROR: The input is not set");

  // we fetch the estimated state from the previous iteration
  const ObserverBase::StateVector & x_hat = getCurrentEstimatedState();
  Eigen::VectorBlock<const ObserverBase::StateVector, sizeX1> x1_hat = x_hat.segment<sizeX1>(x1Index);
  Eigen::VectorBlock<const ObserverBase::StateVector, sizeX2> x2_hat = x_hat.segment<sizeX2>(x2Index);
  Eigen::VectorBlock<const ObserverBase::StateVector, sizeGyroBias> b_hat = x_hat.segment<sizeGyroBias>(gyroBiasIndex);
  Eigen::VectorBlock<const ObserverBase::StateVector, sizeOri> q_hat = x_hat.segment<sizeOri>(oriIndex);
  Eigen::VectorBlock<const ObserverBase::StateVector, sizePos> pl_hat = x_hat.segment<sizePos>(posIndex);

  state_kine_.position = pl_hat;
  state_kine_.orientation.fromVector4(q_hat);

  // we fetch the input from the previous iteration
  const InputWaiko & input = convert_input<InputWaiko>(getInput(k));
  const Vector3 & yv = input.yv_;
  const Vector3 & ya = input.ya_;
  const Vector3 & yg = input.yg_;

  Vector3 unbiased_yg = yg;
  if(withGyroBias_)
  {
    unbiased_yg -= b_hat;
  }

  // we compute the state dynamics
  Eigen::VectorBlock<Vector, sizeX1> x1_hat_dot = dx_hat_.segment<sizeX1Tangent>(x1IndexTangent);
  Eigen::VectorBlock<Vector, sizeX2Tangent> x2_hat_dot = dx_hat_.segment<sizeX2Tangent>(x2IndexTangent);
  Eigen::VectorBlock<Vector, sizeGyroBiasTangent> b_hat_dot =
      dx_hat_.segment<sizeGyroBiasTangent>(gyroBiasIndexTangent);
  Eigen::VectorBlock<Vector, sizeOriTangent> w_l =
      dx_hat_.segment<sizeOriTangent>(oriIndexTangent); // using R_dot = RS(w_l)
  Eigen::VectorBlock<Vector, sizePosTangent> v_l = dx_hat_.segment<sizePosTangent>(posIndexTangent);

  x1_hat_dot = x1_hat.cross(unbiased_yg) - cst::gravityConstant * x2_hat + ya + alpha_ * (yv - x1_hat); // x1
  x2_hat_dot = x2_hat.cross(unbiased_yg) - beta_ / cst::gravityConstant * (yv - x1_hat); // x2
  b_hat_dot = rho_ * x1_hat.cross(yv); // using b_dot = rho * S(x1_hat) * yv
  // using R_dot = RS(w_l) and w_l = yg - gamma * S(R_hat^T ez) x2_hat
  w_l = unbiased_yg + gamma_ * x2_hat.cross(state_kine_.orientation.toMatrix3().transpose() * Vector3::UnitZ());
  // using pl_dot = -S(yg) pl + x1
  v_l = x1_hat + pl_hat.cross(unbiased_yg);

  for(const InputWaiko::ContactInput & contactInput : input.contact_inputs_)
  {
    Vector3 meas_pl = contactInput.ori_.transpose() * contactInput.pos_;
    Vector3 meas_tilt = contactInput.ori_.transpose() * Vector3::UnitZ();
    Matrix3 R_tilde = contactInput.ori_ * state_kine_.orientation.toMatrix3().transpose();
    Vector3 R_tilde_vec = kine::skewSymmetricToRotationVector(R_tilde - R_tilde.transpose()) / 2.0;

    x1_hat_dot += contactInput.mu_ * (meas_pl - pl_hat);
    x2_hat_dot += contactInput.tau_ * (meas_tilt - x2_hat);
    if(withGyroBias_)
    {
      // b_hat_dot = rho * S(x1_hat) * yv + g0 * (rho / beta) * S(x2_hat)Ry^T ez - g0/4 * rho * tau * min(gamma, lambda)
      // / beta * R_hat^T vec(Pa(R_tilde)) + rho * mu * S(pl_hat) Ry^T py
      b_hat_dot += cst::gravityConstant * rho_ / beta_ * x2_hat.cross(meas_tilt)
                   - cst::gravityConstant / 4.0 * rho_ * contactInput.tau_ * std::min(gamma_, contactInput.lambda_)
                         / beta_ * state_kine_.orientation.toMatrix3().transpose() * R_tilde_vec
                   + rho_ * contactInput.mu_ * pl_hat.cross(meas_pl);
    }
    w_l += contactInput.lambda_ * state_kine_.orientation.toMatrix3().transpose() * Vector3::UnitZ()
           * Vector3::UnitZ().transpose() * R_tilde_vec;
    v_l += contactInput.eta_ * (meas_pl - pl_hat);
  }

  return dx_hat_;
}

void WaikoHumanoid::addCorrectionTerms()
{
  TimeIndex k = this->x_.getTime();
  BOOST_ASSERT(u_ && u_->checkIndex(k) && "ERROR: The input is not set");

  // we fetch the estimated state from the previous iteration
  const ObserverBase::StateVector & x_hat = getCurrentEstimatedState();
  const InputWaiko & input = convert_input<InputWaiko>(getInput(k));

  Eigen::VectorBlock<const ObserverBase::StateVector, sizeX2> x2_hat = x_hat.segment<sizeX2>(x2Index);
  Eigen::VectorBlock<const ObserverBase::StateVector, sizePos> pl_hat = x_hat.segment<sizePos>(posIndex);

  // we add the correction terms compute the state dynamics
  Eigen::Ref<Vector3> x1_hat_dot = dx_hat_.segment<sizeX1Tangent>(x1IndexTangent);
  Eigen::Ref<Vector3> x2_hat_dot = dx_hat_.segment<sizeX2Tangent>(x2IndexTangent);
  Eigen::Ref<Vector3> b_hat_dot = dx_hat_.segment<sizeGyroBiasTangent>(gyroBiasIndexTangent);
  Eigen::Ref<Vector3> w_l = dx_hat_.segment<sizeOriTangent>(oriIndexTangent); // using R_dot = RS(w_l * dt)
  Eigen::Ref<Vector3> v_l = dx_hat_.segment<sizePosTangent>(posIndexTangent);

  for(const InputWaiko::ContactInput & contactInput : input.contact_inputs_)
  {
    Vector3 meas_pl = contactInput.ori_.transpose() * contactInput.pos_;
    Vector3 meas_tilt = contactInput.ori_.transpose() * Vector3::UnitZ();
    Matrix3 R_tilde = contactInput.ori_ * state_kine_.orientation.toMatrix3().transpose();
    Vector3 R_tilde_vec = kine::skewSymmetricToRotationVector(R_tilde - R_tilde.transpose()) / 2.0;

    x1_hat_dot += contactInput.mu_ * (meas_pl - pl_hat);
    x2_hat_dot += contactInput.tau_ * (meas_tilt - x2_hat);
    if(withGyroBias_)
    {
      // b_hat_dot = rho * S(x1_hat) * yv + g0 * (rho / beta) * S(x2_hat)Ry^T ez - g0/4 * rho * tau * min(gamma, lambda)
      // / beta * R_hat^T vec(Pa(R_tilde)) + rho * mu * S(pl_hat) Ry^T py
      b_hat_dot += cst::gravityConstant * rho_ / beta_ * x2_hat.cross(meas_tilt)
                   - cst::gravityConstant / 4.0 * rho_ * contactInput.tau_ * std::min(gamma_, contactInput.lambda_)
                         / beta_ * state_kine_.orientation.toMatrix3().transpose() * R_tilde_vec
                   + rho_ * contactInput.mu_ * pl_hat.cross(meas_pl);
    }
    w_l += contactInput.lambda_ * state_kine_.orientation.toMatrix3().transpose() * Vector3::UnitZ()
           * Vector3::UnitZ().transpose() * R_tilde_vec;
    v_l += contactInput.eta_ * (meas_pl - pl_hat);
  }
}

void WaikoHumanoid::integrateState_()
{
  ObserverBase::StateVector & x_hat = getCurrentEstimatedState();

  Eigen::VectorBlock<ObserverBase::StateVector, sizeX1> x1_hat = x_hat.segment<sizeX1>(x1Index);
  Eigen::VectorBlock<ObserverBase::StateVector, sizeX2> x2_hat = x_hat.segment<sizeX2>(x2Index);
  Eigen::VectorBlock<ObserverBase::StateVector, sizeGyroBias> b_hat = x_hat.segment<sizeGyroBias>(gyroBiasIndex);
  Eigen::VectorBlock<ObserverBase::StateVector, sizePos> pl_hat = x_hat.segment<sizePos>(posIndex);

  // we add the correction terms compute the state dynamics
  const auto & x1_hat_dot = dx_hat_.segment<sizeX1Tangent>(x1IndexTangent);
  const auto & x2_hat_dot = dx_hat_.segment<sizeX2Tangent>(x2IndexTangent);
  const auto & b_hat_dot = dx_hat_.segment<sizeGyroBiasTangent>(gyroBiasIndexTangent);
  const auto & w_l = dx_hat_.segment<sizeOriTangent>(oriIndexTangent);
  const auto & v_l = dx_hat_.segment<sizePosTangent>(posIndexTangent);

  // discrete-time integration of x1_hat, x2_hat and b_hat
  x1_hat += x1_hat_dot * dt_;
  x2_hat += x2_hat_dot * dt_;
  b_hat += b_hat_dot * dt_;
  pl_hat += v_l * dt_;

  // discrete-time integration of p and R
  state_kine_.linVel = v_l;
  state_kine_.angVel = w_l;

  state_kine_.orientation.integrateRightSide(w_l * dt_);
  state_kine_.position = pl_hat;

  x_hat.segment<sizeOri>(oriIndex) = state_kine_.orientation.toVector4();
  x_hat.segment<sizePos>(posIndex) = state_kine_.position();

  setState(x_hat, getCurrentTime() + 1);
}

void WaikoHumanoid::initEstimator(const Vector3 & x1,
                                  const Vector3 & x2,
                                  const Vector3 & gyro_bias,
                                  const Vector4 & ori,
                                  const Vector3 & pos)
{
  Eigen::VectorXd initStateVector = Eigen::VectorXd::Zero(getStateSize());

  initStateVector.segment<sizeX1>(x1Index) = x1;
  initStateVector.segment<sizeX2>(x2Index) = x2;
  initStateVector.segment<sizeGyroBias>(gyroBiasIndex) = gyro_bias;
  initStateVector.segment<sizeOri>(oriIndex) = ori;
  initStateVector.segment<sizePos>(posIndex) = pos;

  setState(initStateVector, 0);
}

ObserverBase::StateVector WaikoHumanoid::oneStepEstimation_()
{
  computeStateDynamics_();
  addCorrectionTerms();
  integrateState_();
  return getCurrentEstimatedState();
}

} // namespace stateObservation
