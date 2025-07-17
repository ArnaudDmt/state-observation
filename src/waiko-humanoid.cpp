#include "state-observation/tools/rigid-body-kinematics.hpp"
#include <state-observation/observer/waiko-humanoid.hpp>
#include <state-observation/tools/definitions.hpp>
namespace stateObservation
{
WaikoHumanoid::WaikoHumanoid(double dt, double alpha, double beta, double rho, double lambda, double mu)
: ZeroDelayObserver(9, 0, std::make_shared<IndexedInputArrayT<InputWaiko>>()), alpha_(alpha), beta_(beta), rho_(rho),
  lambda_(lambda), mu_(mu), dt_(dt)
{
  dx_hat_.resize(12);
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
  Eigen::VectorBlock<const ObserverBase::StateVector, sizePos> pl_hat = x_hat.segment<sizePos>(posIndex);

  // we fetch the input from the previous iteration
  const InputWaiko & input = convert_input<InputWaiko>(getInput(k));
  const Vector3 & yv = input.yv_;
  const Vector3 & ya = input.ya_;
  const Vector3 & yg = input.yg_;

  // we compute the state dynamics
  Eigen::VectorBlock<Vector, sizeX1> x1_hat_dot = dx_hat_.segment<sizeX1Tangent>(x1IndexTangent);
  Eigen::VectorBlock<Vector, sizeX2Tangent> x2_hat_dot = dx_hat_.segment<sizeX2Tangent>(x2IndexTangent);
  Eigen::VectorBlock<Vector, sizeOriTangent> w_l =
      dx_hat_.segment<sizeOriTangent>(oriIndexTangent); // using R_dot = RS(w_l)
  Eigen::VectorBlock<Vector, sizePosTangent> v_l = dx_hat_.segment<sizePosTangent>(posIndexTangent);

  x1_hat_dot = x1_hat.cross(yg) - cst::gravityConstant * x2_hat + ya + alpha_ * (yv - x1_hat); // x1
  x2_hat_dot = x2_hat.cross(yg) - beta_ * (yv - x1_hat); // x2
  // using R_dot = RS(w_l) and w_l = yg - gamma * S(R_hat^T ez) x2_hat
  w_l = yg + rho_ * x2_hat.cross(state_ori_.toMatrix3().transpose() * Vector3::UnitZ());
  // using pl_dot = -S(yg) pl + x1
  v_l = x1_hat + pl_hat.cross(yg);

  return dx_hat_;
}

void WaikoHumanoid::addCorrectionTerms()
{
  oriCorrFromOriMeas_.setZero();
  oriCorrFromContactPos_.setZero();
  posCorrFromContactPos_.setZero();

  TimeIndex k = this->x_.getTime();
  BOOST_ASSERT(u_ && u_->checkIndex(k) && "ERROR: The input is not set");

  // we fetch the estimated state from the previous iteration
  const ObserverBase::StateVector & x_hat = getCurrentEstimatedState();
  const InputWaiko & input = convert_input<InputWaiko>(getInput(k));

  Eigen::VectorBlock<const ObserverBase::StateVector, sizePos> pl_hat = x_hat.segment<sizePos>(posIndex);

  // we add the correction terms compute the state dynamics
  Eigen::Ref<Vector3> x1_hat_dot = dx_hat_.segment<sizeX1Tangent>(x1IndexTangent);
  Eigen::Ref<Vector3> x2_hat_dot = dx_hat_.segment<sizeX2Tangent>(x2IndexTangent);
  Eigen::Ref<Vector3> w_l = dx_hat_.segment<sizeOriTangent>(oriIndexTangent); // using R_dot = RS(w_l * dt)
  Eigen::Ref<Vector3> v_l = dx_hat_.segment<sizePosTangent>(posIndexTangent);

  for(const InputWaiko::ContactInput & contactInput : input.contact_inputs_)
  {
    Vector3 meas_pl = contactInput.ori_.transpose() * contactInput.pos_;
    Vector3 meas_tilt = contactInput.ori_.transpose() * Vector3::UnitZ();
    Matrix3 R_tilde = contactInput.ori_ * state_ori_.toMatrix3().transpose();
    Vector3 R_tilde_vec = kine::skewSymmetricToRotationVector(R_tilde - R_tilde.transpose()) / 2.0;

    oriCorrFromOriMeas_ +=
        mu_ * state_ori_.toMatrix3().transpose() * Vector3::UnitZ() * Vector3::UnitZ().transpose() * R_tilde_vec;

    posCorrFromContactPos_ += lambda_ * (meas_pl - pl_hat);
  }

  w_l += oriCorrFromOriMeas_ + oriCorrFromContactPos_;
  v_l += posCorrFromContactPos_;
}

void WaikoHumanoid::integrateState_()
{
  ObserverBase::StateVector & x_hat = getCurrentEstimatedState();

  Eigen::VectorBlock<ObserverBase::StateVector, sizeX1> x1_hat = x_hat.segment<sizeX1>(x1Index);
  Eigen::VectorBlock<ObserverBase::StateVector, sizeX2> x2_hat = x_hat.segment<sizeX2>(x2Index);
  Eigen::VectorBlock<ObserverBase::StateVector, sizePos> pl_hat = x_hat.segment<sizePos>(posIndex);

  // we add the correction terms compute the state dynamics
  const auto & x1_hat_dot = dx_hat_.segment<sizeX1Tangent>(x1IndexTangent);
  const auto & x2_hat_dot = dx_hat_.segment<sizeX2Tangent>(x2IndexTangent);
  const auto & w_l = dx_hat_.segment<sizeOriTangent>(oriIndexTangent);
  const auto & v_l = dx_hat_.segment<sizePosTangent>(posIndexTangent);

  // discrete-time integration of x1_hat, x2_hat and b_hat
  x1_hat += x1_hat_dot * dt_;
  x2_hat += x2_hat_dot * dt_;
  pl_hat += v_l * dt_;

  // discrete-time integration of R
  state_ori_.integrateRightSide(w_l * dt_);

  setState(x_hat, getCurrentTime() + 1);
}

void WaikoHumanoid::initEstimator(const Vector3 & x1, const Vector3 & x2, const Vector4 & ori, const Vector3 & pos)
{
  Eigen::VectorXd initStateVector = Eigen::VectorXd::Zero(getStateSize());

  initStateVector.segment<sizeX1>(x1Index) = x1;
  initStateVector.segment<sizeX2>(x2Index) = x2;
  state_ori_.fromVector4(ori);
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
