#include "state-observation/tools/definitions.hpp"
#include <state-observation/observer/viking.hpp>

namespace stateObservation
{

IterationViking::IterationViking(const Vector & initState, const kine::Kinematics & initPose, double dt)
: IterationComplementaryFilter(initState, dt), initPose_(initPose)
{
}

Vector & IterationViking::runIteration_()
{
  Vector dx_hat = computeStateDerivatives_();
  integrateState_(dx_hat);

  return finalState_;
}

Vector IterationViking::computeStateDerivatives_()
{
  const Vector3 & yv = y_.head<3>();
  const Vector3 & ya = y_.segment<3>(3);
  const Vector3 & yg = y_.segment<3>(6);

  const Eigen::Ref<Vector3> x1_hat = initState_.segment<3>(0);
  const Eigen::Ref<Vector3> x2_hat_prime = initState_.segment<3>(3);

  Eigen::Matrix<double, 12, 1> dx_hat;
  dx_hat.segment<3>(0) = x1_hat.cross(yg) - cst::gravityConstant * x2_hat_prime + ya + alpha_ * (yv - x1_hat); // x1
  dx_hat.segment<3>(3) = x2_hat_prime.cross(yg) - beta_ * (yv - x1_hat); // x2_prime

  dx_hat.segment<3>(6) = (x1_hat - posCorrFromContactPos_); // using p_dot = R(v_l) = R(x1 - delta)

  sigma_ = rho_ * (initPose_.orientation.toMatrix3().transpose() * Vector3::UnitZ()).cross(x2_hat_prime)
           + oriCorrFromContactPos_ + oriCorrFromOriMeas_;

  dx_hat.segment<3>(9) = (yg - sigma_); // using R_dot = RS(w_l) = RS(yg-sigma)

  return dx_hat;
}

void IterationViking::integrateState_(const Vector & dx_hat)
{
  const Vector3 & vl = dx_hat.segment<3>(6);
  const Vector3 & omega = dx_hat.segment<3>(9);

  // we copy the state and pose before integration to keep them in case we need to replay this iteration with new
  // measurements
  finalState_ = initState_;
  finalPose_ = initPose_;

  // discrete-time integration of x1 and x2
  finalState_.segment<6>(0) += dx_hat.segment<6>(0) * dt_;

  // discrete-time integration of p and R
  finalPose_.SE3_integration(vl * dt_, omega * dt_);

  finalState_.segment<3>(6) = finalPose_.position();
  finalState_.tail(4) = finalPose_.orientation.toVector4();
}

void IterationViking::addOrientationMeasurement(const Matrix3 & oriMeasurement, double gain)
{
  Matrix3 rot_diff = oriMeasurement * initPose_.orientation.toMatrix3().transpose();
  Vector3 rot_diff_vec = kine::skewSymmetricToRotationVector(rot_diff - rot_diff.transpose()) / 2.0;

  oriCorrFromOriMeas_ -= gain * initPose_.orientation.toMatrix3().transpose() * Vector3::UnitZ()
                         * (Vector3::UnitZ()).transpose() * rot_diff_vec;
}

void IterationViking::addContactPosMeasurement(const Vector3 & posMeasurement,
                                               const Vector3 & imuContactPos,
                                               double gainDelta,
                                               double gainSigma)
{
  oriCorrFromContactPos_ +=
      gainSigma
      * (initPose_.orientation.toMatrix3().transpose() * (posMeasurement - initPose_.position())).cross(imuContactPos);

  posCorrFromContactPos_ +=
      gainDelta
      * (imuContactPos - initPose_.orientation.toMatrix3().transpose() * (posMeasurement - initPose_.position()));
}

IterationViking::~IterationViking() {}

Viking::Viking(double dt, double alpha, double beta, double rho)
: DelayedMeasurementComplemFilter<IterationViking>(dt, 13, 9)
{
  setAlpha(alpha);
  setBeta(beta);
  setRho(rho);
}

Viking::Viking(double dt, double alpha, double beta, double rho, unsigned long bufferCapacity)
: DelayedMeasurementComplemFilter<IterationViking>(dt, 13, 9, bufferCapacity)
{
  setAlpha(alpha);
  setBeta(beta);
  setRho(rho);
}

Viking::~Viking(){};

void Viking::initEstimator(const Vector3 & pos, const Vector3 & x1, const Vector3 & x2_prime, const Vector4 & R)
{
  Eigen::VectorXd initStateVector = Eigen::VectorXd::Zero(getStateSize());

  initStateVector.segment<3>(0) = x1;
  initStateVector.segment<3>(3) = x2_prime;
  initStateVector.segment<3>(6) = pos;
  initStateVector.tail(4) = R;

  stateKinematics_.position = initStateVector.segment<3>(6);
  stateKinematics_.orientation.fromVector4(initStateVector.tail(4));

  initEstimator(initStateVector);
}

void Viking::startNewIteration_()
{
  if(k_est_ == k_data_)
  {
    ++k_data_;

    IterationViking newIter = IterationViking(getCurrentEstimatedState(), stateKinematics_, dt_);

    bufferedIters_.push_front(std::move(newIter));
  }
}

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
    x_().segment<3>(0) = yv_k;
  }
}

void Viking::addContactPosMeasurement(const Vector3 & posMeasurement,
                                      const Vector3 & imuContactPos,
                                      double gainDelta,
                                      double gainSigma)
{
  startNewIteration_();
  getCurrentIter().addContactPosMeasurement(posMeasurement, imuContactPos, gainDelta, gainSigma);
}

void Viking::addOrientationMeasurement(const Matrix3 & oriMeasurement, double gain)
{
  startNewIteration_();
  getCurrentIter().addOrientationMeasurement(oriMeasurement, gain);
}

ObserverBase::StateVector Viking::oneStepEstimation_()
{
  TimeIndex k = this->x_.getTime();
  IterationViking & currentIter = getCurrentIter();

  BOOST_ASSERT(this->y_.size() > 0 && this->y_.checkIndex(k + 1) && "ERROR: The measurement vector is not set");

  ObserverBase::StateVector x_hat = getCurrentEstimatedState();

  setState(currentIter.runIteration_(), k + 1);
  stateKinematics_ = currentIter.finalPose_;

  k_est_++;

  if(withDelayedOri_)
  {
    bufferedIters_.push_front(currentIter);
  }

  return x_hat;
}

ObserverBase::StateVector Viking::replayIterationWithDelayedOri(unsigned long delay, const Matrix3 & meas, double gain)
{
  BOOST_ASSERT_MSG(withDelayedOri_, "The mode allowing to deal with delayed orientations has not been switched on.");

  IterationViking & bufferedIter = bufferedIters_.at(delay - 1);

  bufferedIter.addOrientationMeasurement(meas, gain);

  return bufferedIter.runIteration_();
}

ObserverBase::StateVector Viking::replayIterationsWithDelayedOri(unsigned long delay, const Matrix3 & meas, double gain)
{
  BOOST_ASSERT_MSG(withDelayedOri_, "The mode allowing to deal with delayed orientations has not been switched on.");
  BOOST_ASSERT_MSG(k_data_ == k_est_, "The replay must be called at the beginning or the end of the iteration.");

  IterationViking & bufferedIter = bufferedIters_.at(delay - 1);
  StateVector latestState = getCurrentEstimatedState();

  Eigen::Ref<Eigen::Matrix<double, 7, 1>> latestStatePose = latestState.tail(7);

  kine::Kinematics & latestKine = getCurrentIter().finalPose_;

  // compute the delta between the pose at the buffered iteration and the most recent one.
  kine::Kinematics deltaKine = bufferedIter.finalPose_.getInverse() * latestKine;

  // we add the delayed orientation measurement to the inputs of the buffered iteration and recompute the state update
  // (bufferedIter.updatedPose_ is thus now the new one)
  replayIterationWithDelayedOri(delay, meas, gain);

  // apply the pose delta to the updated pose of the buffered iteration
  latestKine = bufferedIter.finalPose_ * deltaKine;

  latestStatePose = latestKine.toVector(kine::Kinematics::Flags::pose);

  setCurrentState(latestState);
  getCurrentIter().finalState_ = latestState;

  return latestState;
}

} // namespace stateObservation
