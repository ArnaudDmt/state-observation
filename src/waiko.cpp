#include "state-observation/tools/definitions.hpp"
#include <state-observation/observer/waiko.hpp>

namespace stateObservation
{
Waiko::Waiko(double alpha, double beta, double rho, double dt)
: ZeroDelayObserver(13, 9), currentIter_(alpha, beta, rho, dt)
{
  currentIter_.alpha_ = alpha;
  currentIter_.beta_ = beta;
  currentIter_.rho_ = rho;
  currentIter_.dt_ = dt;
}

void Waiko::initEstimator(const Vector3 & pos, const Vector3 & x1, const Vector3 & x2_prime, const Vector4 & R)
{
  Eigen::VectorXd initStateVector = Eigen::VectorXd::Zero(getStateSize());

  initStateVector.segment<3>(0) = x1;
  initStateVector.segment<3>(3) = x2_prime;
  initStateVector.segment<3>(6) = pos;
  initStateVector.tail(4) = R;

  setState(initStateVector, 0);

  getCurrentIter().initState_ = initStateVector;
  getCurrentIter().updatedState_ = initStateVector;

  getCurrentIter().initPose_.position = pos;
  getCurrentIter().initPose_.orientation.fromVector4(R);
  getCurrentIter().updatedPose_ = getCurrentIter().initPose_;
}

void Waiko::Iteration::startNewIteration()
{
  if(k_est_ == k_data_)
  {
    ++k_data_;

    initState_ = updatedState_;
    initPose_ = updatedPose_;

    resetCorrectionTerms();
  }
}

void Waiko::Iteration::resetCorrectionTerms()
{
  sigma_.setZero();
  oriCorrFromOriMeas_.setZero();
  posCorrFromContactPos_.setZero();
  oriCorrFromContactPos_.setZero();
}

void Waiko::setMeasurement(const Vector3 & yv_k,
                           const Vector3 & ya_k,
                           const Vector3 & yg_k,
                           TimeIndex k,
                           bool resetImuLocVelHat)
{
  getCurrentIter().startNewIteration();

  ObserverBase::MeasureVector y_k(getMeasureSize());
  y_k << yv_k, ya_k, yg_k;

  setMeasurement(y_k, k);

  if(resetImuLocVelHat)
  {
    x_().segment<3>(0) = yv_k;
  }
}

void Waiko::setMeasurement(const ObserverBase::MeasureVector & y_k, TimeIndex k)
{
  getCurrentIter().startNewIteration();

  ZeroDelayObserver::setMeasurement(y_k, k);

  getCurrentIter().saveMeasurement(getMeasurement(getMeasurementTime()));
}

void Waiko::Iteration::addOrientationMeasurement(const Matrix3 & oriMeasurement, double gain)
{
  startNewIteration();

  Matrix3 rot_diff = oriMeasurement * initPose_.orientation.toMatrix3().transpose();
  Vector3 rot_diff_vec = kine::skewSymmetricToRotationVector(rot_diff - rot_diff.transpose()) / 2.0;

  oriCorrFromOriMeas_ -= gain * initPose_.orientation.toMatrix3().transpose() * Vector3::UnitZ()
                         * (Vector3::UnitZ()).transpose() * rot_diff_vec;
}

void Waiko::addContactPosMeasurement(const Vector3 & posMeasurement,
                                     const Vector3 & imuContactPos,
                                     double gainDelta,
                                     double gainSigma)
{
  getCurrentIter().addContactPosMeasurement(posMeasurement, imuContactPos, gainDelta, gainSigma);
}

void Waiko::addOrientationMeasurement(const Matrix3 & oriMeasurement, double gain)
{
  getCurrentIter().addOrientationMeasurement(oriMeasurement, gain);
}

void Waiko::Iteration::addContactPosMeasurement(const Vector3 & posMeasurement,
                                                const Vector3 & imuContactPos,
                                                double gainDelta,
                                                double gainSigma)
{
  startNewIteration();

  oriCorrFromContactPos_ +=
      gainSigma
      * (initPose_.orientation.toMatrix3().transpose() * (posMeasurement - initPose_.position())).cross(imuContactPos);

  posCorrFromContactPos_ +=
      gainDelta
      * (imuContactPos - initPose_.orientation.toMatrix3().transpose() * (posMeasurement - initPose_.position()));
}

ObserverBase::StateVector Waiko::oneStepEstimation_()
{
  TimeIndex k = this->x_.getTime();
  Iteration & currentIter = getCurrentIter();

  BOOST_ASSERT(this->y_.size() > 0 && this->y_.checkIndex(k + 1) && "ERROR: The measurement vector is not set");

  ObserverBase::StateVector x_hat = getCurrentEstimatedState();

  Eigen::Matrix<double, 12, 1> dx_hat = currentIter.computeStateDerivatives();
  currentIter.integrateState(dx_hat);

  setState(currentIter.updatedState_, k + 1);

  currentIter.k_est_++;

  if(withDelayedOri_)
  {
    bufferedIters_.push_front(currentIter);
  }

  return x_hat;
}

ObserverBase::StateVector Waiko::Iteration::replayBufferedIteration()
{
  Eigen::Matrix<double, 12, 1> dx_hat = computeStateDerivatives();
  integrateState(dx_hat);

  return updatedState_;
}

Eigen::Matrix<double, 12, 1> Waiko::Iteration::computeStateDerivatives()
{
  const Vector3 & yv = y_k_.head<3>();
  const Vector3 & ya = y_k_.segment<3>(3);
  const Vector3 & yg = y_k_.segment<3>(6);

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

void Waiko::Iteration::integrateState(const Eigen::Matrix<double, 12, 1> & dx_hat)
{
  const Vector3 & vl = dx_hat.segment<3>(6);
  const Vector3 & omega = dx_hat.segment<3>(9);

  // we copy the state and pose before integration to keep them in case we need to replay this iteration with new
  // measurements
  updatedState_ = initState_;
  updatedPose_ = initPose_;

  // discrete-time integration of x1 and x2
  updatedState_.segment<6>(0) += dx_hat.segment<6>(0) * dt_;

  // discrete-time integration of p and R
  updatedPose_.SE3_integration(vl * dt_, omega * dt_);

  updatedState_.segment<3>(6) = updatedPose_.position();
  updatedState_.tail(4) = updatedPose_.orientation.toVector4();
}

ObserverBase::StateVector Waiko::replayIterationWithDelayedOri(unsigned long delay, const Matrix3 & meas, double gain)
{
  BOOST_ASSERT_MSG(withDelayedOri_, "The mode allowing to deal with delayed orientations has not been switched on.");

  Iteration & bufferedIter = bufferedIters_.at(delay - 1);
  // allows to avoid runing startNewIteration() and thus resetting the correction terms.
  bufferedIter.k_est_--;

  bufferedIter.addOrientationMeasurement(meas, gain);

  bufferedIter.k_est_++;

  return bufferedIter.replayBufferedIteration();
}

ObserverBase::StateVector Waiko::replayIterationsWithDelayedOri(unsigned long delay, const Matrix3 & meas, double gain)
{
  BOOST_ASSERT_MSG(withDelayedOri_, "The mode allowing to deal with delayed orientations has not been switched on.");
  BOOST_ASSERT_MSG(getCurrentIter().k_data_ == getCurrentIter().k_est_,
                   "The replay must be called at the beginning or the end of the iteration.");

  Iteration & bufferedIter = bufferedIters_.at(delay - 1);
  StateVector latestState = getCurrentEstimatedState();

  Eigen::Ref<Eigen::Matrix<double, 7, 1>> latestStatePose = latestState.tail(7);

  kine::Kinematics & latestKine = getCurrentIter().updatedPose_;

  // compute the delta between the pose at the buffered iteration and the most recent one.
  kine::Kinematics deltaKine = bufferedIter.updatedPose_.getInverse() * latestKine;

  // we add the delayed orientation measurement to the inputs of the buffered iteration and recompute the state update
  // (bufferedIter.updatedPose_ is thus now the new one)
  replayIterationWithDelayedOri(delay, meas, gain);

  // apply the pose delta to the updated pose of the buffered iteration
  latestKine = bufferedIter.updatedPose_ * deltaKine;

  latestStatePose = latestKine.toVector(kine::Kinematics::Flags::pose);

  setCurrentState(latestState);
  getCurrentIter().updatedState_ = latestState;

  return latestState;
}

} // namespace stateObservation
