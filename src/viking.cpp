#include "state-observation/tools/definitions.hpp"
#include <state-observation/observer/viking.hpp>

namespace stateObservation
{
Viking::Viking(double alpha, double beta, double rho, double dt)
: ZeroDelayObserver(13, 9), iterInfos_(alpha, beta, rho, dt)
{
  iterInfos_.alpha_ = alpha;
  iterInfos_.beta_ = beta;
  iterInfos_.rho_ = rho;
  iterInfos_.dt_ = dt;
}

void Viking::initEstimator(const Vector3 & pos, const Vector3 & x1, const Vector3 & x2_prime, const Vector4 & R)
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

void Viking::IterInfos::startNewIteration()
{
  if(k_est_ == k_data_)
  {
    ++k_data_;

    initState_ = updatedState_;
    initPose_ = updatedPose_;

    resetCorrectionTerms();
  }
}

void Viking::IterInfos::resetCorrectionTerms()
{
  sigma_.setZero();
  oriCorrFromMeas_.setZero();
  posCorrFromMeas_.setZero();
}

void Viking::setMeasurement(const Vector3 & yv_k,
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

void Viking::setMeasurement(const ObserverBase::MeasureVector & y_k, TimeIndex k)
{
  getCurrentIter().startNewIteration();

  ZeroDelayObserver::setMeasurement(y_k, k);

  getCurrentIter().saveMeasurement(getMeasurement(getMeasurementTime()));
}

void Viking::IterInfos::addOrientationMeasurement(const Matrix3 & oriMeasurement, double gain)
{
  startNewIteration();

  Matrix3 rot_diff = oriMeasurement * initPose_.orientation.toMatrix3().transpose();
  Vector3 rot_diff_vec = kine::skewSymmetricToRotationVector(rot_diff - rot_diff.transpose()) / 2.0;

  oriCorrFromMeas_ -= gain * initPose_.orientation.toMatrix3().transpose() * Vector3::UnitZ()
                      * (Vector3::UnitZ()).transpose() * rot_diff_vec;
}

void Viking::addPosMeasurement(const Vector3 & posMeasurement, double gain)
{
  getCurrentIter().addPosMeasurement(posMeasurement, gain);
}

void Viking::addOrientationMeasurement(const Matrix3 & oriMeasurement, double gain)
{
  getCurrentIter().addOrientationMeasurement(oriMeasurement, gain);
}

void Viking::IterInfos::addPosMeasurement(const Vector3 & posMeasurement, double gain)
{
  startNewIteration();

  posCorrFromMeas_ += gain * (posMeasurement - initPose_.position());
}

ObserverBase::StateVector Viking::oneStepEstimation_()
{
  TimeIndex k = this->x_.getTime();
  IterInfos & currentIter = getCurrentIter();

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

ObserverBase::StateVector Viking::IterInfos::replayBufferedIteration()
{
  Eigen::Matrix<double, 12, 1> dx_hat = computeStateDerivatives();
  integrateState(dx_hat);

  return updatedState_;
}

Eigen::Matrix<double, 12, 1> Viking::IterInfos::computeStateDerivatives()
{
  const Vector3 & yv = y_k_.head<3>();
  const Vector3 & ya = y_k_.segment<3>(3);
  const Vector3 & yg = y_k_.segment<3>(6);

  const Eigen::Ref<Vector3> x1_hat = initState_.segment<3>(0);
  const Eigen::Ref<Vector3> x2_hat_prime = initState_.segment<3>(3);

  Eigen::Matrix<double, 12, 1> dx_hat;
  dx_hat.segment<3>(0) = x1_hat.cross(yg) - cst::gravityConstant * x2_hat_prime + ya + alpha_ * (yv - x1_hat); // x1
  dx_hat.segment<3>(3) = x2_hat_prime.cross(yg) - beta_ * (yv - x1_hat); // x2_prime

  dx_hat.segment<3>(6) = (x1_hat - posCorrFromMeas_); // using p_dot = R(v_l) = R(x1 - delta)

  sigma_ =
      rho_ * (initPose_.orientation.toMatrix3().transpose() * Vector3::UnitZ()).cross(x2_hat_prime) + oriCorrFromMeas_;

  dx_hat.segment<3>(9) = (yg - sigma_); // using R_dot = RS(w_l) = RS(yg-sigma)

  return dx_hat;
}

void Viking::IterInfos::integrateState(const Eigen::Matrix<double, 12, 1> & dx_hat)
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

ObserverBase::StateVector Viking::replayIterationWithDelayedOri(unsigned long delay,
                                                                const Matrix3 & oriMeas,
                                                                double gain)
{
  BOOST_ASSERT_MSG(withDelayedOri_, "The mode allowing to deal with delayed orientations has not been switched on.");

  IterInfos & bufferedIter = bufferedIters_.at(delay - 1);
  // allows to avoid runing startNewIteration() and thus resetting the correction terms.
  bufferedIter.k_est_--;

  bufferedIter.addOrientationMeasurement(oriMeas, gain);

  bufferedIter.k_est_++;

  return bufferedIter.replayBufferedIteration();
}

ObserverBase::StateVector Viking::replayIterationsWithDelayedOri(unsigned long delay,
                                                                 const Matrix3 & oriMeas,
                                                                 double gain)
{
  BOOST_ASSERT_MSG(withDelayedOri_, "The mode allowing to deal with delayed orientations has not been switched on.");
  BOOST_ASSERT_MSG(getCurrentIter().k_data_ == getCurrentIter().k_est_,
                   "The replay must be called at the beginning or the end of the iteration.");

  IterInfos & bufferedIter = bufferedIters_.at(delay - 1);
  StateVector latestState = getCurrentEstimatedState();

  Eigen::Ref<Eigen::Matrix<double, 7, 1>> latestStatePose = latestState.tail(7);

  kine::Kinematics & latestKine = getCurrentIter().updatedPose_;

  kine::Kinematics deltaKine = bufferedIter.updatedPose_.getInverse() * latestKine;

  // we add the delayed orientation measurement to the inputs of the buffered iteration and recompute the state update.
  replayIterationWithDelayedOri(delay, oriMeas, gain);
  latestKine = bufferedIter.updatedPose_ * deltaKine;

  latestStatePose = latestKine.toVector(kine::Kinematics::Flags::pose);

  setCurrentState(latestState);
  getCurrentIter().updatedState_ = latestState;

  return latestState;
}

ObserverBase::StateVector Viking::replayIterationWithDelayedPosition(unsigned long delay,
                                                                     const Matrix3 & posMeas,
                                                                     double gain)
{
  BOOST_ASSERT_MSG(withDelayedOri_, "The mode allowing to deal with delayed orientations has not been switched on.");

  IterInfos & bufferedIter = bufferedIters_.at(delay - 1);
  // allows to avoid runing startNewIteration() and thus resetting the correction terms.
  bufferedIter.k_est_--;

  bufferedIter.addPosMeasurement(posMeas, gain);

  bufferedIter.k_est_++;

  return bufferedIter.replayBufferedIteration();
}

ObserverBase::StateVector Viking::replayIterationsWithDelayedPosition(unsigned long delay,
                                                                      const Matrix3 & posMeas,
                                                                      double gain)
{
  BOOST_ASSERT_MSG(withDelayedOri_, "The mode allowing to deal with delayed orientations has not been switched on.");
  BOOST_ASSERT_MSG(getCurrentIter().k_data_ == getCurrentIter().k_est_,
                   "The replay must be called at the beginning or the end of the iteration.");

  IterInfos & bufferedIter = bufferedIters_.at(delay - 1);
  StateVector latestState = getCurrentEstimatedState();

  Eigen::Ref<Eigen::Matrix<double, 7, 1>> latestStatePose = latestState.tail(7);

  kine::Kinematics & latestKine = getCurrentIter().updatedPose_;

  // compute the delta between the pose at the buffered iteration and the most recent one.
  kine::Kinematics deltaKine = bufferedIter.updatedPose_.getInverse() * latestKine;

  // we add the delayed position measurement to the inputs of the buffered iteration and recompute the state update
  // (bufferedIter.updatedPose_ is thus now the new one)
  replayIterationWithDelayedPosition(delay, posMeas, gain);

  // apply the pose delta to the updated pose of the buffered iteration
  latestKine = bufferedIter.updatedPose_ * deltaKine;

  latestStatePose = latestKine.toVector(kine::Kinematics::Flags::pose);

  setCurrentState(latestState);
  getCurrentIter().updatedState_ = latestState;

  return latestState;
}

} // namespace stateObservation
