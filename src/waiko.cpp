#include <state-observation/observer/waiko.hpp>
#include <state-observation/tools/definitions.hpp>


namespace stateObservation
{

Waiko::Waiko(double dt, double alpha, double beta, double rho, unsigned long bufferCapacity)
: DelayedMeasurementComplemFilter(dt, 13, 9, bufferCapacity)
{
  setAlpha(alpha);
  setBeta(beta);
  setRho(rho);
}

Waiko::~Waiko() {}

void Waiko::setMeasurement(const Vector3 & yv_k,
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

void Waiko::computeCorrectionTerms(StateIterator it)
{
  TimeIndex k = it->getTime();

  BOOST_ASSERT(this->u_.size() > 0 && this->u_.checkIndex(k - 1) && "ERROR: The input is not set");

  StateIterator prevIter = it + 1;
  const Eigen::Ref<Vector3> initPos = (*prevIter)().segment<3>(6);
  const Eigen::Ref<Vector4> initOri_quat = (*prevIter)().tail(4);
  kine::Orientation initOri;
  initOri.fromVector4(initOri_quat);
  Matrix3 initOri_inv = initOri.toMatrix3().transpose().eval();
  const Eigen::Ref<Vector3> x2_hat_prime = (*prevIter)().segment<3>(3);

  InputWaiko & input = std::any_cast<InputWaiko &>(u_[k - 1]);

  oriCorrection_.setZero();

  posCorrection_.setZero();
  for(auto & [posMeas, gainDelta, gainSigma] : input.pos_measurements_from_contact_)
  {
    const Eigen::Ref<Vector3> posMeasurement = posMeas.head(3);
    const Eigen::Ref<Vector3> imuContactPos = posMeas.tail(3);
    posCorrection_ += gainDelta * (imuContactPos - initOri_inv * (posMeasurement - initPos));
    oriCorrection_ += gainSigma * (initOri_inv * (posMeasurement - initPos)).cross(imuContactPos);
  }

  for(auto & [oriMeas, gain] : input.ori_measurements_)
  {
    Matrix3 rot_diff = oriMeas * initOri_inv;
    Vector3 rot_diff_vec = kine::skewSymmetricToRotationVector(rot_diff - rot_diff.transpose()) / 2.0;

    oriCorrection_ -= gain * initOri_inv * Vector3::UnitZ() * (Vector3::UnitZ()).transpose() * rot_diff_vec;
  }

  oriCorrection_ = rho_ * (initOri_inv * Vector3::UnitZ()).cross(x2_hat_prime) + oriCorrection_;
}

void Waiko::startNewIteration_()
{
  TimeIndex k = getCurrentTime();

  if(!u_.checkIndex(k))
  {
    pushInput(InputWaiko());
  }
}

void Waiko::addContactPosMeasurement(const Vector3 & posMeasurement,
                                     const Vector3 & imuContactPos,
                                     double gainDelta,
                                     double gainSigma)
{
  startNewIteration_();

  InputWaiko & input = std::any_cast<InputWaiko &>(u_.back());
  Vector6 inputPos;
  inputPos << posMeasurement, imuContactPos;

  input.pos_measurements_from_contact_.emplace_back(
      InputWaiko::ContactPosMeas_Gains(std::move(inputPos), gainDelta, gainSigma));
}

void Waiko::addOrientationMeasurement(const Matrix3 & oriMeasurement, double gain)
{
  startNewIteration_();

  InputWaiko & input = std::any_cast<InputWaiko &>(u_.back());
  input.ori_measurements_.push_back(InputWaiko::OriMeas_Gain(oriMeasurement, gain));
}

ObserverBase::StateVector Waiko::computeStateDynamics_(StateIterator it)
{
  TimeIndex k = (*it).getTime();
  MeasureVector & synced_Meas = y_[k];

  computeCorrectionTerms(it);

  const Vector3 & yv = synced_Meas.head<3>();
  const Vector3 & ya = synced_Meas.segment<3>(3);
  const Vector3 & yg = synced_Meas.segment<3>(6);

  // we fetch the estimated state from the previous iteration
  const ObserverBase::StateVector & x_hat = (*(it + 1))();
  kine::Kinematics kine(x_hat.tail(7), kine::Kinematics::Flags::pose);

  const auto & x1_hat = x_hat.segment<3>(0);
  const auto & x2_hat_prime = x_hat.segment<3>(3);

  Eigen::Matrix<double, 12, 1> dx_hat;
  dx_hat.segment<3>(0) = x1_hat.cross(yg) - cst::gravityConstant * x2_hat_prime + ya + alpha_ * (yv - x1_hat); // x1
  dx_hat.segment<3>(3) = x2_hat_prime.cross(yg) - beta_ * (yv - x1_hat); // x2_prime

  dx_hat.segment<3>(6) = (x1_hat - posCorrection_); // using p_dot = R(v_l) = R(x1 - delta)

  dx_hat.segment<3>(9) = (yg - oriCorrection_); // using R_dot = RS(w_l) = RS(yg-sigma)

  return dx_hat;
}

void Waiko::integrateState_(StateIterator it, const Vector & dx_hat)
{
  const ObserverBase::StateVector & initState = (*(it + 1))();
  ObserverBase::StateVector & newState = (*(it))();

  const Vector3 & vl = dx_hat.segment<3>(6);
  const Vector3 & omega = dx_hat.segment<3>(9);

  kine::Kinematics kine(initState.tail(7), kine::Kinematics::Flags::pose);
  // discrete-time integration of x1 and x2
  newState.segment<6>(0) = initState.segment<6>(0) + dx_hat.segment<6>(0) * dt_;

  // discrete-time integration of p and R
  kine.SE3_integration(vl * dt_, omega * dt_);

  newState.segment<3>(6) = kine.position();
  newState.tail(4) = kine.orientation.toVector4();
}

void Waiko::initEstimator(const Vector3 & x1, const Vector3 & x2_prime, const Vector3 & pos, const Vector4 & R)
{
  Eigen::VectorXd initStateVector = Eigen::VectorXd::Zero(getStateSize());

  initStateVector.segment<3>(0) = x1;
  initStateVector.segment<3>(3) = x2_prime;
  initStateVector.segment<3>(6) = pos;
  initStateVector.tail(4) = R;

  initEstimator(initStateVector);
}

ObserverBase::StateVector Waiko::oneStepEstimation_(StateIterator it)
{
  BOOST_ASSERT(this->y_.size() > 0 && this->y_.checkIndex(it->getTime()) && "ERROR: The measurement vector is not set");

  Vector dx_hat = computeStateDynamics_(it);
  integrateState_(it, dx_hat);

  return (*(it))();
}

} // namespace stateObservation
