/*
 * Copyright (c) 2019-2020
 * @author Mehdi BENALLEGUE
 *
 * National Institute of Advanced Industrial Science and Technology (AIST)
 */
#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#ifndef NDEBUG
#  include <iostream>
#endif

namespace stateObservation
{
inline Matrix6 STATE_OBSERVATION_DLLAPI blockMat6(const Matrix3 & m1,
                                                  const Matrix3 & m2,
                                                  const Matrix3 & m3,
                                                  const Matrix3 & m4)
{
  Matrix6 m;
  m << m1, m2, m3, m4;

  return m;
}

/// resets one block on the diagonal of the state Covariance Matrix
/// i.e. sets value of a square block on the diagonal of the covMat
/// and sets to zero all the values related to their lines and columns
template<int blockSize>
void STATE_OBSERVATION_DLLAPI setBlockStateCovariance(Matrix & covMat, const Matrix & covBlock, Index blockIndex)
{
  long int matrixSize = covMat.rows();
  covMat.block<blockSize, blockSize>(blockIndex, blockIndex) = covBlock;
  covMat.block(blockIndex, 0, blockSize, blockIndex).setZero();
  covMat.block(0, blockIndex, blockIndex, blockSize).setZero();
  covMat.block(blockIndex + blockSize, blockIndex, matrixSize - blockIndex - blockSize, blockSize).setZero();
  covMat.block(blockIndex, blockIndex + blockSize, blockSize, matrixSize - blockIndex - blockSize).setZero();
}

inline void STATE_OBSERVATION_DLLAPI
    fillSymmetricMatrix(Matrix3 & m, const Vector3 & vdiag, double e1, double e2, double e3)
{
  m.diagonal() = vdiag;
  m(1, 0) = m(0, 1) = e1;
  m(2, 0) = m(0, 2) = e2;
  m(2, 1) = m(1, 2) = e3;
}

const double KineticsObserver::defaultMass = 50;

const double KineticsObserver::statePoseInitVarianceDefault = 1e-4;
const double KineticsObserver::stateOriInitVarianceDefault = 1e-4;
const double KineticsObserver::stateLinVelInitVarianceDefault = 1e-6;
const double KineticsObserver::stateAngVelInitVarianceDefault = 1e-6;
const double KineticsObserver::gyroBiasInitVarianceDefault = 1e-10;
const double KineticsObserver::unmodeledWrenchInitVarianceDefault = 1e2;
const double KineticsObserver::contactForceInitVarianceDefault = 1e2;
const double KineticsObserver::contactTorqueInitVarianceDefault = 1e2;

const double KineticsObserver::statePoseProcessVarianceDefault = 1e-8;
const double KineticsObserver::stateOriProcessVarianceDefault = 1e-8;
const double KineticsObserver::stateLinVelProcessVarianceDefault = 1e-8;
const double KineticsObserver::stateAngVelProcessVarianceDefault = 1e-8;
const double KineticsObserver::gyroBiasProcessVarianceDefault = 1e-12;
const double KineticsObserver::unmodeledWrenchProcessVarianceDefault = 1e-8;
const double KineticsObserver::contactPositionProcessVarianceDefault = 1e-8;
const double KineticsObserver::contactOrientationProcessVarianceDefault = 1e-8;
const double KineticsObserver::contactForceProcessVarianceDefault = 1e-8;
const double KineticsObserver::contactTorqueProcessVarianceDefault = 1e-8;

const double KineticsObserver::acceleroVarianceDefault = 1e-4;
const double KineticsObserver::gyroVarianceDefault = 1e-8;
const double KineticsObserver::forceSensorVarianceDefault = 1e-8;
const double KineticsObserver::torqueSensorVarianceDefault = 1e-10;
const double KineticsObserver::positionSensorVarianceDefault = 1e-4;
const double KineticsObserver::orientationSensorVarianceDefault = 1e-3;

const double KineticsObserver::linearStiffnessDefault = 40000;
const double KineticsObserver::angularStiffnessDefault = 400;
const double KineticsObserver::linearDampingDefault = 120;
const double KineticsObserver::angularDampingDefault = 12;

const double KineticsObserver::defaultdx = 1e-6;

const int measurementSizeBase = 0;
const int inputSize = 0;

KineticsObserver::KineticsObserver(unsigned maxContacts, unsigned maxNumberOfIMU)
: maxContacts_(maxContacts), maxImuNumber_(maxNumberOfIMU), contacts_(maxContacts_), imuSensors_(maxImuNumber_),
  stateSize_(sizeStateBase + maxImuNumber_ * sizeGyroBias + maxContacts * sizeContact),
  stateTangentSize_(sizeStateTangentBase + maxImuNumber_ * sizeGyroBias + sizeContactTangent * maxContacts),
  measurementSize_(0), measurementTangentSize_(0), worldCentroidStateVector_(stateSize_),
  worldCentroidStateVectorDx_(stateTangentSize_), oldWorldCentroidStateVector_(stateSize_),
  additionalForce_(Vector3::Zero()), additionalTorque_(Vector3::Zero()),
  ekf_(stateSize_, stateTangentSize_, measurementSizeBase, measurementSizeBase, false, false),
  finiteDifferencesJacobians_(false), withGyroBias_(true), withUnmodeledWrench_(true),
  withAccelerationEstimation_(false), withDampingInMatrixA_(true), withAdaptativeContactProcessCov_(true), k_est_(0),
  k_data_(0), mass_(defaultMass), dt_(defaultdx), processNoise_(0x0), measurementNoise_(0x0),
  numberOfContactRealSensors_(0), currentIMUSensorNumber_(0),
  linearStiffnessMatDefault_(Matrix3::Identity() * linearStiffnessDefault),
  angularStiffnessMatDefault_(Matrix3::Identity() * angularStiffnessDefault),
  linearDampingMatDefault_(Matrix3::Identity() * linearDampingDefault),
  angularDampingMatDefault_(Matrix3::Identity() * angularDampingDefault),
  acceleroCovMatDefault_(Matrix3::Identity() * acceleroVarianceDefault),
  gyroCovMatDefault_(Matrix3::Identity() * gyroVarianceDefault),
  contactWrenchSensorCovMatDefault_(blockMat6(Matrix3::Identity() * forceSensorVarianceDefault,
                                              Matrix3::Zero(),
                                              Matrix3::Zero(),
                                              Matrix3::Identity() * torqueSensorVarianceDefault)),
  absPoseSensorCovMatDefault_(blockMat6(Matrix3::Identity() * positionSensorVarianceDefault,
                                        Matrix3::Zero(),
                                        Matrix3::Zero(),
                                        Matrix3::Identity() * orientationSensorVarianceDefault)),
  absOriSensorCovMatDefault_(Matrix3::Identity() * orientationSensorVarianceDefault),
  statePosInitCovMat_(Matrix3::Identity() * statePoseInitVarianceDefault),
  stateOriInitCovMat_(Matrix3::Identity() * stateOriInitVarianceDefault),
  stateLinVelInitCovMat_(Matrix3::Identity() * stateLinVelInitVarianceDefault),
  stateAngVelInitCovMat_(Matrix3::Identity() * stateAngVelInitVarianceDefault),
  gyroBiasInitCovMat_(Matrix3::Identity() * gyroBiasInitVarianceDefault),
  unmodeledWrenchInitCovMat_(Matrix6::Identity() * unmodeledWrenchInitVarianceDefault),
  statePosProcessCovMat_(Matrix3::Identity() * statePoseProcessVarianceDefault),
  stateOriProcessCovMat_(Matrix3::Identity() * stateOriProcessVarianceDefault),
  stateLinVelProcessCovMat_(Matrix3::Identity() * stateLinVelProcessVarianceDefault),
  stateAngVelProcessCovMat_(Matrix3::Identity() * stateAngVelProcessVarianceDefault),
  gyroBiasProcessCovMat_(Matrix3::Identity() * gyroBiasProcessVarianceDefault),
  unmodeledWrenchProcessCovMat_(Matrix6::Identity() * unmodeledWrenchProcessVarianceDefault),
  contactPositionProcessCovMat_(Matrix3::Identity() * contactPositionProcessVarianceDefault),
  contactOrientationProcessCovMat_(Matrix3::Identity() * contactOrientationProcessVarianceDefault),
  contactForceProcessCovMat_(Matrix3::Identity() * contactForceInitVarianceDefault),
  contactTorqueProcessCovMat_(Matrix3::Identity() * contactTorqueInitVarianceDefault)
{
  ekf_.setFunctor(this);
  ekf_.setStateArithmetics(this);

  /*  The state vector is initialized to zero but can be initialized afterwards using setInitWorldCentroidStateVector();
   */
  worldCentroidStateVector_.setZero();
  oldWorldCentroidStateVector_ = worldCentroidStateVector_;

  ekf_.setState(worldCentroidStateVector_, k_est_);

  stateKinematicsInitCovMat_.setZero();
  stateKinematicsInitCovMat_.block<sizePos, sizePos>(posIndexTangent(), posIndexTangent()) = statePosInitCovMat_;
  stateKinematicsInitCovMat_.block<sizeOriTangent, sizeOriTangent>(oriIndexTangent(), oriIndexTangent()) =
      stateOriInitCovMat_;
  stateKinematicsInitCovMat_.block<sizeLinVel, sizeLinVel>(linVelIndexTangent(), linVelIndexTangent()) =
      stateLinVelInitCovMat_;
  stateKinematicsInitCovMat_.block<sizeAngVel, sizeAngVel>(angVelIndexTangent(), angVelIndexTangent()) =
      stateAngVelInitCovMat_;

  stateKinematicsProcessCovMat_.setZero();
  stateKinematicsProcessCovMat_.block<sizePos, sizePos>(posIndexTangent(), posIndexTangent()) = statePosProcessCovMat_;
  stateKinematicsProcessCovMat_.block<sizeOriTangent, sizeOriTangent>(oriIndexTangent(), oriIndexTangent()) =
      stateOriProcessCovMat_;
  stateKinematicsProcessCovMat_.block<sizeLinVel, sizeLinVel>(linVelIndexTangent(), linVelIndexTangent()) =
      stateLinVelProcessCovMat_;
  stateKinematicsProcessCovMat_.block<sizeAngVel, sizeAngVel>(angVelIndexTangent(), angVelIndexTangent()) =
      stateAngVelProcessCovMat_;

  contactProcessCovMatDefault_.setZero();
  contactProcessCovMatDefault_.block<sizePos, sizePos>(0, 0) = contactPositionProcessCovMat_;
  contactProcessCovMatDefault_.block<sizeOriTangent, sizeOriTangent>(3, 3) = contactOrientationProcessCovMat_;
  contactProcessCovMatDefault_.block<sizeForce, sizeForce>(6, 6) = contactForceProcessCovMat_;
  contactProcessCovMatDefault_.block<sizeTorque, sizeTorque>(9, 9) = contactTorqueProcessCovMat_;

  contactInitCovMatDefault_ = contactProcessCovMatDefault_;

  I_.set(Matrix3::Identity(), k_data_);
  Id_.set(Matrix3::Zero(), k_data_);
  comd_.set(Vector3::Zero(), k_data_);
  comdd_.set(Vector3::Zero(), k_data_);
  sigma_.set(Vector3::Zero(), k_data_);
  sigmad_.set(Vector3::Zero(), k_data_);

  ekf_.setStateCovariance(ekf_.getPmatrixZero());
  ekf_.setQ(ekf_.getQmatrixZero());
  ekf_.setR(ekf_.getRmatrixZero());

  resetStateCovarianceMat();
  resetProcessCovarianceMat();

  worldCentroidStateVectorDx_.setConstant(1e-6);

  for(unsigned nbContacts = 2; nbContacts <= maxContacts_; nbContacts++)
  {
    Eigen::MatrixXd one_t(3, nbContacts * 3);
    Eigen::MatrixXd z_t = Eigen::MatrixXd::Zero(3, nbContacts * 3);

    for(unsigned i = 0; i < nbContacts; i++)
    {
      one_t.block(0, i * 3, 3, 3) = Eigen::Matrix3d::Identity();
      z_t(2, (i * 3) + 2) = 1.0;
    }

    Eigen::MatrixXd one_t_pinv = (1.0 / nbContacts) * one_t.transpose();
    Eigen::MatrixXd z_t_pinv = (1.0 / nbContacts) * z_t.transpose();

    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(nbContacts * 3, nbContacts * 3) - one_t_pinv * one_t;

    m_matrices_.push_back(M);

    Eigen::MatrixXd M_prime = Eigen::MatrixXd::Identity(nbContacts * 3, nbContacts * 3) - z_t_pinv * z_t;

    m_prime_matrices_.push_back(M_prime);
  }
}

KineticsObserver::~KineticsObserver() {}

Index KineticsObserver::getStateSize() const
{
  return stateSize_;
}

Index KineticsObserver::getStateTangentSize() const
{
  return stateTangentSize_;
}

Index KineticsObserver::getMeasurementSize() const
{
  Index size = 0;
  for(VectorIMUConstIterator i = imuSensors_.begin(); i != imuSensors_.end(); ++i)
  {
    if(i->time == k_data_)
    {
      size += sizeIMUSignal;
    }
  }

  size += numberOfContactRealSensors_ * sizeWrench;

  if(absPoseSensor_.time == k_data_)
  {
    size += sizePose;
  }

  if(absOriSensor_.time == k_data_)
  {
    size += sizeOri;
  }

  return size;
}

double KineticsObserver::getSamplingTime() const
{
  return dt_;
}

void KineticsObserver::setSamplingTime(double dt)
{
  dt_ = dt;
}

void KineticsObserver::setMass(double m)
{
  mass_ = m;
}

void KineticsObserver::updateMeasurements()
{
  for(VectorContactIterator i = contacts_.begin(), ie = contacts_.end(); i != ie; ++i)
  {
    if(i->isSet)
    {
      BOOST_ASSERT((i->time == k_data_) && "The contacts have not all been updated. \
              Either remove lost contacts using removeContact \
              or Run updateContactWithWrenchSensor or updateContactWithNoSensor on every existing contact");

      Contact & contact = *i;
      /// the following code is only an attempt to maintain a consistent state of the state observer
      /// therefore we unset the state
      if(contact.time != k_data_)
      {
        if(contact.withRealSensor)
        {
          contact.withRealSensor = false;
          numberOfContactRealSensors_--;
        }
        contact.isSet = false;
      }
    }
  }

  ///////////// initialize the measurement Vector and matrix //////////////

  measurementSize_ = sizeIMUSignal * currentIMUSensorNumber_ + sizeWrench * numberOfContactRealSensors_;
  measurementTangentSize_ = measurementSize_;
  if(absPoseSensor_.time == k_data_)
  {
    measurementSize_ += sizePose;
    measurementTangentSize_ += sizePoseTangent;
  }

  if(absOriSensor_.time == k_data_)
  {
    measurementSize_ += sizeOri;
    measurementTangentSize_ += sizeOriTangent;
  }

  measurementVector_.resize(measurementSize_);
  measurementCovMatrix_.resize(measurementTangentSize_, measurementTangentSize_);
  measurementCovMatrix_.setZero();

  Index curMeasIndex = 0;

  for(VectorIMUIterator i = imuSensors_.begin(), ie = imuSensors_.end(); i != ie; ++i)
  {
    if(i->time == k_data_)
    {
      IMU & imu = *i;
      imu.measIndex = curMeasIndex;
      measurementVector_.segment<sizeIMUSignal>(curMeasIndex) = imu.acceleroGyro;
      measurementCovMatrix_.block<sizeAcceleroSignal, sizeAcceleroSignal>(curMeasIndex, curMeasIndex) =
          imu.covMatrixAccelero;
      curMeasIndex += sizeAcceleroSignal;
      measurementCovMatrix_.block<sizeGyroSignal, sizeGyroSignal>(curMeasIndex, curMeasIndex) = imu.covMatrixGyro;
      curMeasIndex += sizeGyroSignal;
    }
  }

  for(VectorContactIterator i = contacts_.begin(), ie = contacts_.end(); i != ie; ++i)
  {
    if(i->withRealSensor)
    {
      Contact & contact = *i;

      contact.measIndex = curMeasIndex;
      measurementVector_.segment<sizeWrench>(curMeasIndex) = contact.wrenchMeasurement;
      measurementCovMatrix_.block<sizeWrench, sizeWrench>(curMeasIndex, curMeasIndex) = contact.sensorCovMatrix();
      curMeasIndex += sizeWrench;
    }
  }

  if(absPoseSensor_.time == k_data_)
  {
    absPoseSensor_.measIndex = curMeasIndex;
    BOOST_ASSERT(absPoseSensor_.pose.position.isSet() && absPoseSensor_.pose.orientation.isSet()
                 && "The absolute pose needs to contain the position and the orientation");
    measurementVector_.segment<sizePose>(curMeasIndex) = absPoseSensor_.pose.toVector(flagsPoseKine);
    measurementCovMatrix_.block<sizePoseTangent, sizePoseTangent>(curMeasIndex, curMeasIndex) =
        absPoseSensor_.covMatrix();
    curMeasIndex += sizePos;
  }

  if(absOriSensor_.time == k_data_)
  {
    absOriSensor_.measIndex = curMeasIndex;
    BOOST_ASSERT(absOriSensor_.ori.isSet() && "The absolute orientation is not set");
    measurementVector_.segment<sizeOri>(curMeasIndex) = absOriSensor_.ori.toVector4();
    measurementCovMatrix_.block<sizeOriTangent, sizeOriTangent>(curMeasIndex, curMeasIndex) = absOriSensor_.covMatrix();
  }

  ekf_.setMeasureSize(measurementSize_, measurementTangentSize_);
  ekf_.setMeasurement(measurementVector_, k_data_);
  ekf_.setR(measurementCovMatrix_);
}

void KineticsObserver::setContactProcessCovMat(Index contactNbr,
                                               const Matrix3 * restPosProcessCov,
                                               const Matrix3 * restOriProcessCov,
                                               const Matrix3 * forceProcessCov,
                                               const Matrix3 * torqueProcessCov)
{
  Matrix processCovMat = ekf_.getQ();
  if(restPosProcessCov != nullptr)
  {
    // no need to change Q here as it will be recomputed in updateContactPoseProcessCovariance()
    contactRestPosProcessChanged_ = true;
    contacts_[contactNbr].restPosProcessCovMat = *restPosProcessCov;
  }
  if(restOriProcessCov != nullptr)
  {
    // no need to change Q here as it will be recomputed in updateContactPoseProcessCovariance()
    contactRestOriProcessChanged_ = true;
    contacts_[contactNbr].restOriProcessCovMat = *restOriProcessCov;
  }
  if(forceProcessCov != nullptr)
  {
    setBlockStateCovariance<sizeForceTangent>(processCovMat, *forceProcessCov, contactForceIndexTangent(contactNbr));
  }
  if(torqueProcessCov != nullptr)
  {
    setBlockStateCovariance<sizeTorqueTangent>(processCovMat, *torqueProcessCov, contactTorqueIndexTangent(contactNbr));
  }
  ekf_.setQ(processCovMat);
}

const Vector & KineticsObserver::update()
{

  if(k_est_ != k_data_)
  {
    updateMeasurements();
    updateContactCovariances();

    ekf_.updateStateAndMeasurementPrediction();

    if(finiteDifferencesJacobians_)
    {
      ekf_.setA(ekf_.getAMatrixFD(worldCentroidStateVectorDx_));
      ekf_.setC(ekf_.getCMatrixFD(worldCentroidStateVectorDx_));
    }
    else
    {
      ekf_.setA(computeAMatrix());
      ekf_.setC(computeCMatrix());
    }

    worldCentroidStateVector_ = ekf_.getEstimatedState(k_data_);

    if(worldCentroidStateVector_.hasNaN())
    {
#ifndef NDEBUG
      // std::cout << "Kinetics observer: NaN value detected" << std::endl;
#endif
      stateNaNCorrection_();
    }
    else
    {
      oldWorldCentroidStateVector_ = worldCentroidStateVector_;
    }

    // update of worldCentroidStateKinematics_ and of the contacts pose with the newly estimated state
    worldCentroidStateKinematics_.reset();
    updateLocalKineAndContacts_();
    if(withAccelerationEstimation_)
    {
      // update of worldCentroidStateKinematics_ with the accelerations
      estimateAccelerations();
    }
    updateGlobalKine_();

    endIteration_();
  }

  return worldCentroidStateVector_;
}

const Vector & KineticsObserver::getCurrentStateVector() const
{
  return worldCentroidStateVector_;
}

stateObservation::TimeIndex KineticsObserver::getStateVectorTimeIndex() const
{
  return ekf_.getCurrentTime();
}

kine::LocalKinematics KineticsObserver::getLocalCentroidKinematics() const
{
  return worldCentroidStateKinematics_;
}

kine::LocalKinematics KineticsObserver::getLocalKinematicsOf(const Kinematics & userBodyKine)
{
  Kinematics centroidBodyKine = userBodyKine;
  if(centroidBodyKine.position.isSet())
  {
    centroidBodyKine.position() -= com_();
  }
  if(centroidBodyKine.linVel.isSet())
  {
    centroidBodyKine.linVel() -= comd_();
  }
  if(centroidBodyKine.linAcc.isSet())
  {
    centroidBodyKine.linAcc() -= comdd_();
  }
  LocalKinematics localCentroidBodyKine = LocalKinematics(centroidBodyKine);

  return localCentroidBodyKine; /// product of the kinematics
}

/* Care : unsafe access, for the moment the global kinematics are updated only after the update, so don't call this
 * function before*/
kine::Kinematics KineticsObserver::getGlobalCentroidKinematics() const
{
  return worldCentroidKinematics_;
}

kine::Kinematics KineticsObserver::getGlobalKinematicsOf(const Kinematics & userBodyKin) const
{
  Kinematics centroidBodyKine = userBodyKin;
  if(centroidBodyKine.position.isSet())
  {
    centroidBodyKine.position() -= com_();
  }
  if(centroidBodyKine.linVel.isSet())
  {
    centroidBodyKine.linVel() -= comd_();
  }
  if(centroidBodyKine.linAcc.isSet())
  {
    centroidBodyKine.linAcc() -= comdd_();
  }

  return Kinematics(Kinematics(worldCentroidStateKinematics_),
                    centroidBodyKine); /// product of the kinematics -> worldBodyKine
}

Vector6 KineticsObserver::getContactWrench(Index contactNbr) const
{
  return worldCentroidStateVector_.segment<sizeWrench>(contactWrenchIndex(contactNbr));
}

kine::Kinematics KineticsObserver::getContactPose(Index contactNbr) const
{
  return Kinematics(worldCentroidStateVector_.segment<sizeStateKine>(contactKineIndex(contactNbr)), flagsContactKine);
}

Vector6 KineticsObserver::getUnmodeledWrench() const
{
  return worldCentroidStateVector_.segment<sizeWrench>(unmodeledWrenchIndex());
}

Vector6 KineticsObserver::getUnmodeledWrenchIn(const kine::Kinematics & userTargetframeKine)
{
  Vector3 forceCentroidFrame = getUnmodeledWrench().segment<sizeForce>(0);
  Vector3 torqueCentroidFrame = getUnmodeledWrench().segment<sizeTorque>(sizeForce);

  const Vector3 & userCentroidPosition = com_();
  Matrix3 targetUserOri = userTargetframeKine.orientation.inverse();
  Vector3 targetUserPosition = -(targetUserOri * userTargetframeKine.position());

  // lever arm between the target frame's origin and the centroid, expressed in the target frame
  const Vector3 targetCentroidPosition = targetUserPosition + targetUserOri * userCentroidPosition;

  // the orientation of the centroid frame in the user frame is the identity matrix
  Matrix3 & targetCentroidOri = targetUserOri;

  // expression of the wrench in the target frame
  Vector3 forceTargetframe = targetCentroidOri * forceCentroidFrame;
  Vector3 torqueTargetframe = targetCentroidOri * torqueCentroidFrame + targetCentroidPosition.cross(forceTargetframe);

  Vector6 wrenchTargetframe;
  wrenchTargetframe << forceTargetframe, torqueTargetframe;

  return wrenchTargetframe;
}

kine::LocalKinematics KineticsObserver::estimateAccelerations()
{
  Vector3 forceCentroid = additionalForce_;
  Vector3 torqueCentroid = additionalTorque_;

  addUnmodeledAndContactWrench_(worldCentroidStateVector_, forceCentroid, torqueCentroid);

  /// The accelerations are about to be computed so we set them to "initialized"
  computeLocalAccelerations_(worldCentroidStateKinematics_, forceCentroid, torqueCentroid,
                             worldCentroidStateKinematics_.linAcc.set(), worldCentroidStateKinematics_.angAcc.set());

  return worldCentroidStateKinematics_;
}

void KineticsObserver::setWorldCentroidStateKinematics(const LocalKinematics & localKine,
                                                       bool resetForces,
                                                       bool resetCovariance)
{
  BOOST_ASSERT(localKine.position.isSet() && localKine.orientation.isSet() && localKine.linVel.isSet()
               && localKine.angVel.isSet()
               && "The Kinematics is not correctly initialized, should be the position, orientation, and linear and "
                  "angular verlocities");
  worldCentroidStateKinematics_ = localKine;
  worldCentroidStateVector_.segment<sizeStateKine>(kineIndex()) =
      worldCentroidStateKinematics_.toVector(flagsStateKine);

  if(resetForces)
  {
    for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
    {
      if(i->isSet)
      {
        worldCentroidStateVector_.segment<sizeWrench>(contactWrenchIndex(i)).setZero();
      }
    }
  }

  ekf_.setState(worldCentroidStateVector_, k_est_);

  if(resetCovariance)
  {
    Matrix stateCovariance = ekf_.getStateCovariance();
    resetStateCovarianceMat();
    setBlockStateCovariance<sizeStateKineTangent>(stateCovariance, stateKinematicsInitCovMat_, kineIndex());

    if(resetForces)
    {
      for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
      {
        if(i->isSet)
        {
          setBlockStateCovariance<sizeContact>(stateCovariance, contactInitCovMatDefault_, contactIndex(i));
        }
      }
    }
    ekf_.setStateCovariance(stateCovariance);
  }
}

void KineticsObserver::setWorldCentroidStateKinematics(const Kinematics & kine, bool resetCovariance)
{
  LocalKinematics localKine = LocalKinematics(kine);
  BOOST_ASSERT(kine.position.isSet() && kine.orientation.isSet() && kine.linVel.isSet() && kine.angVel.isSet()
               && "The Kinematics is not correctly initialized, should be the position, orientation, and linear and "
                  "angular verlocities");
  worldCentroidStateKinematics_ = localKine;
  worldCentroidStateVector_.segment<sizeStateKine>(kineIndex()) =
      worldCentroidStateKinematics_.toVector(flagsStateKine);

  ekf_.setState(worldCentroidStateVector_, k_est_);

  if(resetCovariance)
  {
    Matrix stateCovariance = ekf_.getStateCovariance();
    setBlockStateCovariance<sizeStateKineTangent>(stateCovariance, stateKinematicsInitCovMat_, kineIndex());

    ekf_.setStateCovariance(stateCovariance);
  }
}

void KineticsObserver::setGyroBias(const Vector3 & bias, unsigned numberOfIMU, bool resetCovariance)
{
  worldCentroidStateVector_.segment<sizeGyroBias>(gyroBiasIndex(numberOfIMU)) = bias;
  ekf_.setState(worldCentroidStateVector_, k_est_);

  if(resetCovariance)
  {
    Matrix stateCovariance = ekf_.getStateCovariance();
    setBlockStateCovariance<sizeGyroBias>(stateCovariance, gyroBiasInitCovMat_, gyroBiasIndex(numberOfIMU));

    ekf_.setStateCovariance(stateCovariance);
  }
}

void KineticsObserver::setStateUnmodeledWrench(const Vector6 & wrench, bool resetCovariance)
{
  worldCentroidStateVector_.segment<sizeWrench>(unmodeledWrenchIndex()) = wrench;
  ekf_.setState(worldCentroidStateVector_, k_est_);

  if(resetCovariance)
  {
    Matrix stateCovariance = ekf_.getStateCovariance();
    setBlockStateCovariance<sizeWrench>(stateCovariance, unmodeledWrenchInitCovMat_, unmodeledWrenchIndex());

    ekf_.setStateCovariance(stateCovariance);
  }
}

void KineticsObserver::setStateVector(const Vector & v, bool resetCovariance)
{
  worldCentroidStateVector_ = v;
  ekf_.setState(v, k_est_);

  updateLocalKineAndContacts_();

  if(resetCovariance)
  {
    resetStateCovarianceMat();
  }
  oldWorldCentroidStateVector_ = worldCentroidStateVector_;
}

void KineticsObserver::setAdditionalWrench(const Vector3 & forceUserFrame, const Vector3 & momentUserFrame)
{
  startNewIteration_();
  convertWrenchFromUserToCentroid(forceUserFrame, momentUserFrame, additionalForce_, additionalTorque_);
}

void KineticsObserver::convertWrenchFromUserToCentroid(const Vector3 & forceUserFrame,
                                                       const Vector3 & momentUserFrame,
                                                       Vector3 & forceCentroidFrame,
                                                       Vector3 & momentCentroidFrame)
{
  forceCentroidFrame = forceUserFrame;
  momentCentroidFrame = momentUserFrame - com_().cross(forceUserFrame);
}

void KineticsObserver::convertWrenchFromCentroidToUser(const Vector3 & forceCentroidFrame,
                                                       const Vector3 & momentCentroidFrame,
                                                       Vector3 & forceUserFrame,
                                                       Vector3 & momentUserFrame)
{
  forceUserFrame = forceCentroidFrame;
  momentUserFrame = momentCentroidFrame + com_().cross(forceCentroidFrame);
}

void KineticsObserver::setWithUnmodeledWrench(bool b)
{
  withUnmodeledWrench_ = b;
}

void KineticsObserver::setWithAccelerationEstimation(bool b)
{
  withAccelerationEstimation_ = b;
}

bool KineticsObserver::getWithAccelerationEstimation() const
{
  return withAccelerationEstimation_;
}

void KineticsObserver::setWithGyroBias(bool b)
{
  withGyroBias_ = b;
}

void KineticsObserver::setWithDampingInMatrixA(bool b)
{
  withDampingInMatrixA_ = b;
}

void KineticsObserver::setWithAdaptativeContactProcessCov(bool b)
{
  withAdaptativeContactProcessCov_ = b;
}

bool KineticsObserver::getWithAdaptativeContactProcessCov() const
{
  return withAdaptativeContactProcessCov_;
}

Index KineticsObserver::setIMU(const Vector3 & accelero,
                               const Vector3 & gyrometer,
                               const Kinematics & userImuKinematics,
                               Index num)
{
  return setIMU(accelero, gyrometer, userImuKinematics, num, nullptr, nullptr);
}

Index KineticsObserver::setIMU(const Vector3 & accelero,
                               const Vector3 & gyrometer,
                               const Matrix3 & acceleroCov,
                               const Matrix3 & gyroCov,
                               const Kinematics & userImuKinematics,
                               Index num)
{
  return setIMU(accelero, gyrometer, userImuKinematics, num, &acceleroCov, &gyroCov);
}

Index KineticsObserver::setIMU(const Vector3 & accelero,
                               const Vector3 & gyrometer,
                               const Kinematics & userImuKinematics,
                               Index num,
                               const Matrix3 * acceleroCov,
                               const Matrix3 * gyroCov)
{
  BOOST_ASSERT((acceleroCov == nullptr || (acceleroCov != nullptr && gyroCov != nullptr))
               && "Wrong usage of internal setIMU");
  /// ensure the measuements are labeled with the good time stamp
  startNewIteration_();

  if(num < 0)
  {
    num = 0;
    while(imuSensors_[static_cast<size_t>(num)].time != k_data_ && static_cast<size_t>(num) < imuSensors_.size())
    {
      ++num;
    }
  }

  BOOST_ASSERT(unsigned(num) < maxImuNumber_ && "The inserted IMU number exceeds the maximum number");

  IMU & imu = imuSensors_[static_cast<size_t>(num)]; /// reference

  BOOST_ASSERT(imu.time < k_data_ && "The IMU has been already set, use another number");

  imu.stateIndex = angVelIndex() + sizeAngVel + sizeGyroBias * num;
  imu.stateIndexTangent = angVelIndexTangent() + sizeAngVelTangent + sizeGyroBiasTangent * num;

  imu.acceleroGyro.head<3>() = accelero;
  imu.acceleroGyro.tail<3>() = gyrometer;
  if(acceleroCov)
  {
    imu.covMatrixAccelero = *acceleroCov;
    imu.covMatrixGyro = *gyroCov;
  }

  if(imu.time == 0) /// this is the first value for the IMU
  {
    imu.userImuKinematics = userImuKinematics;
    imu.centroidImuKinematics = LocalKinematics(convertUserToCentroidFrame_(imu.userImuKinematics, k_data_));
    if(!acceleroCov)
    {
      imu.covMatrixAccelero = acceleroCovMatDefault_;
      imu.covMatrixGyro = gyroCovMatDefault_;
    }
    BOOST_ASSERT(imu.centroidImuKinematics.position.isSet() && imu.centroidImuKinematics.orientation.isSet()
                 && "The kinematics of the IMU is incorrectly initialized");
    if(!imu.centroidImuKinematics.linVel.isSet())
    {
      imu.centroidImuKinematics.linVel.set().setZero();
    }
    if(!imu.centroidImuKinematics.angVel.isSet())
    {
      imu.centroidImuKinematics.angVel.set().setZero();
    }
    if(!imu.centroidImuKinematics.linAcc.isSet())
    {
      imu.centroidImuKinematics.linAcc.set().setZero();
    }
    if(!imu.centroidImuKinematics.angAcc.isSet())
    {
      imu.centroidImuKinematics.angAcc.set().setZero();
    }
  }
  else
  {
    imu.userImuKinematics.update(userImuKinematics, dt_ * static_cast<double>(k_data_ - imu.time), flagsIMUKine);
    imu.centroidImuKinematics = LocalKinematics(convertUserToCentroidFrame_(imu.userImuKinematics, k_data_));
  }

  imu.time = k_data_;

  ++currentIMUSensorNumber_;

  return num;
}

void KineticsObserver::setIMUDefaultCovarianceMatrix(const Matrix3 & acceleroCov, const Matrix3 & gyroCov)
{
  acceleroCovMatDefault_ = acceleroCov;
  gyroCovMatDefault_ = gyroCov;
}

void KineticsObserver::updateContactWithWrenchSensor(const Vector6 & wrenchMeasurement,
                                                     const Kinematics & userContactKine,
                                                     unsigned contactNumber)
{
  /// ensure the measuements are labeled with the good time stamp
  startNewIteration_();

  BOOST_ASSERT(contactNumber < maxContacts_ && "Tried to set the wrench of a contact number higher than the maximum.");

  BOOST_ASSERT((contacts_[contactNumber].isSet) && "Tried to set the wrench of non-existing contact. \
                                            The contact must be added BEFORE setting a contact wrench Sensor");

  if(contacts_[contactNumber].time == k_data_ - 1) /// the contact is not newly set
  {
    contacts_[contactNumber].userContactKine.update(userContactKine, dt_, Contact::contactKineFlags);
    convertUserToCentroidFrame_(contacts_[contactNumber].userContactKine, contacts_[contactNumber].centroidContactKine,
                                k_data_);
    // we convert the contact's kinematics from the user frame to the centroid's frame
  }
  else /// the contact is newly set
  {
    contacts_[contactNumber].userContactKine = userContactKine;
    convertUserToCentroidFrame_(contacts_[contactNumber].userContactKine, contacts_[contactNumber].centroidContactKine,
                                k_data_);
    // we convert the contact's kinematics from the user frame to the centroid's frame
  }
  contacts_[contactNumber].wrenchMeasurement = wrenchMeasurement;
  contacts_[contactNumber].time = k_data_;

  if(!contacts_[contactNumber].sensorCovMatrix.isSet())
  {
    contacts_[contactNumber].sensorCovMatrix = contactWrenchSensorCovMatDefault_;
  }

  if(!(contacts_[contactNumber].withRealSensor))
  {
    contacts_[contactNumber].withRealSensor = true;
    numberOfContactRealSensors_++;
  }
}

void KineticsObserver::updateContactWithWrenchSensor(const Vector6 & wrenchMeasurement,
                                                     const Matrix6 & wrenchCovMatrix,
                                                     const Kinematics & userContactKine,
                                                     unsigned contactNumber)
{
  /// ensure the measuements are labeled with the good time stamp
  startNewIteration_();

  BOOST_ASSERT(contactNumber < maxContacts_ && "Tried to set the wrench of a contact number higher than the maximum.");

  BOOST_ASSERT((contacts_[contactNumber].isSet) && "Tried to set the wrench of non-existing contact. \
                                            The contact must be added BEFORE setting a contact wrench Sensor");

  if(contacts_[contactNumber].time == k_data_ - 1) /// the contact is not newly set
  {
    contacts_[contactNumber].userContactKine.update(userContactKine, dt_, Contact::contactKineFlags);
    convertUserToCentroidFrame_(contacts_[contactNumber].userContactKine, contacts_[contactNumber].centroidContactKine,
                                k_data_);
    // we convert the contact's kinematics from the user frame to the centroid's frame
  }
  else /// the contact is newlyset
  {
    contacts_[contactNumber].userContactKine = userContactKine;
    convertUserToCentroidFrame_(contacts_[contactNumber].userContactKine, contacts_[contactNumber].centroidContactKine,
                                k_data_);
    // we convert the contact's kinematics from the user frame to the centroid's frame
  }
  contacts_[contactNumber].wrenchMeasurement = wrenchMeasurement;
  contacts_[contactNumber].time = k_data_;
  contacts_[contactNumber].sensorCovMatrix = wrenchCovMatrix;

  if(!(contacts_[contactNumber].withRealSensor))
  {
    contacts_[contactNumber].withRealSensor = true;
    numberOfContactRealSensors_++;
  }
}

void KineticsObserver::setContactWrenchSensorDefaultCovarianceMatrix(const Matrix6 & wrenchSensorCovMat)
{
  contactWrenchSensorCovMatDefault_ = wrenchSensorCovMat;
}

void KineticsObserver::updateContactCovariances()
{
  Index nbCurrentContacts = getNumberOfSetContacts();

  if(((getNumberOfSetContacts() == nb_prevContacts_) && !contactRestPosProcessChanged_
      && !contactRestOriProcessChanged_)
     || getNumberOfSetContacts() == 0 || !withAdaptativeContactProcessCov_)
  {
    return;
  }

  Matrix processCovMat = ekf_.getQ();

  // exceptional case if there is only one contact!
  if(nbCurrentContacts == 1)
  {
    for(VectorContactConstIterator contact_it = contacts_.begin(); contact_it != contacts_.end(); ++contact_it)
    {
      if(contact_it->isSet)
      {
        processCovMat
            .block(contactPosIndexTangent(contact_it), contactPosIndexTangent(contact_it), sizePosTangent,
                   sizePosTangent)
            .setZero();
        processCovMat
            .block(contactOriIndexTangent(contact_it), contactOriIndexTangent(contact_it), sizeOriTangent,
                   sizeOriTangent)
            .setZero();

        ekf_.setQ(processCovMat);
        return;
      }
    }
  }

  if(contactRestPosProcessChanged_ || nbCurrentContacts != nb_prevContacts_)
  {
    Eigen::MatrixXd & M = m_matrices_.at(nbCurrentContacts - 2);

    Eigen::MatrixXd posProcessCov = Eigen::MatrixXd::Zero(nbCurrentContacts * 3, nbCurrentContacts * 3);

    int i = 0;
    for(auto & contact : contacts_)
    {
      if(contact.isSet)
      {
        posProcessCov.block(i * 3, i * 3, 3, 3) = contact.restPosProcessCovMat;
        i++;
      }
    }

    // cov(Mv) = M cov(v) M'. But here M is symmetric
    Eigen::MatrixXd covMv = M * posProcessCov * M;

    i = 0;
    for(VectorContactConstIterator contact1_it = contacts_.begin(); contact1_it != contacts_.end(); ++contact1_it)
    {
      if(contact1_it->isSet)
      {
        int j = 0;
        for(VectorContactConstIterator contact2_it = contacts_.begin(); contact2_it != contacts_.end(); ++contact2_it)
        {
          if(contact2_it->isSet)
          {
            processCovMat.block(contactPosIndexTangent(contact1_it), contactPosIndexTangent(contact2_it),
                                sizePosTangent, sizePosTangent) = covMv.block(i * 3, j * 3, 3, 3);
            j++;
          }
        }
        i++;
      }
    }
  }
  if(contactRestOriProcessChanged_ || nbCurrentContacts != nb_prevContacts_)
  {
    Eigen::MatrixXd & M_prime = m_prime_matrices_.at(nbCurrentContacts - 2);

    Eigen::MatrixXd oriProcessCov = Eigen::MatrixXd::Zero(nbCurrentContacts * 3, nbCurrentContacts * 3);

    int i = 0;
    for(auto & contact : contacts_)
    {
      if(contact.isSet)
      {
        oriProcessCov.block(i * 3, i * 3, 3, 3) = contact.restOriProcessCovMat;
        i++;
      }
    }

    // cov(Mv) = M_prime cov(v) M_prime'. But here M_prime is symmetric
    Eigen::MatrixXd covM_prime_v = M_prime * oriProcessCov * M_prime;

    i = 0;
    for(VectorContactConstIterator contact1_it = contacts_.begin(); contact1_it != contacts_.end(); ++contact1_it)
    {
      if(contact1_it->isSet)
      {
        int j = 0;
        for(VectorContactConstIterator contact2_it = contacts_.begin(); contact2_it != contacts_.end(); ++contact2_it)
        {
          if(contact2_it->isSet)
          {
            processCovMat.block(contactOriIndexTangent(contact1_it), contactOriIndexTangent(contact2_it), 3, 3)
                .setZero();
            // we select only the elements of the resulting matrix associated to the yaw as we add no process on the
            // roll and the pitch of the rest pose
            processCovMat(contactOriIndexTangent(contact1_it) + 2, contactOriIndexTangent(contact2_it) + 2) =
                covM_prime_v((i * 3) + 2, (j * 3) + 2);
            j++;
          }
        }
        i++;
      }
    }
  }
  ekf_.setQ(processCovMat);
}

void KineticsObserver::updateContactWithNoSensor(const Kinematics & userContactKine, unsigned contactNumber)
{
  /// ensure the measuements are labeled with the good time stamp
  startNewIteration_();

  BOOST_ASSERT(contactNumber < maxContacts_ && "Tried to set the wrench of a contact number higher than the maximum.");

  BOOST_ASSERT((contacts_[contactNumber].isSet) && "Tried to set the wrench of non-existing contact. \
                                            The contact must be added BEFORE setting a contact wrench Sensor");

  if(contacts_[contactNumber].time == k_data_ - 1) /// the contact is not newly set
  {
    contacts_[contactNumber].userContactKine.update(userContactKine, dt_, Contact::contactKineFlags);
    convertUserToCentroidFrame_(contacts_[contactNumber].userContactKine, contacts_[contactNumber].centroidContactKine,
                                k_data_);
    // we convert the contact's kinematics from the user frame to the centroid's frame
  }
  else /// the contact is newlyset
  {
    contacts_[contactNumber].userContactKine = userContactKine;
    convertUserToCentroidFrame_(contacts_[contactNumber].userContactKine, contacts_[contactNumber].centroidContactKine,
                                k_data_);
    // we convert the contact's kinematics from the user frame to the centroid's frame
  }

  contacts_[contactNumber].time = k_data_;

  if(contacts_[contactNumber].withRealSensor)
  {
    contacts_[contactNumber].withRealSensor = false;
    numberOfContactRealSensors_--;
  }
}

void KineticsObserver::setAbsolutePoseSensor(const Kinematics & pose)
{
  /// ensure the measuements are labeled with the good time stamp
  startNewIteration_();

  absPoseSensor_.time = k_data_;
  absPoseSensor_.pose = pose;

  if(!(absPoseSensor_.covMatrix.isSet()))
  {
    absPoseSensor_.covMatrix = absPoseSensorCovMatDefault_;
  }
}

void KineticsObserver::setAbsolutePoseSensor(const Kinematics & pose, const Matrix6 & CovarianceMatrix)
{
  /// ensure the measuements are labeled with the good time stamp
  startNewIteration_();

  absPoseSensor_.time = k_data_;
  absPoseSensor_.pose = pose;

  absPoseSensor_.covMatrix = CovarianceMatrix;
}

void KineticsObserver::setAbsolutePoseSensorDefaultCovarianceMatrix(const Matrix6 & newdefault)
{
  absPoseSensorCovMatDefault_ = newdefault;
}

void KineticsObserver::setAbsoluteOriSensor(const Orientation & ori)
{
  /// ensure the measuements are labeled with the good time stamp
  startNewIteration_();

  absOriSensor_.time = k_data_;
  absOriSensor_.ori = ori;

  if(!(absOriSensor_.covMatrix.isSet()))
  {
    absOriSensor_.covMatrix = absOriSensorCovMatDefault_;
  }
}

void KineticsObserver::setAbsoluteOriSensor(const Orientation & ori, const Matrix3 & CovarianceMatrix)
{
  /// ensure the measuements are labeled with the good time stamp
  startNewIteration_();

  absOriSensor_.time = k_data_;
  absOriSensor_.ori = ori;

  absOriSensor_.covMatrix = CovarianceMatrix;
}

void KineticsObserver::setAbsoluteOriSensorDefaultCovarianceMatrix(const Matrix3 & newdefault)
{
  absOriSensorCovMatDefault_ = newdefault;
}

void KineticsObserver::setCoMInertiaMatrix(const Matrix3 & I, const Matrix3 & I_dot)
{
  startNewIteration_();
  I_.set(I, k_data_);
  Id_.set(I_dot, k_data_);
}

void KineticsObserver::setCoMInertiaMatrix(const Matrix3 & I)
{
  startNewIteration_();

  if(I_.getTime() < k_data_)
  {
    Id_.set(tools::derivate(I_(), I, dt_ * double(k_data_ - I_.getTime())), k_data_);
  }
  I_.set(I, k_data_);
}

void KineticsObserver::setCoMInertiaMatrix(const Vector6 & Iv, const Vector6 & Iv_dot)
{
  startNewIteration_();

  I_.set();
  I_.setIndex(k_data_);
  fillSymmetricMatrix(I_(), Iv.head<3>(), Iv(3), Iv(4), Iv(5));

  Id_.set();
  Id_.setIndex(k_data_);
  fillSymmetricMatrix(Id_(), Iv_dot.head<3>(), Iv_dot(3), Iv_dot(4), Iv_dot(5));
}

void KineticsObserver::setCoMInertiaMatrix(const Vector6 & Iv)
{
  startNewIteration_();
  namespace t = tools;

  if(I_.getTime() < k_data_)
  {
    Id_.set();
    Id_.setIndex(k_data_);
    double dt = dt_ * double(k_data_ - I_.getTime());
    fillSymmetricMatrix(Id_(), t::derivate<Vector3>(I_().diagonal(), Iv.head<3>(), dt),
                        t::derivate(I_()(1, 0), Iv(3), dt), t::derivate(I_()(2, 0), Iv(4), dt),
                        t::derivate(I_()(2, 1), Iv(5), dt));
  }

  I_.set();
  I_.setIndex(k_data_);
  fillSymmetricMatrix(I_(), Iv.head<3>(), Iv(3), Iv(4), Iv(5));
}

void KineticsObserver::setCenterOfMass(const Vector3 & com, const Vector3 & com_dot, const Vector3 & com_dot_dot)
{
  startNewIteration_();
  com_.set(com, k_data_);
  comd_.set(com_dot, k_data_);
  comdd_.set(com_dot_dot, k_data_);
}

void KineticsObserver::setCenterOfMass(const Vector3 & com, const Vector3 & com_dot)
{
  startNewIteration_();
  com_.set(com, k_data_);

  if(comd_.getTime() < k_data_)
  {
    comdd_.set(tools::derivate(comd_(), com_dot, dt_ * double(k_data_ - comd_.getTime())), k_data_);
  }
  comd_.set(com_dot, k_data_);
}

void KineticsObserver::setCenterOfMass(const Vector3 & com)
{
  startNewIteration_();

  if(com_.getTime() < k_data_)
  {
    double dt = dt_ * double(k_data_ - com_.getTime());
    Vector3 com_dot = tools::derivate(com_(), com, dt);

    comdd_.set(tools::derivate(comd_(), com_dot, dt), k_data_);

    comd_.set(com_dot, k_data_);
  }

  com_.set(com, k_data_);
}

void KineticsObserver::setCoMAngularMomentum(const Vector3 & sigma, const Vector3 & sigma_dot)
{
  startNewIteration_();
  sigma_.set(sigma, k_data_);
  sigmad_.set(sigma_dot, k_data_);
}

void KineticsObserver::setCoMAngularMomentum(const Vector3 & sigma)
{
  startNewIteration_();
  if(sigma_.getTime() < k_data_)
  {
    sigmad_.set(tools::derivate(sigma_(), sigma, dt_ * double(k_data_ - sigma_.getTime())), k_data_);
  }
  sigma_.set(sigma, k_data_);
}

Index KineticsObserver::addContact(const Kinematics & worldContactRefKine,
                                   const Matrix12 & initialCovarianceMatrix,
                                   const Matrix12 & processCovarianceMatrix,
                                   Index contactNumber,
                                   const Matrix3 & linearStiffness,
                                   const Matrix3 & linearDamping,
                                   const Matrix3 & angularStiffness,
                                   const Matrix3 & angularDamping)
{
  BOOST_ASSERT(worldContactRefKine.position.isSet() && worldContactRefKine.orientation.isSet()
               && "The added contact pose is not initialized correctly (position and orientation)");

  /// attributes the contact an index called contactNumber. Automatically attributes the first available number in the
  /// range defined by the maximum amount of contacts
  if(contactNumber < 0)
  {
    contactNumber = 0;

    while(unsigned(contactNumber) < maxContacts_ && contacts_[static_cast<size_t>(contactNumber)].isSet)
    {
      ++contactNumber;
    }
  }

  BOOST_ASSERT(unsigned(contactNumber) < maxContacts_
               && "Trying to add contact: The contact number exceeds the maximum allowed, please give a number of "
                  "contact between 0 and maxContact-1");

  /// this is a bug-prone protection code that is here only to guarantee the consistence of the state
  if(unsigned(contactNumber) >= maxContacts_)
  {
    contactNumber = maxContacts_ - 1;
  }

  BOOST_ASSERT(!contacts_[contactNumber].isSet
               && "The contact already exists, please remove it before adding it again");

  Contact & contact = contacts_[static_cast<size_t>(contactNumber)]; /// reference

  contact.isSet = true; /// set the contacts

  contact.stateIndex = contactsIndex() + contactNumber * sizeContact;
  contact.stateIndexTangent = contactsIndexTangent() + contactNumber * sizeContactTangent;
  contact.worldRestPose = worldContactRefKine;

  if(linearDamping != Matrix3::Constant(-1))
  {
    contact.linearStiffness = linearStiffness;
  }
  else
  {
    contact.linearStiffness = linearStiffnessMatDefault_;
  }

  if(linearDamping != Matrix3::Constant(-1))
  {
    contact.linearDamping = linearDamping;
  }
  else
  {
    contact.linearDamping = linearDampingMatDefault_;
  }

  if(angularStiffness != Matrix3::Constant(-1))
  {
    contact.angularStiffness = angularStiffness;
  }
  else
  {
    contact.angularStiffness = angularStiffnessMatDefault_;
  }

  if(angularDamping != Matrix3::Constant(-1))
  {
    contact.angularDamping = angularDamping;
  }
  else
  {
    contact.angularDamping = angularDampingMatDefault_;
  }

  /// update the state vector. The contact forces and moments are initialized to zero but are updated in update()
  worldCentroidStateVector_.segment<sizeContact>(contact.stateIndex)
      << contact.worldRestPose.toVector(flagsContactKine),
      Vector6::Zero();
  ekf_.setState(worldCentroidStateVector_, k_est_);

  /// sets the initial covariance matrix
  Matrix stateCovMat = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeContactTangent>(stateCovMat, initialCovarianceMatrix, contact.stateIndexTangent);
  ekf_.setStateCovariance(stateCovMat);

  /// Sets the process cov mat
  Matrix processCovMat = ekf_.getQ();
  setBlockStateCovariance<sizeContactTangent>(processCovMat, processCovarianceMatrix, contact.stateIndexTangent);
  contact.restPosProcessCovMat = processCovarianceMatrix.block<3, 3>(0, 0);
  contact.restOriProcessCovMat = processCovarianceMatrix.block<3, 3>(sizePosTangent, sizePosTangent);
  ekf_.setQ(processCovMat);

  return contactNumber;
}

/// version when the contact position is perfectly known
Index KineticsObserver::addContact(const Kinematics & worldContactRefKine,
                                   Index contactNumber,
                                   const Matrix3 & linearStiffness,
                                   const Matrix3 & linearDamping,
                                   const Matrix3 & angularStiffness,
                                   const Matrix3 & angularDamping)
{
  return addContact(worldContactRefKine, contactInitCovMatDefault_, contactProcessCovMatDefault_, contactNumber,
                    linearStiffness, linearDamping, angularStiffness, angularDamping);
}

void KineticsObserver::removeContact(Index contactNbr)
{
  BOOST_ASSERT(contacts_[contactNbr].isSet && "Tried to remove a non-existing contact.");
  auto & c = contacts_[static_cast<size_t>(contactNbr)];
  c.isSet = false;
  removedContacts_.insert(contactNbr);
  if(c.withRealSensor)
  {
    c.withRealSensor = false;
    --numberOfContactRealSensors_;
  }
}

void KineticsObserver::clearContacts()
{
  contacts_.clear();
  numberOfContactRealSensors_ = 0;
}

Index KineticsObserver::getNumberOfSetContacts() const
{
  Index out = 0;
  for(const auto & c : contacts_)
  {
    if(c.isSet)
    {
      out += 1;
    }
  }
  return out;
}

std::vector<Index> KineticsObserver::getListOfContacts() const
{
  std::vector<Index> v;
  for(unsigned i = 0; i < contacts_.size(); ++i)
  {
    if(contacts_[i].isSet)
    {
      v.push_back(i);
    }
  }
  return v;
}

// ////////////////////////////////////////////////////////////
///                 Covariance matrices
// ///////////////////////////////////////////////////////////

void KineticsObserver::setStateCovarianceMat(const Matrix & P)
{
  ekf_.setStateCovariance(P);
}

void KineticsObserver::setKinematicsInitCovarianceDefault(const Matrix & P_kine)
{
  stateKinematicsInitCovMat_ = P_kine;
}

void KineticsObserver::setKinematicsInitCovarianceDefault(const Matrix3 & P_pos,
                                                          const Matrix3 & P_ori,
                                                          const Matrix3 & P_linVel,
                                                          const Matrix3 & P_angVel)
{
  stateKinematicsInitCovMat_.setZero();

  statePosInitCovMat_ = P_pos;
  stateOriInitCovMat_ = P_ori;
  stateLinVelInitCovMat_ = P_linVel;
  stateAngVelInitCovMat_ = P_angVel;

  stateKinematicsInitCovMat_.block<sizePos, sizePos>(posIndexTangent(), posIndexTangent()) = statePosInitCovMat_;
  stateKinematicsInitCovMat_.block<sizeOriTangent, sizeOriTangent>(oriIndexTangent(), oriIndexTangent()) =
      stateOriInitCovMat_;
  stateKinematicsInitCovMat_.block<sizeLinVel, sizeLinVel>(linVelIndexTangent(), linVelIndexTangent()) =
      stateLinVelInitCovMat_;
  stateKinematicsInitCovMat_.block<sizeAngVel, sizeAngVel>(angVelIndexTangent(), angVelIndexTangent()) =
      stateAngVelInitCovMat_;
}

void KineticsObserver::setGyroBiasInitCovarianceDefault(const Matrix3 & covMat)
{
  gyroBiasInitCovMat_ = covMat;
}

void KineticsObserver::setUnmodeledWrenchInitCovMatDefault(const Matrix6 & initCovMat)
{
  unmodeledWrenchInitCovMat_ = initCovMat;
}

void KineticsObserver::setContactInitCovMatDefault(const Matrix12 & contactCovMat)
{
  contactInitCovMatDefault_ = contactCovMat;
}

void KineticsObserver::setKinematicsStateCovariance(const Matrix & P_kine)
{
  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeStateKineTangent>(P, P_kine, kineIndexTangent());
  ekf_.setStateCovariance(P);
}

void KineticsObserver::setGyroBiasStateCovariance(const Matrix3 & covMat, unsigned imuNumber)
{
  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeGyroBias>(P, covMat, gyroBiasIndexTangent(imuNumber));
  ekf_.setStateCovariance(P);
}

void KineticsObserver::setUnmodeledWrenchStateCovMat(const Matrix6 & currentCovMat)
{
  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeWrench>(P, currentCovMat, unmodeledWrenchIndexTangent());
  ekf_.setStateCovariance(P);
}

void KineticsObserver::setContactStateCovMat(Index contactNbr, const Matrix12 & contactCovMat)
{
  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeContactTangent>(P, contactCovMat, contactIndexTangent(contactNbr));
  ekf_.setStateCovariance(P);
}

void KineticsObserver::setKinematicsProcessCovarianceDefault(const Matrix12 & P_kine)
{
  stateKinematicsProcessCovMat_ = P_kine;
}
void KineticsObserver::setKinematicsProcessCovarianceDefault(const Matrix3 & P_pos,
                                                             const Matrix3 & P_ori,
                                                             const Matrix3 & P_linVel,
                                                             const Matrix3 & P_angVel)
{
  stateKinematicsProcessCovMat_.setZero();

  statePosProcessCovMat_ = P_pos;
  stateOriProcessCovMat_ = P_ori;
  stateLinVelProcessCovMat_ = P_linVel;
  stateAngVelProcessCovMat_ = P_angVel;

  stateKinematicsProcessCovMat_.block<sizePos, sizePos>(posIndexTangent(), posIndexTangent()) = statePosProcessCovMat_;
  stateKinematicsProcessCovMat_.block<sizeOriTangent, sizeOriTangent>(oriIndexTangent(), oriIndexTangent()) =
      stateOriProcessCovMat_;
  stateKinematicsProcessCovMat_.block<sizeLinVel, sizeLinVel>(linVelIndexTangent(), linVelIndexTangent()) =
      stateLinVelProcessCovMat_;
  stateKinematicsProcessCovMat_.block<sizeAngVel, sizeAngVel>(angVelIndexTangent(), angVelIndexTangent()) =
      stateAngVelProcessCovMat_;
}

void KineticsObserver::setGyroBiasProcessCovarianceDefault(const Matrix3 & covMat)
{
  gyroBiasProcessCovMat_ = covMat;
}
void KineticsObserver::setUnmodeledWrenchProcessCovarianceDefault(const Matrix6 & covMat)
{
  unmodeledWrenchProcessCovMat_ = covMat;
}

void KineticsObserver::setContactProcessCovarianceDefault(const Matrix12 & covMat)
{
  contactProcessCovMatDefault_ = covMat;
}

void KineticsObserver::setKinematicsProcessCovariance(const Matrix12 & covMat)
{
  Matrix P = ekf_.getProcessCovariance();
  setBlockStateCovariance<sizeStateKine>(P, covMat, kineIndexTangent());
  ekf_.setProcessCovariance(P);
}

void KineticsObserver::setGyroBiasProcessCovariance(const Matrix3 & covMat, unsigned imuNumber)
{
  Matrix P = ekf_.getProcessCovariance();
  setBlockStateCovariance<sizeGyroBias>(P, covMat, gyroBiasIndexTangent(imuNumber));
  ekf_.setProcessCovariance(P);
}

void KineticsObserver::setUnmodeledWrenchProcessCovMat(const Matrix6 & processCovMat)
{
  Matrix P = ekf_.getProcessCovariance();
  setBlockStateCovariance<sizeWrench>(P, processCovMat, unmodeledWrenchIndexTangent());
  ekf_.setProcessCovariance(P);
}

void KineticsObserver::setContactProcessCovMat(Index contactNbr, const Matrix12 & contactCovMat)
{
  if((contactCovMat.block(0, 0, sizePosTangent, contactCovMat.cols()).array() != 0.0).any())
  {
    contactRestPosProcessChanged_ = true;
    contactRestOriProcessChanged_ = true;
  }

  Matrix P = ekf_.getProcessCovariance();
  setBlockStateCovariance<sizeContactTangent>(P, contactCovMat, contactIndexTangent(contactNbr));
  ekf_.setProcessCovariance(P);
}

Matrix KineticsObserver::getStateCovarianceMat() const
{
  return ekf_.getStateCovariance();
}

void KineticsObserver::setProcessNoiseCovarianceMat(const Matrix & Q)
{
  ekf_.setProcessCovariance(Q);
}

Vector KineticsObserver::getMeasurementVector()
{
  Vector measurement(getMeasurementSize());
  Index currIndex = 0;
  if(k_est_ != k_data_)
  {
    for(VectorIMUIterator i = imuSensors_.begin(); i != imuSensors_.end(); ++i)
    {
      if(i->time == k_data_)
      {
        measurement.segment<sizeIMUSignal>(currIndex) = i->acceleroGyro;
        currIndex += sizeIMUSignal;
      }
    }

    for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
    {
      if(i->isSet)
      {
        if(i->time == k_data_ && i->withRealSensor)
        {
          measurement.segment<sizeWrench>(currIndex) = i->wrenchMeasurement;
          currIndex += sizeWrench;
        }
      }
    }

    if(absPoseSensor_.time == k_data_)
    {
      measurement.segment<sizePose>(currIndex) = absPoseSensor_.pose.toVector(flagsPoseKine);
      currIndex += sizePose;
    }
    if(absOriSensor_.time == k_data_)
    {
      measurement.segment<sizeOri>(currIndex) = absOriSensor_.ori.toVector4();
    }
  }
  return measurement;
}

const ExtendedKalmanFilter & KineticsObserver::getEKF() const
{
  return ekf_;
}

ExtendedKalmanFilter & KineticsObserver::getEKF()
{
  return ekf_;
}

void KineticsObserver::resetStateCovarianceMat()
{
  resetStateKinematicsCovMat();
  for(unsigned i = 0; i < imuSensors_.size(); ++i)
  {
    if(imuSensors_[i].time == k_data_)
    {
      resetStateGyroBiasCovMat(i);
    }
  }
  resetStateUnmodeledWrenchCovMat();
  resetStateContactsCovMat();
}

void KineticsObserver::resetStateKinematicsCovMat()
{
  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeStateKineTangent>(P, stateKinematicsInitCovMat_, kineIndexTangent());
  ekf_.setStateCovariance(P);
}

void KineticsObserver::resetStateGyroBiasCovMat(Index i)
{
  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeGyroBias>(P, gyroBiasInitCovMat_, gyroBiasIndexTangent(i));
  ekf_.setStateCovariance(P);
}

void KineticsObserver::resetStateUnmodeledWrenchCovMat()
{
  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeWrench>(P, unmodeledWrenchInitCovMat_, unmodeledForceIndexTangent());
  ekf_.setStateCovariance(P);
}

void KineticsObserver::resetStateContactsCovMat()
{
  for(unsigned i = 0; i < contacts_.size(); ++i)
  {
    if(contacts_[i].isSet)
    {
      resetStateContactCovMat(i);
    }
  }
}

void KineticsObserver::resetStateContactCovMat(Index contactNbr)
{
  BOOST_ASSERT(contactNbr < contacts_.size() && contacts_[contactNbr].isSet
               && "Tried to set the covariance of a non existant contact");

  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeContactTangent>(P, contactInitCovMatDefault_,
                                              contacts_[static_cast<size_t>(contactNbr)].stateIndexTangent);
  ekf_.setStateCovariance(P);
}

void KineticsObserver::resetProcessCovarianceMat()
{
  resetProcessKinematicsCovMat();
  for(unsigned i = 0; i < imuSensors_.size(); ++i)
  {
    resetProcessGyroBiasCovMat(i);
  }
  resetProcessUnmodeledWrenchCovMat();
  resetProcessContactsCovMat();
}

void KineticsObserver::resetProcessKinematicsCovMat()
{
  Matrix P = ekf_.getProcessCovariance();
  setBlockStateCovariance<sizeContactTangent>(P, stateKinematicsProcessCovMat_, kineIndexTangent());
  ekf_.setProcessCovariance(P);
}

void KineticsObserver::resetProcessGyroBiasCovMat(Index i)
{
  Matrix P = ekf_.getProcessCovariance();
  setBlockStateCovariance<sizeGyroBias>(P, gyroBiasProcessCovMat_, gyroBiasIndexTangent(i));
  ekf_.setProcessCovariance(P);
}

void KineticsObserver::resetProcessUnmodeledWrenchCovMat()
{
  Matrix P = ekf_.getProcessCovariance();
  setBlockStateCovariance<sizeWrench>(P, unmodeledWrenchProcessCovMat_, unmodeledForceIndexTangent());
  ekf_.setProcessCovariance(P);
}

Index KineticsObserver::getInputSize() const
{
  return inputSize;
}

void KineticsObserver::resetProcessContactsCovMat()
{
  for(unsigned i = 0; i < contacts_.size(); ++i)
  {
    if(contacts_[i].isSet)
    {
      resetProcessContactCovMat(i);
    }
  }
}

void KineticsObserver::resetProcessContactCovMat(Index contactNbr)
{
  BOOST_ASSERT(contactNbr < maxContacts_ && contacts_[contactNbr].isSet
               && "Tried to set the covariance of a non existant contact");

  Matrix P = ekf_.getProcessCovariance();
  setBlockStateCovariance<sizeContactTangent>(P, contactProcessCovMatDefault_,
                                              contacts_[static_cast<size_t>(contactNbr)].stateIndexTangent);
  ekf_.setProcessCovariance(P);
}

void KineticsObserver::resetSensorsDefaultCovMats()
{
  acceleroCovMatDefault_ = Matrix3::Identity() * acceleroVarianceDefault;
  gyroCovMatDefault_ = Matrix3::Identity() * gyroVarianceDefault;
  contactWrenchSensorCovMatDefault_ = blockMat6(Matrix3::Identity() * forceSensorVarianceDefault, Matrix3::Zero(),
                                                Matrix3::Zero(), Matrix3::Identity() * torqueSensorVarianceDefault);
  absPoseSensorCovMatDefault_ = blockMat6(Matrix3::Identity() * positionSensorVarianceDefault, Matrix3::Zero(),
                                          Matrix3::Zero(), Matrix3::Identity() * orientationSensorVarianceDefault);
  absOriSensorCovMatDefault_ = Matrix3::Identity() * orientationSensorVarianceDefault;
}

void KineticsObserver::resetInputs()
{
  for(VectorIMUIterator i = imuSensors_.begin(); i != imuSensors_.end(); ++i)
  {
    i->time = k_est_;
  }

  for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    i->time = k_est_;
  }

  absPoseSensor_.time = k_est_;
  absOriSensor_.time = k_est_;
}

void KineticsObserver::setFiniteDifferenceStep(const Vector & v)
{
  worldCentroidStateVectorDx_ = v;
}

void KineticsObserver::useFiniteDifferencesJacobians(bool b)
{
  finiteDifferencesJacobians_ = b;
}

void KineticsObserver::setStateContact(Index index,
                                       Kinematics worldContactRestPose,
                                       const Vector6 & wrench,
                                       bool resetCovariance)
{
  Contact & contact = contacts_[static_cast<size_t>(index)];

  BOOST_ASSERT(contact.isSet && "The contact is currently not set");
  worldCentroidStateVector_.segment<sizePose>(contactPosIndex(index)) =
      worldContactRestPose.toVector().segment<sizePose>(0);

  worldCentroidStateVector_.segment<sizeWrench>(contactWrenchIndex(index)) = wrench;

  contact.worldRestPose.fromVector(worldCentroidStateVector_.segment<sizePose>(contactPosIndex(index)),
                                   flagsContactKine);

  if(resetCovariance)
  {
    resetStateContactCovMat(index);
  }

  ekf_.setState(worldCentroidStateVector_, k_est_);
}

void KineticsObserver::stateNaNCorrection_()
{
  nanDetected_ = true;

  /// TODO implement this function
  assert(false && "NaN Correction not yet implemented. Please Contact mehdi.benallegue@gmail.com");
}

void KineticsObserver::startNewIteration_()
{
  if(k_est_ == k_data_)
  {
    ++k_data_;
    numberOfContactRealSensors_ = 0;
    currentIMUSensorNumber_ = 0;
    for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
    {
      if(i->isSet)
      {
        i->withRealSensor = false;
      }
    }
    additionalForce_.setZero();
    additionalTorque_.setZero();
    contactRestPosProcessChanged_ = false;
    contactRestOriProcessChanged_ = false;
  }
}

void KineticsObserver::endIteration_()
{
  if(k_est_ != k_data_)
  {
    ++k_est_; // the timestamp of the state we estimated

    nb_prevContacts_ = getNumberOfSetContacts();

    removedContacts_.clear();
  }
}

void KineticsObserver::setProcessNoise(NoiseBase * noise)
{
  processNoise_ = noise;
}

void KineticsObserver::resetProcessNoise()
{
  processNoise_ = 0x0;
}

NoiseBase * KineticsObserver::getProcessNoise() const
{
  return processNoise_;
}

void KineticsObserver::setMeasurementNoise(NoiseBase * noise)
{
  measurementNoise_ = noise;
}

void KineticsObserver::resetMeasurementNoise()
{
  measurementNoise_ = 0x0;
}

NoiseBase * KineticsObserver::getMeasurementNoise() const
{
  return measurementNoise_;
}

Matrix KineticsObserver::computeAMatrix()
{
  // update of worldCentroidStateKinematics_ with the accelerations
  estimateAccelerations();

  const Vector & statePrediction = ekf_.updateStatePrediction();
  // position of the centroid frame in the world frame, predicted with the state-transition model
  const Vector3 & predictedWorldCentroidStatePos = statePrediction.segment<sizePos>(posIndex());
  // orientation of the centroid frame in the world frame, predicted with the state-transition model
  Orientation predictedWorldCentroidStateOri;
  predictedWorldCentroidStateOri.fromVector4(statePrediction.segment<sizeOri>(oriIndex())).toMatrix3();
  // linear velocity of the centroid frame in the world frame, predicted with the state-transition model
  const Vector3 & predictedWorldCentroidStateLinVel = statePrediction.segment<sizeLinVel>(linVelIndex());
  // angular velocity of the centroid frame in the world frame, predicted with the state-transition model
  const Vector3 & predictedWorldCentroidStateAngVel = statePrediction.segment<sizeAngVel>(angVelIndex());

  Matrix A = Matrix::Zero(stateTangentSize_, stateTangentSize_);

  LocalKinematics & stateKine_k = worldCentroidStateKinematics_;

  double dt2_2 = 0.5 * pow(dt_, 2);

  Vector3 rotVec = dt_ * stateKine_k.angVel() + dt2_2 * stateKine_k.angAcc();
  Vector3 transVec = dt_ * stateKine_k.linVel() + dt2_2 * stateKine_k.linAcc();

  double sq_norm_rotVec = rotVec.squaredNorm();
  double norm_rotVec = rotVec.norm();
  double sin_rotVec_2 = sin(0.5 * norm_rotVec);
  double sin_rotVec = sin(norm_rotVec);
  double cos_rotVec = cos(norm_rotVec);
  Matrix3 v_rotVec = kine::v_matrix(-rotVec);

  // Jacobians of the angular acceleration
  Matrix3 I_inv = I_().inverse();
  // we can merge the two last lines not to create a new variable. We consider that the inertia matrix is given in the
  // centroid's frame.
  Matrix3 J_omegadot_ext_torque = I_inv;

  Matrix3 J_omegadot_omega = I_inv
                             * (kine::skewSymmetric(I_() * stateKine_k.angVel()) - Id_()
                                - kine::skewSymmetric(stateKine_k.angVel()) * I_() + kine::skewSymmetric(sigma_()));

  // Jacobians of the linear acceleration
  Matrix3 J_al_R =
      -cst::gravityConstant * (stateKine_k.orientation.toMatrix3().transpose() * kine::skewSymmetric(Vector3(0, 0, 1)));

  //// creation of the variables linked to the unmodeled wrench. Initialized after if required. ////

  // jacobian matrix of the linear acceleration's state-transition wrt the external force
  Matrix3 J_al_ext_force;
  // jacobian matrix of the linear velocity's state-transition wrt the external force
  Matrix3 J_vl_ext_force;
  // jacobian matrix of the angular velocity's state-transition wrt the external torque
  Matrix3 J_omega_ext_torque;
  // jacobian matrix of the local position's state-transition wrt the external force
  Matrix3 J_pl_ext_force;
  // jacobian matrix of the local position's state-transition wrt the external torque
  Matrix3 J_pl_ext_torque;
  // jacobian matrix of the orientation's state-transition wrt the external torque
  Matrix3 J_R_ext_torque;

  //// Jacobian matrices of the local position's state transition ////

  // Jacobians of the position's state-transition wrt to itself
  Matrix3 J_pl_pl = kine::rotationVectorToRotationMatrix(-rotVec);

  // jacobian matrix of the local position's state-transition wrt the orientation
  Matrix3 J_pl_R = dt2_2 * v_rotVec * J_al_R;
  // jacobian matrix of the local position's state-transition wrt the linear velocity
  Matrix3 J_pl_vl = dt_ * v_rotVec;
  // jacobian matrix of the local position's state-transition wrt the linear acceleration
  Matrix3 J_pl_al = dt2_2 * v_rotVec;

  auto compute_J_rotated_rotvec = [norm_rotVec, cos_rotVec, sin_rotVec,
                                   sq_norm_rotVec](const Vector3 & rotVec, const Vector3 & rotated) -> Matrix3
  {
    Matrix3 jacob =
        (sin_rotVec / pow(norm_rotVec, 3) - cos_rotVec / sq_norm_rotVec) * rotated.cross(rotVec) * rotVec.transpose()
        - sin_rotVec / norm_rotVec * kine::skewSymmetric(rotated)
        - sin_rotVec / pow(norm_rotVec, 3) * rotVec.cross(rotated.cross(rotVec)) * rotVec.transpose()
        - 2 / pow(norm_rotVec, 4) * (1 - cos_rotVec) * (rotVec * rotVec.transpose()) * rotated * rotVec.transpose()
        + (1 - cos_rotVec) / sq_norm_rotVec
              * ((rotVec.transpose() * rotated) * Matrix3::Identity() + rotVec * rotated.transpose());

    return jacob;
  };

  auto compute_J_rotated_V = [norm_rotVec, sq_norm_rotVec,
                              &compute_J_rotated_rotvec](const Vector3 & rotVec, const Vector3 & rotated) -> Matrix3
  {
    Vector3 Rv = kine::rotationVectorToRotationMatrix(rotVec) * rotated;

    Matrix3 jacob = -2 / pow(norm_rotVec, 4)
                        * (rotVec * rotVec.transpose() * rotated + rotVec.cross(rotated) - rotVec.cross(Rv))
                        * rotVec.transpose()
                    + 1 / sq_norm_rotVec
                          * (rotVec.transpose() * rotated * Matrix3::Identity() + rotVec * rotated.transpose()
                             - kine::skewSymmetric(rotated) + kine::skewSymmetric(Rv)
                             - kine::skewSymmetric(rotVec) * compute_J_rotated_rotvec(rotVec, rotated));

    return jacob;
  };

  Matrix3 J_pl_rotVec =
      compute_J_rotated_rotvec(-rotVec, stateKine_k.position()) + compute_J_rotated_V(-rotVec, transVec);
  Matrix3 J_pl_omega = -dt_ * J_pl_rotVec - dt2_2 * J_pl_rotVec * J_omegadot_omega;

  Matrix3 J_pl_omega_dot = -dt2_2 * J_pl_rotVec;

  A.block<sizePosTangent, sizePosTangent>(posIndexTangent(), posIndexTangent()) = J_pl_pl;
  A.block<sizePosTangent, sizeOriTangent>(posIndexTangent(), oriIndexTangent()) = J_pl_R;
  A.block<sizePosTangent, sizeLinVelTangent>(posIndexTangent(), linVelIndexTangent()) = J_pl_vl;
  A.block<sizePosTangent, sizeAngVelTangent>(posIndexTangent(), angVelIndexTangent()) = J_pl_omega;

  //// Jacobian matrices of the orientation's state transition ////

  // jacobian matrix of the orientation's state-transition wrt rotVec
  Matrix3 J_R_rotVec;
  if(norm_rotVec > cst::epsilonAngle)
  {
    J_R_rotVec.noalias() =
        2.0 / norm_rotVec
        * (((norm_rotVec - 2.0 * sin_rotVec_2) / (2.0 * sq_norm_rotVec))
               * (stateKine_k.orientation.toMatrix3() * rotVec * rotVec.transpose())
           + sin_rotVec_2 * (stateKine_k.orientation.toMatrix3() * kine::rotationVectorToRotationMatrix(0.5 * rotVec)));
  }
  else
  {
    J_R_rotVec.noalias() = stateKine_k.orientation.toMatrix3() * kine::rotationVectorToRotationMatrix(0.5 * rotVec);
  }

  // R wrt omegadot. The intermediate jacobian used to compute the ones with respect to the angular velocity and
  // acceleration
  Matrix3 J_R_omegadot = J_R_rotVec * dt2_2; // used in other Jacobians

  // jacobian matrix of the orientation's state-transition wrt itself
  Matrix3 J_R_R = Matrix::Identity(sizeOriTangent, sizeOriTangent);
  // jacobian matrix of the orientation's state-transition wrt the local angular velocity
  Matrix3 J_R_omega =
      J_R_rotVec * (dt_ * Matrix::Identity(sizeAngVelTangent, sizeAngVelTangent) + dt2_2 * J_omegadot_omega);

  A.block<sizeOriTangent, sizeOriTangent>(oriIndexTangent(), oriIndexTangent()) = J_R_R;
  A.block<sizeOriTangent, sizeAngVelTangent>(oriIndexTangent(), angVelIndexTangent()) = J_R_omega;

  //// Jacobian matrices of the local linear velocity's state transition ////

  // jacobian matrix of the local linear velocity's state-transition wrt the orientation
  Matrix3 J_vl_R = dt_ * J_al_R;
  // jacobian matrix of the local linear velocity's state-transition wrt itself
  Matrix3 J_vl_vl =
      Matrix::Identity(sizeLinVelTangent, sizeLinVelTangent) - dt_ * kine::skewSymmetric(stateKine_k.angVel());
  // jacobian matrix of the local linear velocity's state-transition wrt the local angular velocity
  Matrix3 J_vl_omega = dt_ * kine::skewSymmetric(stateKine_k.linVel());

  A.block<sizeLinVelTangent, sizeOriTangent>(linVelIndexTangent(), oriIndexTangent()) = J_vl_R;
  A.block<sizeLinVelTangent, sizeLinVelTangent>(linVelIndexTangent(), linVelIndexTangent()) = J_vl_vl;
  A.block<sizeLinVelTangent, sizeAngVelTangent>(linVelIndexTangent(), angVelIndexTangent()) = J_vl_omega;

  //// Jacobian matrices of the local angular velocity's state transition ////

  // jacobian matrix of the local angular velocity's state-transition wrt itself
  Matrix3 J_omega_omega = Matrix3::Identity() + dt_ * J_omegadot_omega;
  A.block<sizeAngVelTangent, sizeAngVelTangent>(angVelIndexTangent(), angVelIndexTangent()) = J_omega_omega;

  //// Jacobian matrices of the local angular velocity's state transition ////
  if(withGyroBias_)
  {
    // jacobian matrix of the gyrometer bias' state-transition wrt itself
    Matrix3 J_gyrobias_gyrobias = Matrix::Identity(sizeGyroBiasTangent, sizeGyroBiasTangent);
    for(unsigned i = 0; i < imuSensors_.size(); ++i)
    {
      A.block<sizeGyroBiasTangent, sizeGyroBiasTangent>(gyroBiasIndexTangent(i), gyroBiasIndexTangent(i)) =
          J_gyrobias_gyrobias;
    }
  }

  //// Jacobian matrices wrt the unmodeled wrench ////
  if(withUnmodeledWrench_)
  {
    // jacobian matrix of the external force's state-transition wrt itself
    Matrix3 J_ext_force_ext_force = Matrix::Identity(sizeForceTangent, sizeForceTangent);
    // jacobian matrix of the linear acceleration wrt the external force
    J_al_ext_force = Matrix::Identity(sizeLinAccTangent, sizeTorqueTangent) / mass_;
    // jacobian matrix of the local position's state-transition wrt the external force
    J_pl_ext_force = dt2_2 / mass_ * v_rotVec;
    // jacobian matrix of the local linear velocity's state-transition wrt the external force
    J_vl_ext_force = dt_ * J_al_ext_force;
    // jacobian matrix of the external torque's state-transition wrt itself
    Matrix3 J_ext_torque_ext_torque = Matrix::Identity(sizeTorqueTangent, sizeTorqueTangent);
    // jacobian matrix of the local position's state-transition wrt the external torque
    J_pl_ext_torque = J_pl_omega_dot * J_omegadot_ext_torque;
    // jacobian matrix of the orientation's state-transition wrt the external torque
    J_R_ext_torque = J_R_omegadot * J_omegadot_ext_torque;
    // jacobian matrix of the local angular velocity's state-transition wrt the external torque
    J_omega_ext_torque = dt_ * J_omegadot_ext_torque;

    A.block<sizeForceTangent, sizeForceTangent>(unmodeledForceIndexTangent(), unmodeledForceIndexTangent()) =
        J_ext_force_ext_force;
    A.block<sizePosTangent, sizeForceTangent>(posIndexTangent(), unmodeledForceIndexTangent()) = J_pl_ext_force;
    A.block<sizeLinVelTangent, sizeAngVelTangent>(linVelIndexTangent(), unmodeledForceIndexTangent()) = J_vl_ext_force;
    A.block<sizeTorqueTangent, sizeTorqueTangent>(unmodeledTorqueIndexTangent(), unmodeledTorqueIndexTangent()) =
        J_ext_torque_ext_torque;
    A.block<sizePosTangent, sizeForceTangent>(posIndexTangent(), unmodeledTorqueIndexTangent()) = J_pl_ext_torque;
    A.block<sizeOriTangent, sizeTorqueTangent>(oriIndexTangent(), unmodeledTorqueIndexTangent()) = J_R_ext_torque;
    A.block<sizeAngVelTangent, sizeTorqueTangent>(angVelIndexTangent(), unmodeledTorqueIndexTangent()) =
        J_omega_ext_torque;
  }

  //// Jacobian matrices with respect to contacts ////

  // jacobian matrix of the contact position's state-transition wrt itself
  Matrix3 J_poscontact_poscontact =
      Matrix::Identity(sizePosTangent, sizePosTangent); // out of the loop as it is constant
                                                        // but then creates a useless variable if there is no contact
  for(VectorContactConstIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      const Contact & contact = *i;

      // predicted rest position of the contact
      Vector3 predictedWorldContactRestPosition = statePrediction.segment<sizePos>(contactPosIndex(i));
      // predicted rest orientation of the contact
      Orientation predictedWorldContactRestOri;
      predictedWorldContactRestOri.fromVector4(statePrediction.segment<sizeOri>(contactOriIndex(i))).toMatrix3();

      // input kinematics of the contact in the centroid frame
      const Kinematics & centroidContactKine = contact.centroidContactKine;

      // Jacobian matrix of the local linear acceleration with respect to the contact force
      Matrix3 J_linAcc_Fcis = (1 / mass_) * centroidContactKine.orientation.toMatrix3();
      // jacobian matrix of the local angular acceleration with respect to the contact force
      Matrix3 J_omegadot_Fcis =
          (I_inv * kine::skewSymmetric(centroidContactKine.position())) * centroidContactKine.orientation.toMatrix3();
      // jacobian matrix of the local angular acceleration with respect to the contact torque
      Matrix3 J_omegadot_Tcis = I_inv * centroidContactKine.orientation.toMatrix3();

      //// Jacobian matrices of the contact kinematics state transition ////

      // jacobian matrix of the local position's state-transition wrt the contact force
      Matrix3 J_pl_contactForce = J_pl_omega_dot * J_omegadot_Fcis + J_pl_al * J_linAcc_Fcis;
      // jacobian matrix of the local position's state-transition wrt the contact torque
      Matrix3 J_pl_contactTorque = J_pl_omega_dot * J_omegadot_Tcis;
      // jacobian matrix of the orientatiob's state-transition wrt the contact force
      Matrix3 J_R_contactForce = J_R_omegadot * J_omegadot_Fcis;
      // jacobian matrix of the orientation's state-transition wrt the contact torque
      Matrix3 J_R_contactTorque = J_R_omegadot * J_omegadot_Tcis;
      // jacobian matrix of the local linear velocity's state-transition wrt the contact force
      Matrix3 J_vl_contactForce = dt_ * J_linAcc_Fcis;
      // jacobian matrix of the local angular velocity's state-transition wrt the contact force
      Matrix3 J_omega_contactForce = dt_ * J_omegadot_Fcis;
      // jacobian matrix of the local angular velocity's state-transition wrt the contact torque
      Matrix3 J_omega_contactTorque = dt_ * J_omegadot_Tcis;
      // jacobian matrix of the contact orientation's state-transition wrt itself
      Matrix3 J_contactOri_contactOri = J_poscontact_poscontact;

      A.block<sizePosTangent, sizeForceTangent>(posIndexTangent(), contactForceIndexTangent(i)) = J_pl_contactForce;
      A.block<sizePosTangent, sizeTorqueTangent>(posIndexTangent(), contactTorqueIndexTangent(i)) = J_pl_contactTorque;
      A.block<sizeOriTangent, sizeForceTangent>(oriIndexTangent(), contactForceIndexTangent(i)) = J_R_contactForce;
      A.block<sizeOriTangent, sizeTorqueTangent>(oriIndexTangent(), contactTorqueIndexTangent(i)) = J_R_contactTorque;
      A.block<sizeLinVelTangent, sizeForceTangent>(linVelIndexTangent(), contactForceIndexTangent(i)) =
          J_vl_contactForce;
      A.block<sizeAngVelTangent, sizeForceTangent>(angVelIndexTangent(), contactForceIndexTangent(i)) =
          J_omega_contactForce;
      A.block<sizeAngVelTangent, sizeTorqueTangent>(angVelIndexTangent(), contactTorqueIndexTangent(i)) =
          J_omega_contactTorque;
      A.block<sizePosTangent, sizePosTangent>(contactPosIndexTangent(i), contactPosIndexTangent(i)) =
          J_poscontact_poscontact;
      A.block<sizeOriTangent, sizeOriTangent>(contactOriIndexTangent(i), contactOriIndexTangent(i)) =
          J_contactOri_contactOri;
      //// Jacobian matrices of the contact force ////

      // orientation of the world frame in the contact frame.

      Orientation contactWorldOri(
          Matrix3((predictedWorldCentroidStateOri.toMatrix3() * centroidContactKine.orientation.toMatrix3())
                      .transpose())); // better to compute it now as it is used in several expressions
      // jacobian matrix of the contact force wrt the local position
      Matrix3 J_contactForce_pl_at_same_time =
          -(contactWorldOri.toMatrix3() * contact.linearStiffness * predictedWorldCentroidStateOri.toMatrix3());
      Vector3 sumVelContact = centroidContactKine.linVel()
                              + predictedWorldCentroidStateAngVel.cross(centroidContactKine.position())
                              + predictedWorldCentroidStateLinVel;

      // jacobian matrix of the contact force wrt the orientation
      Matrix3 J_contactForce_R_at_same_time =
          contactWorldOri.toMatrix3()
          * (contact.linearStiffness
                 * kine::skewSymmetric(predictedWorldCentroidStateOri.toMatrix3()
                                       * (centroidContactKine.position() + predictedWorldCentroidStatePos))
             + contact.linearDamping * kine::skewSymmetric(predictedWorldCentroidStateOri.toMatrix3() * sumVelContact)
             - kine::skewSymmetric(contact.linearStiffness
                                       * (predictedWorldCentroidStateOri.toMatrix3()
                                          * (centroidContactKine.position() + predictedWorldCentroidStatePos))
                                   + contact.linearDamping
                                         * (predictedWorldCentroidStateOri.toMatrix3() * sumVelContact)
                                   - contact.linearStiffness * predictedWorldContactRestPosition));

      // jacobian matrix of the contact force wrt the linear velocity
      Matrix3 J_contactForce_vl_at_same_time = Matrix3::Zero();
      Matrix3 J_contactForce_omega_at_same_time = Matrix3::Zero();
      if(withDampingInMatrixA_)
      {
        J_contactForce_vl_at_same_time =
            -(contactWorldOri.toMatrix3() * contact.linearDamping * predictedWorldCentroidStateOri.toMatrix3());
        // jacobian matrix of the contact force wrt the angular velocity
        J_contactForce_omega_at_same_time =
            contactWorldOri.toMatrix3() * contact.linearDamping
            * (predictedWorldCentroidStateOri.toMatrix3() * kine::skewSymmetric(centroidContactKine.position()));
      }

      // jacobian matrix of the contact force wrt the contact position
      Matrix3 J_contactForce_contactPosition_at_same_time = contactWorldOri.toMatrix3() * contact.linearStiffness;

      A.block<sizeForceTangent, sizePosTangent>(contactForceIndexTangent(i), posIndexTangent()) =
          J_contactForce_pl_at_same_time * J_pl_pl;
      A.block<sizeForceTangent, sizeOriTangent>(contactForceIndexTangent(i), oriIndexTangent()) =
          J_contactForce_pl_at_same_time * J_pl_R + J_contactForce_R_at_same_time
          + J_contactForce_vl_at_same_time * J_vl_R;
      A.block<sizeForceTangent, sizeLinVelTangent>(contactForceIndexTangent(i), linVelIndexTangent()) =
          J_contactForce_pl_at_same_time * J_pl_vl + J_contactForce_vl_at_same_time * J_vl_vl;
      A.block<sizeForceTangent, sizeAngVelTangent>(contactForceIndexTangent(i), angVelIndexTangent()) =
          J_contactForce_pl_at_same_time * J_pl_omega + J_contactForce_R_at_same_time * J_R_omega
          + J_contactForce_vl_at_same_time * J_vl_omega + J_contactForce_omega_at_same_time * J_omega_omega;

      A.block<sizeForceTangent, sizePosTangent>(contactForceIndexTangent(i), contactPosIndexTangent(i)) =
          J_contactForce_contactPosition_at_same_time;
      A.block<sizeForceTangent, sizeForceTangent>(contactForceIndexTangent(i), contactForceIndexTangent(i)) =
          J_contactForce_pl_at_same_time * J_pl_contactForce + J_contactForce_R_at_same_time * J_R_contactForce
          + J_contactForce_omega_at_same_time * J_omega_contactForce
          + J_contactForce_vl_at_same_time * J_vl_contactForce;

      A.block<sizeForceTangent, sizeTorqueTangent>(contactForceIndexTangent(i), contactTorqueIndexTangent(i)) =
          J_contactForce_pl_at_same_time * J_pl_contactTorque + J_contactForce_R_at_same_time * J_R_contactTorque
          + J_contactForce_omega_at_same_time * J_omega_contactTorque;

      //// Jacobian matrices of the contact torque ////
      Vector3 angVelSum =
          predictedWorldCentroidStateOri * (centroidContactKine.angVel() + predictedWorldCentroidStateAngVel);
      Vector3 ex = Vector3::UnitX();
      Vector3 ey = Vector3::UnitY();
      Vector3 ez = Vector3::UnitZ();

      // difference between the rest orientation of the contact and the one obtained by forward kinematics from the
      // centroid, expressed in the world frame.
      Orientation world_restOriVsFk =
          predictedWorldCentroidStateOri * centroidContactKine.orientation * predictedWorldContactRestOri.inverse();
      Orientation invWorld_restOriVsFk = world_restOriVsFk.inverse();

      // intermediary terms for the jacobian computation
      Matrix3 Vk = -ex * ez.transpose()
                       * (kine::skewSymmetric(world_restOriVsFk.toMatrix3() * ey)
                          + invWorld_restOriVsFk.toMatrix3() * kine::skewSymmetric(ey))
                   - ey * ex.transpose()
                         * (kine::skewSymmetric(world_restOriVsFk.toMatrix3() * ez)
                            + invWorld_restOriVsFk.toMatrix3() * kine::skewSymmetric(ez))
                   - ez * ey.transpose()
                         * (kine::skewSymmetric(world_restOriVsFk.toMatrix3() * ex)
                            + invWorld_restOriVsFk.toMatrix3() * kine::skewSymmetric(ex));

      Matrix3 Vk2 = -ex * ez.transpose()
                        * (kine::skewSymmetric(invWorld_restOriVsFk.toMatrix3() * ey)
                           + world_restOriVsFk.toMatrix3() * kine::skewSymmetric(ey))
                    - ey * ex.transpose()
                          * (kine::skewSymmetric(invWorld_restOriVsFk.toMatrix3() * ez)
                             + world_restOriVsFk.toMatrix3() * kine::skewSymmetric(ez))
                    - ez * ey.transpose()
                          * (kine::skewSymmetric(invWorld_restOriVsFk.toMatrix3() * ex)
                             + world_restOriVsFk.toMatrix3() * kine::skewSymmetric(ex));

      // jacobian matrix of the contact torque wrt the orientation
      Matrix3 J_contactTorque_R_at_same_time =
          -(contactWorldOri.toMatrix3()
            * (kine::skewSymmetric(0.5 * contact.angularStiffness
                                       * (kine::skewSymmetricToRotationVector(world_restOriVsFk.toMatrix3()
                                                                              - invWorld_restOriVsFk.toMatrix3()))
                                   + contact.angularDamping * angVelSum)
               + 0.5 * contact.angularStiffness * Vk - contact.angularDamping * kine::skewSymmetric(angVelSum)));

      // jacobian matrix of the contact torque wrt the local angular velocity
      Matrix3 J_contactTorque_omega_at_same_time =
          -(contactWorldOri.toMatrix3() * contact.angularDamping * predictedWorldCentroidStateOri.toMatrix3());
      // jacobian matrix of the contact torque wrt the contact orientation
      Matrix3 J_contactTorque_contactOri_at_same_time =
          0.5 * (contactWorldOri.toMatrix3() * contact.angularStiffness * Vk2);

      A.block<sizeTorqueTangent, sizeOriTangent>(contactTorqueIndexTangent(i), oriIndexTangent()) =
          J_contactTorque_R_at_same_time;
      A.block<sizeTorqueTangent, sizeAngVelTangent>(contactTorqueIndexTangent(i), angVelIndexTangent()) =
          J_contactTorque_R_at_same_time * J_R_omega + J_contactTorque_omega_at_same_time * J_omega_omega;
      A.block<sizeTorqueTangent, sizeForceTangent>(contactTorqueIndexTangent(i), contactOriIndexTangent(i)) =
          J_contactTorque_contactOri_at_same_time;
      A.block<sizeTorqueTangent, sizeForceTangent>(contactTorqueIndexTangent(i), contactForceIndexTangent(i)) =
          J_contactTorque_R_at_same_time * J_R_contactForce + J_contactTorque_omega_at_same_time * J_omega_contactForce;
      A.block<sizeTorqueTangent, sizeTorqueTangent>(contactTorqueIndexTangent(i), contactTorqueIndexTangent(i)) =
          J_contactTorque_R_at_same_time * J_R_contactTorque
          + J_contactTorque_omega_at_same_time * J_omega_contactTorque;

      if(withUnmodeledWrench_)
      {
        A.block<sizeForceTangent, sizeForceTangent>(contactForceIndexTangent(i), unmodeledForceIndexTangent()) =
            J_contactForce_pl_at_same_time * J_pl_ext_force + J_contactForce_vl_at_same_time * J_vl_ext_force;
        A.block<sizeForceTangent, sizeTorqueTangent>(contactForceIndexTangent(i), unmodeledTorqueIndexTangent()) =
            J_contactForce_pl_at_same_time * J_pl_ext_torque + J_contactForce_R_at_same_time * J_R_ext_torque
            + J_contactForce_omega_at_same_time * J_omega_ext_torque;
        A.block<sizeTorqueTangent, sizeTorqueTangent>(contactTorqueIndexTangent(i), unmodeledTorqueIndexTangent()) =
            J_contactTorque_R_at_same_time * J_R_ext_torque + J_contactTorque_omega_at_same_time * J_omega_ext_torque;
      }
    }
  }
  return A;
}

Matrix KineticsObserver::computeCMatrix()
{
  const Vector & statePrediction = ekf_.updateStatePrediction();
  Orientation predictedWorldCentroidStateOri;
  predictedWorldCentroidStateOri.fromVector4(statePrediction.segment<sizeOri>(oriIndex())).toMatrix3();
  const Vector3 & predictedWorldCentroidStateAngVel = statePrediction.segment<sizeAngVel>(angVelIndex());

  Vector3 forceCentroid = additionalForce_;
  Vector3 torqueCentroid = additionalTorque_;

  addUnmodeledAndContactWrench_(statePrediction, forceCentroid, torqueCentroid);

  LocalKinematics predictedWorldCentroidStateKinematics(statePrediction.segment<sizeStateKine>(kineIndex()),
                                                        flagsStateKine);

  /// The accelerations are about to be computed so we set them to "initialized"
  Vector3 & linacc = predictedWorldCentroidStateKinematics.linAcc.set();
  Vector3 & angacc = predictedWorldCentroidStateKinematics.angAcc.set();

  computeLocalAccelerations_(predictedWorldCentroidStateKinematics, forceCentroid, torqueCentroid, linacc, angacc);

  Matrix C = Matrix::Zero(measurementTangentSize_, stateTangentSize_);

  Matrix3 Iinv = I_().inverse();

  // Jacobians of the gyrometer bias
  for(VectorIMUConstIterator i = imuSensors_.begin(); i != imuSensors_.end(); ++i)
  {
    if(i->time == k_data_)
    {
      const IMU & imu = *i;

      Matrix3 oriCentroidToImu = imu.centroidImuKinematics.orientation.toMatrix3().transpose();

      C.block<sizeAcceleroSignal, sizeAngVelTangent>(imu.measIndex, angVelIndexTangent()) =
          -kine::skewSymmetric(
              (oriCentroidToImu * predictedWorldCentroidStateAngVel).cross(imu.centroidImuKinematics.position()))
              * oriCentroidToImu
          - kine::skewSymmetric(oriCentroidToImu * predictedWorldCentroidStateAngVel)
                * kine::skewSymmetric(imu.centroidImuKinematics.position()) * oriCentroidToImu
          - kine::skewSymmetric(imu.centroidImuKinematics.position()) * oriCentroidToImu
                * (Iinv
                   * (kine::skewSymmetric(I_() * predictedWorldCentroidStateAngVel) - Id_()
                      - kine::skewSymmetric(predictedWorldCentroidStateAngVel) * I_() + kine::skewSymmetric(sigma_())))
          - 2 * kine::skewSymmetric(imu.centroidImuKinematics.linVel()) * oriCentroidToImu;

      C.block<sizeAcceleroSignal, sizeForce>(imu.measIndex, unmodeledForceIndexTangent()) =
          1.0 / mass_ * oriCentroidToImu;

      C.block<sizeAcceleroSignal, sizeTorque>(imu.measIndex, unmodeledTorqueIndexTangent()) =
          -kine::skewSymmetric(imu.centroidImuKinematics.position()) * oriCentroidToImu * Iinv;

      for(VectorContactConstIterator i = contacts_.begin(); i != contacts_.end(); ++i)
      {
        const Contact & contact = *i;
        if(contact.isSet)
        {
          C.block<sizeAcceleroSignal, sizeForceTangent>(imu.measIndex, contactForceIndexTangent(i)) =
              oriCentroidToImu * 1.0 / mass_ * contact.centroidContactKine.orientation.toMatrix3()
              - kine::skewSymmetric(imu.centroidImuKinematics.position()) * oriCentroidToImu * Iinv
                    * kine::skewSymmetric(contact.centroidContactKine.position())
                    * contact.centroidContactKine.orientation.toMatrix3();

          C.block<sizeAcceleroSignal, sizeTorqueTangent>(imu.measIndex, contactTorqueIndexTangent(i)) =
              -kine::skewSymmetric(imu.centroidImuKinematics.position()) * oriCentroidToImu * Iinv
              * contact.centroidContactKine.orientation.toMatrix3();
        }
      }

      /// gyrometer
      C.block<sizeGyroSignal, sizeAngVelTangent>(imu.measIndex + sizeAcceleroSignal, angVelIndexTangent()) =
          oriCentroidToImu;

      if(withGyroBias_)
      {
        C.block<sizeGyroSignal, sizeAngVelTangent>(imu.measIndex + sizeAcceleroSignal, gyroBiasIndexTangent(i)) =
            Matrix3::Identity();
      }
    }
  }

  for(VectorContactConstIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    const Contact & contact = *i;

    if(contact.withRealSensor)
    {
      C.block<sizeForceTangent, sizeForceTangent>(contact.measIndex, contactForceIndexTangent(i)) = Matrix3::Identity();
      C.block<sizeTorqueTangent, sizeTorqueTangent>(contact.measIndex + sizeForceTangent,
                                                    contactTorqueIndexTangent(i)) = Matrix3::Identity();
    }
  }

  if(absPoseSensor_.time == k_data_)
  {
    C.block<sizePosTangent, sizePosTangent>(absPoseSensor_.measIndex, posIndexTangent()) =
        predictedWorldCentroidStateOri.toMatrix3();
    C.block<sizeOriTangent, sizeOriTangent>(absPoseSensor_.measIndex + sizePosTangent, oriIndexTangent()) =
        Matrix3::Identity();
  }

  if(absOriSensor_.time == k_data_)
  {
    C.block<sizeOriTangent, sizeOriTangent>(absOriSensor_.measIndex, oriIndexTangent()) = Matrix3::Identity();
  }

  return C;
}

void KineticsObserver::convertUserToCentroidFrame_(const Kinematics & userKine,
                                                   Kinematics & centroidKine,
                                                   [[maybe_unused]] TimeIndex k_data)
{
  /*
  Our centroid frame has the same orientation than the user frame, so the conversion from the user to the centroid
  frame simply depends on the linear kinematics of the center of mass in the user frame.
  */
  BOOST_ASSERT((com_.getTime() == k_data && com_.getTime() == comd_.getTime() && com_.getTime() == comdd_.getTime())
               && "The Center of Mass must be actualized before the conversion");
  centroidKine.position = userKine.position() - com_();
  if(userKine.linVel.isSet())
  {
    centroidKine.linVel = userKine.linVel() - comd_();
  }
  if(userKine.linAcc.isSet())
  {
    centroidKine.linAcc = userKine.linAcc() - comdd_();
  }
  if(userKine.orientation.isSet())
  {
    centroidKine.orientation = userKine.orientation;
  }
  if(userKine.angVel.isSet())
  {
    centroidKine.angVel = userKine.angVel();
  }
  if(userKine.angAcc.isSet())
  {
    centroidKine.angAcc = userKine.angAcc();
  }
}

KineticsObserver::Kinematics KineticsObserver::convertUserToCentroidFrame_(const Kinematics & userKine,
                                                                           [[maybe_unused]] TimeIndex k_data)
{
  /*
  Our centroid frame has the same orientation than the user frame, so the conversion from the user to the centroid
  frame simply depends on the linear kinematics of the center of mass in the user frame.
  */

  Kinematics centroidKine;
  BOOST_ASSERT((com_.getTime() == k_data && com_.getTime() == comd_.getTime() && com_.getTime() == comdd_.getTime())
               && "The Center of Mass must be actualized before the conversion");
  centroidKine.position = userKine.position() - com_();
  if(userKine.linVel.isSet())
  {
    centroidKine.linVel = userKine.linVel() - comd_();
  }
  if(userKine.linAcc.isSet())
  {
    centroidKine.linAcc = userKine.linAcc() - comdd_();
  }
  if(userKine.orientation.isSet())
  {
    centroidKine.orientation = userKine.orientation;
  }
  if(userKine.angVel.isSet())
  {
    centroidKine.angVel = userKine.angVel();
  }
  if(userKine.angAcc.isSet())
  {
    centroidKine.angAcc = userKine.angAcc();
  }
  return centroidKine;
}

void KineticsObserver::updateLocalKineAndContacts_()
{
  worldCentroidStateKinematics_.fromVector(worldCentroidStateVector_.segment<sizeStateKine>(kineIndex()),
                                           flagsStateKine);
  for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      i->worldRestPose.fromVector(worldCentroidStateVector_.segment<sizePose>(contactPosIndex(i)), flagsContactKine);
    }
  }
}

void KineticsObserver::updateGlobalKine_()
{
  worldCentroidKinematics_ = worldCentroidStateKinematics_;
}

void KineticsObserver::addUnmodeledAndContactWrench_(const Vector & worldCentroidStateVector,
                                                     Vector3 & force,
                                                     Vector3 & torque)
{
  force += worldCentroidStateVector.segment<sizeForce>(unmodeledWrenchIndex());

  torque += worldCentroidStateVector.segment<sizeTorque>(unmodeledTorqueIndex());

  for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    const Contact & contact = *i;
    if(contact.isSet)
    {
      // input kinematics of the contact in the centroid frame.
      const Kinematics & centroidContactKinei = contact.centroidContactKine;
      // contact force expressed at the centroid
      Vector3 centroidContactForcei =
          centroidContactKinei.orientation * worldCentroidStateVector.segment<sizeForce>(contactForceIndex(i));

      force += centroidContactForcei;
      // contact torque expressed at the centroid
      torque += centroidContactKinei.orientation * worldCentroidStateVector.segment<sizeTorque>(contactTorqueIndex(i))
                + centroidContactKinei.position().cross(centroidContactForcei);
    }
  }
}

void KineticsObserver::addUnmodeledWrench_(const Vector & worldCentroidStateVector, Vector3 & force, Vector3 & torque)
{
  force += worldCentroidStateVector.segment<sizeForce>(unmodeledWrenchIndex());
  torque += worldCentroidStateVector.segment<sizeForce>(unmodeledTorqueIndex());
}

void KineticsObserver::addContactWrench_(const Kinematics & centroidContactKine,
                                         const Vector3 & centroidContactForce,
                                         const Vector3 & centroidContactTorque,
                                         Vector3 & totalCentroidForce,
                                         Vector3 & totalCentroidTorque)
{
  totalCentroidForce += centroidContactForce;
  totalCentroidTorque += centroidContactTorque + centroidContactKine.position().cross(centroidContactForce);
}

void KineticsObserver::computeLocalAccelerations_(LocalKinematics & worldCentroidStateKinematics,
                                                  const Vector3 & totalCentroidForce,
                                                  const Vector3 & totalCentroidTorque,
                                                  Vector3 & linAcc,
                                                  Vector3 & angAcc)
{
  angAcc = I_().inverse()
           * (totalCentroidTorque - Id_() * worldCentroidStateKinematics.angVel() - sigmad_()
              - worldCentroidStateKinematics.angVel().cross(I_() * worldCentroidStateKinematics.angVel() + sigma_()));

  linAcc =
      (totalCentroidForce / mass_) - worldCentroidStateKinematics.orientation.toMatrix3().transpose() * cst::gravity;
}

void KineticsObserver::computeLocalAccelerations(const Vector & x, Vector & acceleration)
{
  // initialization of the total force at the centroid with the input additional forces.
  Vector3 totalCentroidForce = additionalForce_;
  Vector3 totalCentroidTorque = additionalTorque_;

  // adding the previously estimated contact and unmodeled forces to obtain the total force at the centroid
  addUnmodeledAndContactWrench_(x, totalCentroidForce, totalCentroidTorque);
  LocalKinematics worldCentroidStateKinematics(x, flagsStateKine);

  acceleration.segment<3>(0) =
      (totalCentroidForce / mass_) - worldCentroidStateKinematics.orientation.toMatrix3().transpose() * cst::gravity;

  acceleration.segment<3>(3) =
      I_().inverse()
      * (totalCentroidTorque - Id_() * worldCentroidStateKinematics.angVel() - sigmad_()
         - worldCentroidStateKinematics.angVel().cross(I_() * worldCentroidStateKinematics.angVel() + sigma_()));
}

void KineticsObserver::computeContactWrench_(const Contact & contact,
                                             Kinematics & worldCentroidStateKinematics,
                                             Kinematics & worldRestContactPose,
                                             Vector6 & contactWrench)
{
  // the kinematics of the contact in the centroid's frame, expressed in the centroid's frame
  const Kinematics & centroidContactKine = contact.centroidContactKine;
  // the kinematics of the contact in the world frame, expressed in the world frame
  Kinematics worldFkContactPose;
  worldFkContactPose.setToProductNoAlias(worldCentroidStateKinematics, centroidContactKine);

  contactWrench.segment(0, 3) =
      -(worldFkContactPose.orientation.toMatrix3().transpose()
        * (contact.linearStiffness * (worldFkContactPose.position() - worldRestContactPose.position())
           + contact.linearDamping * worldFkContactPose.linVel()));

  Matrix R = worldFkContactPose.orientation.toMatrix3() * worldRestContactPose.orientation.toMatrix3().transpose();
  contactWrench.segment(3, 3) =
      -worldFkContactPose.orientation.toMatrix3().transpose()
      * (0.5 * contact.angularStiffness * kine::skewSymmetricToRotationVector(R - R.transpose())
         + contact.angularDamping * worldFkContactPose.angVel());
}

void KineticsObserver::computeContactForces_(LocalKinematics & worldCentroidStateKinematics,
                                             Vector3 & contactForce,
                                             Vector3 & contactTorque)
{
  BOOST_ASSERT(contactForce.isZero() && "The contact forces must be initialized with a zero vector");
  BOOST_ASSERT(contactTorque.isZero() && "The contact forces must be initialized with a zero vector");

  for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      Contact & contact = *i;

      // the kinematics of the contact in the centroid's frame, expressed in the centroid's frame
      Kinematics & centroidContactKine = contact.centroidContactKine;
      // the kinematics of the contact in the world frame, expressed in the world frame
      Kinematics worldFkContactPose;
      // the rest kinematics of the contact in the world frame, expressed in the world frame
      Kinematics & worldRestContactPose = contact.worldRestPose;

      worldFkContactPose.setToProductNoAlias(Kinematics(worldCentroidStateKinematics), centroidContactKine);

      Vector3 centroidContactForce =
          worldCentroidStateKinematics.orientation.toMatrix3().transpose()
          * (contact.linearStiffness * (worldRestContactPose.position() - worldFkContactPose.position())
             - contact.linearDamping * worldFkContactPose.linVel());
      contactForce += centroidContactForce;

      Vector3 centroidContactTorque = worldCentroidStateKinematics.orientation.toMatrix3().transpose()
                                      * (-0.5 * contact.angularStiffness
                                             * (worldFkContactPose.orientation.toQuaternion()
                                                * worldRestContactPose.orientation.toQuaternion().inverse())
                                                   .vec()
                                         - contact.angularDamping * worldFkContactPose.angVel());

      contactTorque += centroidContactTorque + centroidContactKine.position().cross(centroidContactForce);
    }
  }
}

void KineticsObserver::stateSum(const Vector & worldCentroidStateVector, const Vector & tangentVector, Vector & sum)
{
  Orientation & o = opt_.ori;
  sum = worldCentroidStateVector;
  /// use the exponential map integration to perform the sum of the states
  sum.segment<sizePos>(posIndex()) += tangentVector.segment<sizePos>(posIndexTangent());
  o.fromVector4(worldCentroidStateVector.segment<sizeOri>(oriIndex()));
  o.integrate(tangentVector.segment<sizeOriTangent>(
      oriIndexTangent())); // we don't use integrateRightOrientation()
                           // as it is used only for the computation of the dynamical evolution of the orientation by
                           // integration
  sum.segment<sizeOri>(oriIndex()) = o.toVector4();
  sum.segment<sizeLinVel + sizeAngVel>(linVelIndex()) +=
      tangentVector.segment<sizeLinVel + sizeAngVel>(linVelIndexTangent());
  if(withGyroBias_)
  {
    for(unsigned i = 0; i < imuSensors_.size(); ++i)
    {
      sum.segment<sizeGyroBias>(gyroBiasIndex(i)) += tangentVector.segment<sizeGyroBias>(gyroBiasIndexTangent(i));
    }
  }
  if(withUnmodeledWrench_)
  {
    sum.segment<sizeWrench>(unmodeledWrenchIndex()) += tangentVector.segment<sizeWrench>(unmodeledWrenchIndexTangent());
  }
  for(VectorContactConstIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      sum.segment<sizePos>(contactPosIndex(i)) += tangentVector.segment<sizePos>(contactPosIndexTangent(i));
      o.fromVector4(worldCentroidStateVector.segment<sizeOri>(contactOriIndex(i)));
      o.integrate(tangentVector.segment<sizeOriTangent>(contactOriIndexTangent(i)));
      sum.segment<sizeOri>(contactOriIndex(i)) = o.toVector4();
      sum.segment<sizeWrench>(contactWrenchIndex(i)) += tangentVector.segment<sizeWrench>(contactWrenchIndexTangent(i));
    }
  }
}

void KineticsObserver::stateDifference(const Vector & worldCentroidStateVector1,
                                       const Vector & worldCentroidStateVector2,
                                       Vector & difference)
{
  Orientation & o1 = opt_.ori1;
  Orientation & o2 = opt_.ori2;
  difference.resize(stateTangentSize_);
  difference.setZero();
  difference.segment<sizePos>(posIndexTangent()).noalias() =
      worldCentroidStateVector1.segment<sizePos>(posIndex()) - worldCentroidStateVector2.segment<sizePos>(posIndex());
  o1.fromVector4(worldCentroidStateVector1.segment<sizeOri>(oriIndex()));
  o2.fromVector4(worldCentroidStateVector2.segment<sizeOri>(oriIndex()));
  difference.segment<sizeOriTangent>(oriIndexTangent()) = o2.differentiate(o1);
  difference.segment<sizeLinVel + sizeAngVel>(linVelIndexTangent()).noalias() =
      worldCentroidStateVector1.segment<sizeLinVel + sizeAngVel>(linVelIndex())
      - worldCentroidStateVector2.segment<sizeLinVel + sizeAngVel>(linVelIndex());
  if(withGyroBias_)
  {
    for(unsigned i = 0; i < imuSensors_.size(); ++i)
    {
      difference.segment<sizeGyroBias>(gyroBiasIndexTangent(i)).noalias() =
          worldCentroidStateVector1.segment<sizeGyroBias>(gyroBiasIndex(i))
          - worldCentroidStateVector2.segment<sizeGyroBias>(gyroBiasIndex(i));
    }
  }
  if(withUnmodeledWrench_)
  {
    difference.segment<sizeWrench>(unmodeledForceIndexTangent()).noalias() =
        worldCentroidStateVector1.segment<sizeWrench>(unmodeledWrenchIndex())
        - worldCentroidStateVector2.segment<sizeWrench>(unmodeledWrenchIndex());
  }

  for(VectorContactConstIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      difference.segment<sizePos>(contactPosIndexTangent(i)).noalias() =
          worldCentroidStateVector1.segment<sizePos>(contactPosIndex(i))
          - worldCentroidStateVector2.segment<sizePos>(contactPosIndex(i));
      o1.fromVector4(worldCentroidStateVector1.segment<sizeOri>(contactOriIndex(i)));
      o2.fromVector4(worldCentroidStateVector2.segment<sizeOri>(contactOriIndex(i)));
      difference.segment<sizeOriTangent>(contactOriIndexTangent(i)) = o2.differentiate(o1);
      difference.segment<sizeWrench>(contactWrenchIndexTangent(i)).noalias() =
          worldCentroidStateVector1.segment<sizeWrench>(contactWrenchIndex(i))
          - worldCentroidStateVector2.segment<sizeWrench>(contactWrenchIndex(i));
    }
  }
}

void KineticsObserver::measurementDifference(const Vector & measureVector1,
                                             const Vector & measureVector2,
                                             Vector & difference)
{
  Orientation & o1 = opt_.ori1;
  Orientation & o2 = opt_.ori2;
  difference.resize(measurementTangentSize_);

  Index currentMeasurementSize = sizeIMUSignal * currentIMUSensorNumber_ + sizeWrench * numberOfContactRealSensors_;

  difference.segment(0, currentMeasurementSize).noalias() =
      measureVector1.segment(0, currentMeasurementSize) - measureVector2.segment(0, currentMeasurementSize);

  if(absPoseSensor_.time == k_data_)
  {

    difference.segment<sizePos>(currentMeasurementSize).noalias() =
        measureVector1.segment<sizePos>(currentMeasurementSize)
        - measureVector2.segment<sizePos>(currentMeasurementSize);

    currentMeasurementSize += sizePos;

    o1.fromVector4(measureVector1.segment<sizeOri>(currentMeasurementSize));
    o2.fromVector4(measureVector2.segment<sizeOri>(currentMeasurementSize));
    difference.segment<sizeOriTangent>(currentMeasurementSize) = o2.differentiate(o1);

    currentMeasurementSize += sizeOri;
  }
  if(absOriSensor_.time == k_data_)
  {
    o1.fromVector4(measureVector1.segment<sizeOri>(currentMeasurementSize));
    o2.fromVector4(measureVector2.segment<sizeOri>(currentMeasurementSize));
    difference.segment<sizeOriTangent>(currentMeasurementSize) = o2.differentiate(o1);
  }
}

Vector KineticsObserver::stateDynamics(const Vector & xInput, const std::any & /*unused*/, TimeIndex)
{
  Vector x = xInput;
  // initialization of the total force at the centroid with the input additional forces.
  initTotalCentroidForce_ = additionalForce_;
  initTotalCentroidTorque_ = additionalTorque_;

  // adding the previously estimated contact and unmodeled forces to obtain the total force at the centroid
  addUnmodeledAndContactWrench_(x, initTotalCentroidForce_, initTotalCentroidTorque_);

  LocalKinematics worldCentroidStateKinematics(x.segment<sizeStateKine>(kineIndex()), flagsStateKine);

  /// The accelerations are about to be computed so we initialize them
  worldCentroidStateKinematics.linAcc = Vector3::Zero();
  worldCentroidStateKinematics.angAcc = Vector3::Zero();

  Vector3 & linacc = worldCentroidStateKinematics.linAcc(); /// reference (Vector3&)
  Vector3 & angacc = worldCentroidStateKinematics.angAcc(); /// reference

  computeLocalAccelerations_(worldCentroidStateKinematics, initTotalCentroidForce_, initTotalCentroidTorque_, linacc,
                             angacc);

  worldCentroidStateKinematics.SE3_integration(dt_);

  x.segment<sizeStateKine>(kineIndex()) = worldCentroidStateKinematics.toVector(flagsStateKine);

  Kinematics globWorldCentroidStateKinematics = Kinematics(worldCentroidStateKinematics);

  if(!withGyroBias_)
  {
    for(VectorIMUIterator i = imuSensors_.begin(), ie = imuSensors_.end(); i != ie; ++i)
    {
      x.segment<sizeGyroBias>(gyroBiasIndex(i)).setZero();
    }
  }
  if(!withUnmodeledWrench_)
  {
    x.segment<sizeWrench>(unmodeledForceIndex()).setZero();
  }

  for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      Contact & contact = *i;

      // rest kinematics of the contact in the world frame
      Kinematics worldContactRestPose; // not using the variable belonging to Contact as this variable must change only
                                       // at the end of the update
      worldContactRestPose.fromVector(x.segment<sizePose>(contactPosIndex(i)), flagsPoseKine);

      Vector6 predictedWrench;
      computeContactWrench_(contact, globWorldCentroidStateKinematics, worldContactRestPose, predictedWrench);
      x.segment<sizeForce>(contactForceIndex(i)) = predictedWrench.segment(0, 3);
      x.segment<sizeTorque>(contactTorqueIndex(i)) = predictedWrench.segment(3, 3);
    }
  }

  if(processNoise_ != 0x0)
  {
    processNoise_->getNoisy(x);
  }

  return x;
}

Vector6 KineticsObserver::getCurrentViscoElasticWrench(Index numContact)
{
  BOOST_ASSERT(!contacts_[numContact].isSet
               && "The contact doesn't exist, the associated visco-elastic wrench cannot be computed.");

  const Contact & contact = contacts_.at(static_cast<size_t>(numContact));

  LocalKinematics worldCentroidStateKinematics;
  worldCentroidStateKinematics.fromVector(getCurrentStateVector().segment<sizeStateKine>(kineIndex()),
                                          kine::Kinematics::Flags::pose | kine::Kinematics::Flags::vel);
  Kinematics globWorldCentroidStateKinematics(worldCentroidStateKinematics);
  // rest kinematics of the contact in the world frame
  Kinematics worldContactRestPose; // not using the variable belonging to Contact as this variable must change only
                                   // at the end of the update
  worldContactRestPose.fromVector(getCurrentStateVector().segment<sizePose>(contactPosIndex(numContact)),
                                  flagsPoseKine);

  Vector6 contactWrench;
  computeContactWrench_(contact, globWorldCentroidStateKinematics, worldContactRestPose, contactWrench);
  return contactWrench;
}

Vector KineticsObserver::measureDynamics(const Vector & x_bar, const std::any & /*unused*/, TimeIndex k)
{
  Vector y(getMeasurementSize());

  Vector3 forceCentroid = additionalForce_;
  Vector3 torqueCentroid = additionalTorque_;

  addUnmodeledAndContactWrench_(x_bar, forceCentroid, torqueCentroid);

  LocalKinematics worldCentroidStateKinematics(x_bar.segment<sizeStateKine>(kineIndex()), flagsStateKine);

  /// The accelerations are about to be computed so we initialize them

  worldCentroidStateKinematics.linAcc = Vector3::Zero();
  worldCentroidStateKinematics.angAcc = Vector3::Zero();

  Vector3 & linacc = worldCentroidStateKinematics.linAcc();
  Vector3 & angacc = worldCentroidStateKinematics.angAcc();

  computeLocalAccelerations_(worldCentroidStateKinematics, forceCentroid, torqueCentroid, linacc, angacc);

  for(VectorIMUConstIterator i = imuSensors_.begin(); i != imuSensors_.end(); ++i)
  {
    if(i->time == k_data_)
    {
      const IMU & imu = *i;
      // the kinematics of the IMU in the world frame, expressed in the IMU's frame
      LocalKinematics worldImuKinematics;
      worldImuKinematics.setToProductNoAlias(worldCentroidStateKinematics, imu.centroidImuKinematics);

      const Matrix3 & worldImuOri = worldImuKinematics.orientation.toMatrix3();

      /// accelerometer
      y.segment<sizeAcceleroSignal>(imu.measIndex).noalias() =
          worldImuKinematics.linAcc() + worldImuOri.transpose() * cst::gravity;

      /// gyrometer
      if(withGyroBias_)
      {
        y.segment<sizeGyroSignal>(imu.measIndex + sizeAcceleroSignal).noalias() =
            worldImuKinematics.angVel() + x_bar.segment<sizeGyroBias>(gyroBiasIndex(i));
      }
      else
      {
        y.segment<sizeGyroSignal>(imu.measIndex + sizeAcceleroSignal).noalias() = worldImuKinematics.angVel();
      }
    }
  }

  for(VectorContactConstIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet && i->time == k_data_ && i->withRealSensor)
    {
      y.segment<sizeWrench>(i->measIndex) = x_bar.segment<sizeWrench>(contactWrenchIndex(i));
    }
  }

  if(absPoseSensor_.time == k)
  {
    y.segment<sizePos>(absPoseSensor_.measIndex) =
        worldCentroidStateKinematics.orientation.toMatrix3() * worldCentroidStateKinematics.toVector(flagsPosKine);
    y.segment<sizeOri>(absPoseSensor_.measIndex + sizePos) = worldCentroidStateKinematics.orientation.toVector4();
  }

  if(absOriSensor_.time == k)
  {
    y.segment<sizeOri>(absOriSensor_.measIndex) = worldCentroidStateKinematics.orientation.toVector4();
  }

  if(measurementNoise_ != 0x0)
  {
    measurementNoise_->getNoisy(y);
  }
  return y;
}

void KineticsObserver::setInitWorldCentroidStateVector(const Vector & initStateVector)
{
  worldCentroidStateVector_.setZero();
  setStateVector(initStateVector, false);
}

Vector6 KineticsObserver::getCentroidContactWrench(Index numContact) const
{
  Vector6 centroidContactWrench;

  // input kinematics of the contact in the centroid frame
  const Kinematics & centroidContactKine = contacts_.at(static_cast<size_t>(numContact)).centroidContactKine;

  centroidContactWrench.segment<sizeForce>(0) =
      centroidContactKine.orientation.toMatrix3()
      * worldCentroidStateVector_.segment<sizeForce>(contactForceIndex(numContact));

  centroidContactWrench.segment<sizeTorque>(sizeForce) =
      centroidContactKine.orientation.toMatrix3()
          * worldCentroidStateVector_.segment<sizeTorque>(contactTorqueIndex(numContact))
      + centroidContactKine.position().cross(centroidContactWrench.segment<sizeForce>(0));

  return centroidContactWrench;
}

kine::Kinematics KineticsObserver::getCentroidContactInputKine(Index numContact) const
{
  return contacts_.at(static_cast<size_t>(numContact)).centroidContactKine;
}

kine::Kinematics KineticsObserver::getWorldContactKineFromCentroid(Index numContact) const
{
  BOOST_ASSERT(contacts_.at(static_cast<size_t>(numContact)).isSet && "This contact is not set.");
  Kinematics worldFkContactKine;
  worldFkContactKine.setToProductNoAlias(Kinematics(worldCentroidStateKinematics_),
                                         contacts_.at(static_cast<size_t>(numContact)).centroidContactKine);
  return worldFkContactKine;
}

kine::Kinematics KineticsObserver::getContactStateRestKinematics(Index numContact) const
{
  return contacts_.at(static_cast<size_t>(numContact)).worldRestPose;
}

kine::Kinematics KineticsObserver::getUserContactInputKine(Index numContact) const
{
  return contacts_.at(static_cast<size_t>(numContact)).userContactKine;
}

Index KineticsObserver::getIMUMeasIndexByNum(Index num) const
{
  return imuSensors_[static_cast<size_t>(num)].measIndex;
}

Index KineticsObserver::getContactMeasIndexByNum(Index num) const
{
  return contacts_[static_cast<size_t>(num)].measIndex;
}

bool KineticsObserver::getContactIsSetByNum(Index num) const
{
  if(static_cast<size_t>(num) >= contacts_.size() || contacts_.empty())
  {
    return false;
  }
  return contacts_[static_cast<size_t>(num)].isSet;
}

double KineticsObserver::getMass() const
{
  return mass_;
}

const IndexedMatrix3 & KineticsObserver::getInertiaMatrix() const
{
  return I_;
}

const IndexedMatrix3 & KineticsObserver::getInertiaMatrixDot() const
{
  return Id_;
}

const IndexedVector3 & KineticsObserver::getAngularMomentum() const
{
  return sigma_;
}

const IndexedVector3 & KineticsObserver::getAngularMomentumDot() const
{
  return sigmad_;
}

const IndexedVector3 & KineticsObserver::getCenterOfMass() const
{
  return com_;
}

const IndexedVector3 & KineticsObserver::getCenterOfMassDot() const
{
  return comd_;
}

const IndexedVector3 & KineticsObserver::getCenterOfMassDotDot() const
{
  return comdd_;
}

Vector6 KineticsObserver::getAdditionalWrench() const
{
  Vector6 additionalWrench;
  additionalWrench.segment<sizeForce>(0) = additionalForce_;
  additionalWrench.segment<sizeTorque>(sizeForce) = additionalTorque_;
  return additionalWrench;
}

} // namespace stateObservation
