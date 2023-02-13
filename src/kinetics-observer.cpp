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
void STATE_OBSERVATION_DLLAPI setBlockStateCovariance(Matrix & covMat, const Matrix & covBlock, int blockIndex)
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
  ekf_(stateSize_, stateTangentSize_, measurementSizeBase, measurementSizeBase, inputSize, false, false),
  finiteDifferencesJacobians_(true), withRungeKutta_(true), withGyroBias_(true), withUnmodeledWrench_(false),
  withAccelerationEstimation_(false), k_est_(0), k_data_(0), mass_(defaultMass), dt_(defaultdx), processNoise_(0x0),
  measurementNoise_(0x0), numberOfContactRealSensors_(0), currentIMUSensorNumber_(0),
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

  /*  The initialization of the observer is now called by MCKineticsObserver
  worldCentroidStateVector_.setZero();
  oldWorldCentroidStateVector_ = worldCentroidStateVector_;

  ekf_.setState(worldCentroidStateVector_, k_est_);
  updateKine_();
  */

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

const Vector & KineticsObserver::update()
{
  if(k_est_ != k_data_)
  {
    for(VectorContactIterator i = contacts_.begin(), ie = contacts_.end(); i != ie; ++i)
    {
      if(i->isSet)
      {
        BOOST_ASSERT((i->time == k_data_) && "The contacts have not all been updated. \
              Either remove lost contacts using removeContact \
              or Run updateContactWithWrenchSensor or updateContactWithNoSensor on every existing contact");

        /// the following code is only an attempt to maintain a consistent state of the state observer
        /// therefore we unset the state
        if(i->time != k_data_)
        {
          if(i->withRealSensor)
          {
            i->withRealSensor = false;
            numberOfContactRealSensors_--;
          }
          i->isSet = false;
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

    measurementVector_.resize(measurementSize_);
    measurementCovMatrix_.resize(measurementTangentSize_, measurementTangentSize_);
    measurementCovMatrix_.setZero();

    int curMeasIndex = 0;

    for(VectorIMUIterator i = imuSensors_.begin(), ie = imuSensors_.end(); i != ie; ++i)
    {
      if(i->time == k_data_)
      {
        i->measIndex = curMeasIndex;
        measurementVector_.segment<sizeIMUSignal>(curMeasIndex) = i->acceleroGyro;
        measurementCovMatrix_.block<sizeAcceleroSignal, sizeAcceleroSignal>(curMeasIndex, curMeasIndex) =
            i->covMatrixAccelero;
        curMeasIndex += sizeAcceleroSignal;
        measurementCovMatrix_.block<sizeGyroSignal, sizeGyroSignal>(curMeasIndex, curMeasIndex) = i->covMatrixGyro;
        curMeasIndex += sizeGyroSignal;
      }
    }

    for(VectorContactIterator i = contacts_.begin(), ie = contacts_.end(); i != ie; ++i)
    {
      if(i->withRealSensor)
      {
        i->measIndex = curMeasIndex;
        measurementVector_.segment<sizeWrench>(curMeasIndex) = i->wrenchMeasurement;
        measurementCovMatrix_.block<sizeWrench, sizeWrench>(curMeasIndex, curMeasIndex) = i->sensorCovMatrix();
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
    }

    ekf_.setMeasureSize(measurementSize_, measurementTangentSize_);
    ekf_.setMeasurement(measurementVector_, k_data_);
    ekf_.setR(measurementCovMatrix_);

    ekf_.updateStateAndMeasurementPrediction();

    if(finiteDifferencesJacobians_)
    {
      ekf_.setA(ekf_.getAMatrixFD(worldCentroidStateVectorDx_));
      predictedWorldCentroidState_ = ekf_.getLastPrediction();
      ekf_.setC(ekf_.getCMatrixFD(worldCentroidStateVectorDx_));
    }
    else
    {
      ekf_.setA(computeAMatrix_());
      predictedWorldCentroidState_ = ekf_.getLastPrediction();
      ekf_.setC(computeCMatrix_());
    }
    // std::cout << std::endl << "d_kine/d_kine : " << std::endl << ekf_.getA().block<12, 12>(posIndexTangent(),
    // posIndexTangent()) << std::endl;

    int j = 0;
    for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
    {
      if(i->isSet)
      {
        // std::cout << std::endl << "contact " + std::to_string(j) + " : Q : " << std::endl << ekf_.getQ().block<12,
        // 12>(contactPosIndexTangent(), posIndexTangent()) << std::endl; std::cout << std::endl << "contact " +
        // std::to_string(j) + " : d_kine/d_contact : " << std::endl << ekf_.getA().block<12, 12>(posIndexTangent(),
        // contactPosIndexTangent(i)) << std::endl; std::cout << std::endl << "contact " + std::to_string(j) + " :
        // d_contact/d_kine : " << std::endl << ekf_.getA().block<12, 12>(contactPosIndexTangent(i), posIndexTangent())
        // << std::endl; std::cout << std::endl << "contact " + std::to_string(j) + " : d_contact/d_contact : " <<
        // std::endl << ekf_.getA().block<12, 12>(contactPosIndexTangent(i), contactPosIndexTangent(i)) << std::endl;
        // std::cout << std::endl << "contact " + std::to_string(j) + " : Kinematics : " << std::endl <<
        // i->centroidContactKine << std::endl;
      }
      j++;
    }

    worldCentroidStateVector_ = ekf_.getEstimatedState(k_data_);

    if(worldCentroidStateVector_.hasNaN())
    {
#ifndef NDEBUG
      std::cout << "Kinetics observer: NaN value detected" << std::endl;
#endif
      worldCentroidStateVector_ = stateNaNCorrection_();
    }
    else
    {
      oldWorldCentroidStateVector_ = worldCentroidStateVector_;
    }

    ++k_est_; // the timestamp of the state we estimated

    worldCentroidStateKinematics_.reset();

    updateLocalKineAndContacts_(); // update of worldCentroidStateKinematics_ and of the contacts pose with the newly
                                   // estimated state
    if(withAccelerationEstimation_)
    {
      estimateAccelerations(); // update of worldCentroidStateKinematics_ with the accelerations
    }
    updateGlobalKine_();
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

Vector KineticsObserver::getPredictedGlobalCentroidState() const
{
  return predictedWorldCentroidState_;
}

std::vector<Vector> KineticsObserver::getPredictedAccelerometersGravityComponent() const
{
  return predictedAccelerometersGravityComponent_;
}

std::vector<Vector> KineticsObserver::getPredictedAccelerometers() const
{
  // std::cout << std::endl << "predictedAccelerometers_: " << std::endl << predictedAccelerometers_.at(0) << std::endl;
  return predictedAccelerometers_;
}

std::vector<Vector> KineticsObserver::getPredictedAccelerometersLinAccComponent() const
{
  return predictedWorldIMUsLinAcc_;
}

kine::LocalKinematics KineticsObserver::getLocalCentroidKinematics() const
{
  return worldCentroidStateKinematics_;
}

kine::LocalKinematics KineticsObserver::getLocalKinematicsOf(const LocalKinematics & locKin) const
{
  return LocalKinematics(worldCentroidStateKinematics_, locKin); /// product of the kinematics
}

kine::Kinematics KineticsObserver::getGlobalCentroidKinematics() const
{
  /* Care : unsafe access, for the moment the global kinematics are updated only after the update, so don't call this
   * function before*/
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

Vector6 KineticsObserver::getContactWrench(int contactNbr) const
{
  return worldCentroidStateVector_.segment<sizeWrench>(contactWrenchIndex(contactNbr));
}

kine::Kinematics KineticsObserver::getContactPosition(int contactNbr) const
{
  return Kinematics(worldCentroidStateVector_.segment<sizeStateKine>(contactKineIndex(contactNbr)), flagsContactKine);
}

Vector6 KineticsObserver::getUnmodeledWrench() const
{
  return worldCentroidStateVector_.segment<sizeWrench>(unmodeledWrenchIndex());
}

kine::LocalKinematics KineticsObserver::estimateAccelerations()
{
  Vector3 forceCentroid = additionalForce_;
  Vector3 torqueCentroid = additionalTorque_;

  addUnmodeledAndContactWrench_(worldCentroidStateVector_, forceCentroid, torqueCentroid);

  /// The accelerations are about to be computed so we set them to "initialized"
  worldCentroidStateKinematics_.linAcc.set(true);
  worldCentroidStateKinematics_.angAcc.set(true);

  computeLocalAccelerations_(worldCentroidStateKinematics_, forceCentroid, torqueCentroid,
                             worldCentroidStateKinematics_.linAcc(), worldCentroidStateKinematics_.angAcc());

  return worldCentroidStateKinematics_;
}

void KineticsObserver::setWorldCentroidStateKinematics(const LocalKinematics & kine,
                                                       bool resetForces,
                                                       bool resetCovariance)
{
  BOOST_ASSERT(kine.position.isSet() && kine.orientation.isSet() && kine.linVel.isSet() && kine.angVel.isSet()
               && "The Kinematics is not correctly initialized, should be the position, orientation, and linear and "
                  "angular verlocities");
  worldCentroidStateKinematics_ = kine;
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
}

void KineticsObserver::setAdditionalWrench(const Vector3 & force, const Vector3 & moment)
{

  additionalForce_ = force;
  additionalTorque_ = moment;
}

void KineticsObserver::setWithUnmodeledWrench(bool b)
{
  withUnmodeledWrench_ = b;
}

void KineticsObserver::setWithAccelerationEstimation(bool b)
{
  withAccelerationEstimation_ = b;
}

void KineticsObserver::setWithInnovation(bool b)
{
  ekf_.withInnovation = b;
}

void KineticsObserver::useRungeKutta(bool b)
{
  withRungeKutta_ = b;
}

bool KineticsObserver::getWithAccelerationEstimation() const
{
  return withAccelerationEstimation_;
}

void KineticsObserver::setWithGyroBias(bool b)
{
  withAccelerationEstimation_ = b;
}

int KineticsObserver::setIMU(const Vector3 & accelero,
                             const Vector3 & gyrometer,
                             const Kinematics & userImuKinematics,
                             int num)
{
  /// ensure the measurements are labeled with the good time stamp
  startNewIteration_();

  if(num < 0)
  {
    num = 0;
    while(imuSensors_[num].time != k_data_ && unsigned(num) < imuSensors_.size())
    {
      ++num;
    }
  }

  BOOST_ASSERT(unsigned(num) < maxImuNumber_ && "The inserted IMU number exceeds the maximum number");

  IMU & imu = imuSensors_[num]; /// reference

  BOOST_ASSERT(imu.time < k_data_ && "The IMU has been already set, use another number");

  imu.acceleroGyro.head<3>() = accelero;
  imu.acceleroGyro.tail<3>() = gyrometer;
  if(imuSensors_[num].time == 0) /// this is the first value for the IMU
  {
    imu.userImuKinematics = userImuKinematics;
    imu.centroidImuKinematics = LocalKinematics(imu.userImuKinematics);

    imu.covMatrixAccelero = acceleroCovMatDefault_;
    imu.covMatrixGyro = gyroCovMatDefault_;

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
  }
  else
  {
    imu.userImuKinematics.update(userImuKinematics, dt_ * (k_data_ - k_data_), flagsIMUKine);
    imu.centroidImuKinematics = LocalKinematics(imu.userImuKinematics);
  }

  imu.time = k_data_;
  ++currentIMUSensorNumber_;

  return num;
}

int KineticsObserver::setIMU(const Vector3 & accelero,
                             const Vector3 & gyrometer,
                             const Matrix3 & acceleroCov,
                             const Matrix3 & gyroCov,
                             const Kinematics & userImuKinematics,
                             int num)
{
  /// ensure the measuements are labeled with the good time stamp
  startNewIteration_();

  if(num < 0)
  {
    num = 0;
    while(imuSensors_[num].time != k_data_ && unsigned(num) < imuSensors_.size())
    {
      ++num;
    }
  }

  BOOST_ASSERT(unsigned(num) < maxImuNumber_ && "The inserted IMU number exceeds the maximum number");

  IMU & imu = imuSensors_[num]; /// reference

  BOOST_ASSERT(imu.time < k_data_ && "The IMU has been already set, use another number");

  imu.acceleroGyro.head<3>() = accelero;
  imu.acceleroGyro.tail<3>() = gyrometer;
  imu.covMatrixAccelero = acceleroCov;
  imu.covMatrixGyro = gyroCov;

  if(imuSensors_[num].time == 0) /// this is the first value for the IMU
  {
    imu.userImuKinematics = userImuKinematics;
    imu.centroidImuKinematics = LocalKinematics(imu.userImuKinematics);
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
  }
  else
  {
    imu.userImuKinematics.update(userImuKinematics, dt_ * (k_data_ - k_data_), flagsIMUKine);
    imu.centroidImuKinematics = LocalKinematics(imu.userImuKinematics);
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
  else /// the contact is newlyset
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

void KineticsObserver::setInertiaMatrix(const Matrix3 & I, const Matrix3 & I_dot)
{
  startNewIteration_();
  I_.set(I, k_data_);
  Id_.set(I_dot, k_data_);
}

void KineticsObserver::setInertiaMatrix(const Matrix3 & I)
{
  startNewIteration_();

  if(I_.getTime() < k_data_)
  {
    Id_.set(tools::derivate(I_(), I, dt_ * double(k_data_ - I_.getTime())), k_data_);
  }
  I_.set(I, k_data_);
}

void KineticsObserver::setInertiaMatrix(const Vector6 & Iv, const Vector6 & Iv_dot)
{
  startNewIteration_();

  I_.set();
  I_.setIndex(k_data_);
  fillSymmetricMatrix(I_(), Iv.head<3>(), Iv(3), Iv(4), Iv(5));

  Id_.set();
  Id_.setIndex(k_data_);
  fillSymmetricMatrix(Id_(), Iv_dot.head<3>(), Iv_dot(3), Iv_dot(4), Iv_dot(5));
}

void KineticsObserver::setInertiaMatrix(const Vector6 & Iv)
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

void KineticsObserver::setAngularMomentum(const Vector3 & sigma, const Vector3 & sigma_dot)
{
  startNewIteration_();
  sigma_.set(sigma, k_data_);
  sigmad_.set(sigma_dot, k_data_);
}

void KineticsObserver::setAngularMomentum(const Vector3 & sigma)
{
  startNewIteration_();
  if(sigma_.getTime() < k_data_)
  {
    sigmad_.set(tools::derivate(sigma_(), sigma, dt_ * double(k_data_ - sigma_.getTime())), k_data_);
  }
  sigma_.set(sigma, k_data_);
}

int KineticsObserver::addContact(const Kinematics & worldContactRefKine,
                                 const Matrix12 & initialCovarianceMatrix,
                                 const Matrix12 & processCovarianceMatrix,
                                 int contactNumber,
                                 const Matrix3 & linearStiffness,
                                 const Matrix3 & linearDamping,
                                 const Matrix3 & angularStiffness,
                                 const Matrix3 & angularDamping)
{

  BOOST_ASSERT(worldContactRefKine.position.isSet() && worldContactRefKine.orientation.isSet()
               && "The added contact pose is not initialized correctly (position and orientation)");

  if(contactNumber < 0) /// attributes the contact an index called contactNumber. Automatically attributes the
                        /// first available number in the range defined by the maximum amount of contacts
  {
    contactNumber = 0;

    while(unsigned(contactNumber) < maxContacts_ && contacts_[contactNumber].isSet)
    {
      ++contactNumber;
    }
  }

  BOOST_ASSERT(unsigned(contactNumber) < maxContacts_
               && "Trying to add contact: The contact number exceeds the maximum allowed, please give a number of "
                  "contact between 0 and maxContact-1");

  if(unsigned(contactNumber) >= maxContacts_) /// this is a bug-prone protection code that is here only to guarantee the
                                              /// consistence of the state
  {
    contactNumber = maxContacts_ - 1;
  }

  BOOST_ASSERT(!contacts_[contactNumber].isSet
               && "The contact already exists, please remove it before adding it again");

  Contact & contact = contacts_[contactNumber]; /// reference

  contact.isSet = true; /// set the contacts
  contact.stateIndex = contactsIndex() + contactNumber * sizeContact;
  contact.stateIndexTangent = contactsIndexTangent() + contactNumber * sizeContactTangent;
  contact.worldRefPose = worldContactRefKine;

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
  worldCentroidStateVector_.segment<sizeContact>(contact.stateIndex) << contact.worldRefPose.toVector(flagsContactKine),
      Vector6::Zero();
  ekf_.setState(worldCentroidStateVector_, k_est_);

  /// sets the initial covariance matrix
  Matrix stateCovMat = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeContactTangent>(stateCovMat, initialCovarianceMatrix, contact.stateIndexTangent);
  ekf_.setStateCovariance(stateCovMat);

  /// Sets the process cov mat
  Matrix processCovMat = ekf_.getQ();
  setBlockStateCovariance<sizeContactTangent>(processCovMat, processCovarianceMatrix, contact.stateIndexTangent);
  ekf_.setQ(processCovMat);

  // std::cout << std::endl << "ekf stateCovariance: " << std::endl << ekf_.getStateCovariance() << std::endl;
  // std::cout << std::endl << "ekf Q: " << std::endl << ekf_.getQ() << std::endl;

  return contactNumber;
}

/// version when the contact position is perfectly known
int KineticsObserver::addContact(const Kinematics & worldContactRefKine,
                                 int contactNumber,
                                 const Matrix3 & linearStiffness,
                                 const Matrix3 & linearDamping,
                                 const Matrix3 & angularStiffness,
                                 const Matrix3 & angularDamping)
{
  return addContact(worldContactRefKine, contactInitCovMatDefault_, contactProcessCovMatDefault_, contactNumber,
                    linearStiffness, linearDamping, angularStiffness, angularDamping);
}

void KineticsObserver::removeContact(int contactNbr)
{
  BOOST_ASSERT(contacts_[contactNbr].isSet && "Tried to remove a non-existing contact.");
  contacts_[contactNbr].isSet = false;
  if(contacts_[contactNbr].withRealSensor)
  {
    contacts_[contactNbr].withRealSensor = false;
    --numberOfContactRealSensors_;
  }
}

void KineticsObserver::clearContacts()
{
  contacts_.clear();
  numberOfContactRealSensors_ = 0;
}

Index KineticsObserver::getNumberOfContacts() const
{
  return contacts_.size();
}

std::vector<int> KineticsObserver::getListOfContacts() const
{
  std::vector<int> v;

  for(unsigned i = 0; i < contacts_.size(); ++i)
  {
    if(contacts_[i].isSet)
    {
      v.push_back(i);
    }
  }
  return v;
}

void KineticsObserver::setStateCovarianceMat(const Matrix & P)
{
  ekf_.setStateCovariance(P);
}

void KineticsObserver::setKinematicsStateCovariance(const Matrix & P_kine)
{
  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeStateKineTangent>(P, P_kine, kineIndexTangent());
  ekf_.setStateCovariance(P);
}

void KineticsObserver::setKinematicsInitCovarianceDefault(const Matrix & P_kine)
{
  stateKinematicsInitCovMat_ = P_kine;
}

void KineticsObserver::setGyroBiasStateCovariance(const Matrix3 & covMat, unsigned imuNumber)
{
  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeGyroBias>(P, covMat, gyroBiasIndexTangent(imuNumber));
  ekf_.setStateCovariance(P);
}

void KineticsObserver::setGyroBiasInitCovarianceDefault(const Matrix3 & covMat)
{
  gyroBiasInitCovMat_ = covMat;
}

void KineticsObserver::setGyroBiasProcessCovariance(const Matrix3 & covMat, unsigned imuNumber)
{
  Matrix P = ekf_.getProcessCovariance();
  setBlockStateCovariance<sizeGyroBias>(P, covMat, gyroBiasIndexTangent(imuNumber));
  ekf_.setProcessCovariance(P);
}

void KineticsObserver::setUnmodeledWrenchStateCovMat(const Matrix6 & currentCovMat)
{
  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeWrench>(P, currentCovMat, unmodeledWrenchIndexTangent());
  ekf_.setStateCovariance(P);
}

void KineticsObserver::setUnmodeledWrenchInitCovMatDefault(const Matrix6 & initCovMat)
{
  unmodeledWrenchInitCovMat_ = initCovMat;
}

void KineticsObserver::setUnmodeledWrenchProcessCovMat(const Matrix6 & processCovMat)
{
  Matrix P = ekf_.getProcessCovariance();
  setBlockStateCovariance<sizeWrench>(P, processCovMat, unmodeledWrenchIndexTangent());
  ekf_.setProcessCovariance(P);
}

void KineticsObserver::setContactStateCovMat(int contactNbr, const Matrix12 & contactCovMat)
{
  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeContactTangent>(P, contactCovMat, contactIndexTangent(contactNbr));
  ekf_.setStateCovariance(P);
}

void KineticsObserver::setContactInitCovMatDefault(const Matrix12 & contactCovMat)
{
  contactInitCovMatDefault_ = contactCovMat;
}

void KineticsObserver::setContactProcessCovMat(int contactNbr, const Matrix12 & contactCovMat)
{
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

void KineticsObserver::resetStateGyroBiasCovMat(unsigned i)
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

void KineticsObserver::resetStateContactCovMat(unsigned contactNbr)
{
  BOOST_ASSERT(contactNbr < contacts_.size() && contacts_[contactNbr].isSet
               && "Tried to set the covariance of a non existant contact");

  Matrix P = ekf_.getStateCovariance();
  setBlockStateCovariance<sizeContactTangent>(P, contactInitCovMatDefault_, contacts_[contactNbr].stateIndexTangent);
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

void KineticsObserver::resetProcessGyroBiasCovMat(unsigned i)
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

void KineticsObserver::resetProcessContactCovMat(unsigned contactNbr)
{
  BOOST_ASSERT(contactNbr < maxContacts_ && contacts_[contactNbr].isSet
               && "Tried to set the covariance of a non existant contact");

  Matrix P = ekf_.getProcessCovariance();
  setBlockStateCovariance<sizeContactTangent>(P, contactProcessCovMatDefault_, contacts_[contactNbr].stateIndexTangent);
  ekf_.setProcessCovariance(P);
}

void KineticsObserver::resetSensorsDefaultCovMat()
{
  acceleroCovMatDefault_ = Matrix3::Identity() * acceleroVarianceDefault;
  gyroCovMatDefault_ = Matrix3::Identity() * gyroVarianceDefault;
  contactWrenchSensorCovMatDefault_ = blockMat6(Matrix3::Identity() * forceSensorVarianceDefault, Matrix3::Zero(),
                                                Matrix3::Zero(), Matrix3::Identity() * torqueSensorVarianceDefault);
  absPoseSensorCovMatDefault_ = blockMat6(Matrix3::Identity() * positionSensorVarianceDefault, Matrix3::Zero(),
                                          Matrix3::Zero(), Matrix3::Identity() * orientationSensorVarianceDefault);
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
}

void KineticsObserver::setFiniteDifferenceStep(const Vector & v)
{
  worldCentroidStateVectorDx_ = v;
}

void KineticsObserver::useFiniteDifferencesJacobians(bool b)
{
  finiteDifferencesJacobians_ = b;
}

Vector KineticsObserver::stateNaNCorrection_()
{
  /// TODO implement this function
  assert(false && "NaN Correction not yet implemented. Please Contact mehdi.benallegue@gmail.com");
  return oldWorldCentroidStateVector_;
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

Matrix KineticsObserver::computeAMatrix_()
{
  estimateAccelerations(); // update of worldCentroidStateKinematics_ with the accelerations

  const Vector & statePrediction = ekf_.updateStatePrediction();
  const Vector3 & predictedWorldCentroidStatePos = statePrediction.segment<sizePos>(posIndex());
  Orientation predictedWorldCentroidStateOri;
  predictedWorldCentroidStateOri.fromVector4(statePrediction.segment<sizeOri>(oriIndex())).toMatrix3();
  const Vector3 & predictedWorldCentroidStateLinVel = statePrediction.segment<sizeLinVel>(linVelIndex());
  const Vector3 & predictedWorldCentroidStateAngVel = statePrediction.segment<sizeAngVel>(angVelIndex());

  Matrix A = Eigen::MatrixXd::Zero(stateTangentSize_, stateTangentSize_);


  double dt2_2 = 0.5 * pow(dt_, 2);
  Matrix3 dt2_2_Sp = dt2_2 * kine::skewSymmetric(worldCentroidStateKinematics_.position());

  // Jacobians of the angular acceleration
  Matrix3 I_inv = I_().inverse();
  Matrix3 J_omegadot_ext_torque = I_inv; // we can merge the two last lines
                                         // not to create a new variable.
                                         // We consider that the inertia matrix is given in the centroid's frame.

  Matrix3 J_omegadot_omega =
      I_inv
      * (kine::skewSymmetric(I_() * worldCentroidStateKinematics_.angVel()) - Id_()
         - kine::skewSymmetric(worldCentroidStateKinematics_.angVel()) * I_() + kine::skewSymmetric(sigma_()));

  // Jacobians of the linear acceleration
  Matrix3 J_al_R =
      -cst::gravityConstant
      * (worldCentroidStateKinematics_.orientation.toMatrix3().transpose() * kine::skewSymmetric(Vector3(0, 0, 1)));
  Matrix3 J_al_ext_force = Matrix::Identity(sizeLinAccTangent, sizeTorqueTangent) / mass_;

  // Jacobians of the centroid's position
  Matrix3 J_pl_pl = Matrix::Identity(sizePosTangent, sizePosTangent)
                    - dt_ * kine::skewSymmetric(worldCentroidStateKinematics_.angVel())
                    + dt2_2
                          * (kine::skewSymmetric2(worldCentroidStateKinematics_.angVel())
                             - kine::skewSymmetric(worldCentroidStateKinematics_.angAcc()));

  A.block<sizePosTangent, sizePosTangent>(posIndexTangent(), posIndexTangent()) =
      J_pl_pl; // For optimization, we can merge the two last operations into one
  Matrix3 J_pl_R = dt2_2 * J_al_R;
  A.block<sizePosTangent, sizeOriTangent>(posIndexTangent(), oriIndexTangent()) = J_pl_R;
  Matrix3 J_pl_vl = dt_ * Matrix::Identity(sizePosTangent, sizeLinVelTangent)
                    - 2 * dt2_2 * kine::skewSymmetric(worldCentroidStateKinematics_.angVel());
  A.block<sizePosTangent, sizeLinVelTangent>(posIndexTangent(), linVelIndexTangent()) = J_pl_vl;
  Matrix3 J_pl_omega = kine::skewSymmetric(dt_ * worldCentroidStateKinematics_.position()
                                           + 2 * dt2_2 * worldCentroidStateKinematics_.linVel())
                       + dt2_2
                             * (kine::skewSymmetric(worldCentroidStateKinematics_.position()) * J_omegadot_omega
                                + kine::skewSymmetric(kine::skewSymmetric(worldCentroidStateKinematics_.position())
                                                      * worldCentroidStateKinematics_.angVel())
                                - kine::skewSymmetric(worldCentroidStateKinematics_.angVel())
                                      * kine::skewSymmetric(worldCentroidStateKinematics_.position()));
  A.block<sizePosTangent, sizeAngVelTangent>(posIndexTangent(), angVelIndexTangent()) = J_pl_omega;
  Matrix3 J_pl_ext_force = dt2_2 / mass_ * Matrix::Identity(sizePosTangent, sizeForceTangent);
  A.block<sizePosTangent, sizeForceTangent>(posIndexTangent(), unmodeledForceIndexTangent()) = J_pl_ext_force;

  // Jacobians of the orientation
  Vector delta = dt_ * worldCentroidStateKinematics_.angVel() + dt2_2 * worldCentroidStateKinematics_.angAcc();
  double sq_norm_delta = delta.squaredNorm();
  double norm_delta = delta.norm();
  double sin_delta_2 = sin(norm_delta / 2);

  Matrix3 J_R_delta;
  if(norm_delta > cst::epsilonAngle)
  {
    J_R_delta.noalias() = 1 / norm_delta
                          * (((norm_delta - 2 * sin_delta_2) / (2 * sq_norm_delta))
                                 * (worldCentroidStateKinematics_.orientation.toMatrix3() * delta * delta.transpose())
                             + sin_delta_2
                                   * (worldCentroidStateKinematics_.orientation.toMatrix3()
                                      * kine::rotationVectorToRotationMatrix(delta / 2)));
  }
  else
  {
    J_R_delta.noalias() =
        0.5 * worldCentroidStateKinematics_.orientation.toMatrix3() * kine::rotationVectorToRotationMatrix(delta / 2);
  }

  // the intermediate jacobian used to compute the ones with respect to the angular velocity and acceleration
  Matrix3 J_R_omegadot = J_R_delta * dt2_2; // used in other Jacobians

  Matrix3 J_R_R = Matrix::Identity(sizeOriTangent, sizeOriTangent);
  A.block<sizeOriTangent, sizeOriTangent>(oriIndexTangent(), oriIndexTangent()) = J_R_R;
  Matrix3 J_R_omega =
      J_R_delta * (dt_ * Matrix::Identity(sizeAngVelTangent, sizeAngVelTangent) + dt2_2 * J_omegadot_omega);
  A.block<sizeOriTangent, sizeAngVelTangent>(oriIndexTangent(), angVelIndexTangent()) = J_R_omega;
  Matrix3 J_R_ext_torque = J_R_omegadot * J_omegadot_ext_torque;
  A.block<sizeOriTangent, sizeTorqueTangent>(oriIndexTangent(), unmodeledTorqueIndexTangent()) = J_R_ext_torque;

  // Jacobians of the linear velocity
  Matrix3 J_vl_R = dt_ * J_al_R;
  A.block<sizeLinVelTangent, sizeOriTangent>(linVelIndexTangent(), oriIndexTangent()) = J_vl_R;
  Matrix3 J_vl_vl = Matrix::Identity(sizeLinVelTangent, sizeLinVelTangent)
                    - dt_ * kine::skewSymmetric(worldCentroidStateKinematics_.angVel());
  A.block<sizeLinVelTangent, sizeLinVelTangent>(linVelIndexTangent(), linVelIndexTangent()) = J_vl_vl;
  Matrix3 J_vl_omega = dt_ * kine::skewSymmetric(worldCentroidStateKinematics_.linVel());
  A.block<sizeLinVelTangent, sizeAngVelTangent>(linVelIndexTangent(), angVelIndexTangent()) = J_vl_omega;
  Matrix3 J_vl_ext_force = dt_ * J_al_ext_force;
  A.block<sizeLinVelTangent, sizeAngVelTangent>(linVelIndexTangent(), unmodeledForceIndexTangent()) = J_vl_ext_force;

  // Jacobians of the angular velocity
  Matrix3 J_omega_omega = dt_ * J_omegadot_omega;
  A.block<sizeAngVelTangent, sizeAngVelTangent>(angVelIndexTangent(), angVelIndexTangent()) = J_omega_omega;
  Matrix3 J_omega_ext_torque = dt_ * J_omegadot_ext_torque;
  A.block<sizeAngVelTangent, sizeTorqueTangent>(angVelIndexTangent(), unmodeledTorqueIndexTangent()) =
      J_omega_ext_torque;

  // Jacobians of the gyrometer bias
  if(withGyroBias_)
  {
    Matrix3 J_gyrobias_gyrobias = Matrix::Identity(sizeGyroBiasTangent, sizeGyroBiasTangent);
    for(unsigned i = 0; i < imuSensors_.size(); ++i)
    {
      A.block<sizeGyroBiasTangent, sizeGyroBiasTangent>(gyroBiasIndexTangent(i), gyroBiasIndexTangent(i)) =
          J_gyrobias_gyrobias;
    }
  }

  // Jacobians of the unmodeled external force
  Matrix3 J_ext_force_ext_force = Matrix::Identity(sizeForceTangent, sizeForceTangent);
  A.block<sizeForceTangent, sizeForceTangent>(unmodeledForceIndexTangent(), unmodeledForceIndexTangent()) =
      J_ext_force_ext_force;

  // Jacobians of the unmodeled external torque
  Matrix3 J_ext_torque_ext_torque = Matrix::Identity(sizeTorqueTangent, sizeTorqueTangent);
  A.block<sizeTorqueTangent, sizeTorqueTangent>(unmodeledTorqueIndexTangent(), unmodeledTorqueIndexTangent()) =
      J_ext_torque_ext_torque;

  // Jacobians with respect to contacts
  Matrix3 J_poscontact_poscontact =
      Matrix::Identity(sizePosTangent, sizePosTangent); // out of the loop as it is constant
                                                        // but then creates a useless variable if there is no contact
  for(VectorContactConstIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      Orientation predictedStateContactOri;
      predictedStateContactOri.fromVector4(statePrediction.segment<sizeOri>(contactOriIndex(i))).toMatrix3();

      // Jacobian of the linar acceleration with respect to the contact force
      Matrix3 J_linAcc_Fcis = (1 / mass_) * i->centroidContactKine.orientation.toMatrix3();
      // Jacobian of the angular acceleration with respect to the contact force
      Matrix3 J_omegadot_Fcis = (I_inv * kine::skewSymmetric(i->centroidContactKine.position()))
                                * i->centroidContactKine.orientation.toMatrix3();
      // Jacobian of the angular acceleration with respect to the contact torque
      Matrix3 J_omegadot_Tcis = I_inv * i->centroidContactKine.orientation.toMatrix3();

      // Jacobian of the centroid's position with respect to the contact force
      Matrix3 J_pl_contactForce = dt2_2 * J_linAcc_Fcis;
      A.block<sizePosTangent, sizeForceTangent>(posIndexTangent(), contactForceIndexTangent(i)) = J_pl_contactForce;
      // Jacobians of the orientation with respect to the contact force and torque
      Matrix3 J_R_contactForce = J_R_omegadot * J_omegadot_Fcis;
      A.block<sizeOriTangent, sizeForceTangent>(oriIndexTangent(), contactForceIndexTangent(i)) = J_R_contactForce;
      Matrix3 J_R_contactTorque = J_R_omegadot * J_omegadot_Tcis;
      A.block<sizeOriTangent, sizeTorqueTangent>(oriIndexTangent(), contactTorqueIndexTangent(i)) = J_R_contactTorque;
      // Jacobian of the linear velocity with respect to the contact force
      Matrix3 J_vl_contactForce = dt_ * J_linAcc_Fcis;
      A.block<sizeLinVelTangent, sizeForceTangent>(linVelIndexTangent(), contactForceIndexTangent(i)) =
          J_vl_contactForce;
      // Jacobian of the angular velocity with respect to the contact force and torque
      Matrix3 J_omega_contactForce = dt_ * J_omegadot_Fcis;
      A.block<sizeAngVelTangent, sizeForceTangent>(angVelIndexTangent(), contactForceIndexTangent(i)) =
          J_omega_contactForce;
      Matrix3 J_omega_contactTorque = dt_ * J_omegadot_Tcis;
      A.block<sizeAngVelTangent, sizeTorqueTangent>(angVelIndexTangent(), contactTorqueIndexTangent(i)) =
          J_omega_contactTorque;

      // Jacobian of the contact position and orientation with respect to themselves
      A.block<sizePosTangent, sizePosTangent>(contactPosIndexTangent(i), contactPosIndexTangent(i)) =
          J_poscontact_poscontact;
      Matrix3 J_contactOri_contactOri = J_poscontact_poscontact;
      A.block<sizeOriTangent, sizeOriTangent>(contactOriIndexTangent(i), contactOriIndexTangent(i)) =
          J_contactOri_contactOri;

      Orientation contactWorldOri(
          Matrix3(i->centroidContactKine.orientation.toMatrix3().transpose()
                  * predictedWorldCentroidStateOri.toMatrix3()
                        .transpose())); // better to compute it now as it is used in several expressions
      // Jacobians of the contacts force
      Matrix3 J_contactForce_pl_at_same_time =
          -(contactWorldOri.toMatrix3() * i->linearStiffness * predictedWorldCentroidStateOri.toMatrix3());
      Vector3 sumVelContact = i->centroidContactKine.linVel()
                              + predictedWorldCentroidStateAngVel.cross(i->centroidContactKine.position())
                              + predictedWorldCentroidStateLinVel;
      Matrix3 J_contactForce_R_at_same_time =
          contactWorldOri.toMatrix3()
          * (i->linearStiffness
                 * kine::skewSymmetric(predictedWorldCentroidStateOri.toMatrix3()
                                       * (i->centroidContactKine.position() + predictedWorldCentroidStatePos))
             + i->linearDamping * kine::skewSymmetric(predictedWorldCentroidStateOri.toMatrix3() * sumVelContact)
             - kine::skewSymmetric(i->linearStiffness
                                       * (predictedWorldCentroidStateOri.toMatrix3()
                                          * (i->centroidContactKine.position() + predictedWorldCentroidStatePos))
                                   + i->linearDamping * (predictedWorldCentroidStateOri.toMatrix3() * sumVelContact)
                                   - i->linearStiffness * i->centroidContactKine.position()));
      Matrix3 J_contactForce_vl_at_same_time =
          -(contactWorldOri.toMatrix3() * i->linearDamping * predictedWorldCentroidStateOri.toMatrix3());
      Matrix3 J_contactForce_omega_at_same_time =
          contactWorldOri.toMatrix3() * i->linearDamping
          * (predictedWorldCentroidStateOri.toMatrix3() * kine::skewSymmetric(i->centroidContactKine.position()));
      Matrix3 J_contactForce_contactPosition_at_same_time =
          contactWorldOri.toMatrix3() * i->linearStiffness * predictedWorldCentroidStateOri.toMatrix3();

      A.block<sizeForceTangent, sizePosTangent>(contactForceIndexTangent(i), posIndexTangent()) =
          J_contactForce_pl_at_same_time * J_pl_pl;
      A.block<sizeForceTangent, sizeOriTangent>(contactForceIndexTangent(i), oriIndexTangent()) =
          J_contactForce_pl_at_same_time * J_pl_R + J_contactForce_R_at_same_time
          + J_contactForce_vl_at_same_time * J_vl_R;
      A.block<sizeForceTangent, sizeLinVelTangent>(contactForceIndexTangent(i), linVelIndexTangent()) =
          J_contactForce_pl_at_same_time * J_pl_vl;
      A.block<sizeForceTangent, sizeAngVelTangent>(contactForceIndexTangent(i), angVelIndexTangent()) =
          J_contactForce_pl_at_same_time * J_pl_omega + J_contactForce_R_at_same_time * J_R_omega
          + J_contactForce_vl_at_same_time * J_vl_omega + J_contactForce_omega_at_same_time * J_omega_omega;
      A.block<sizeForceTangent, sizeForceTangent>(contactForceIndexTangent(i), unmodeledForceIndexTangent()) =
          J_contactForce_pl_at_same_time * J_pl_ext_force + J_contactForce_vl_at_same_time * J_vl_ext_force;
      A.block<sizeForceTangent, sizeTorqueTangent>(contactForceIndexTangent(i), unmodeledTorqueIndexTangent()) =
          J_contactForce_R_at_same_time * J_R_ext_torque + J_contactForce_omega_at_same_time * J_omega_ext_torque;
      A.block<sizeForceTangent, sizePosTangent>(contactForceIndexTangent(i), contactPosIndexTangent(i)) =
          J_contactForce_contactPosition_at_same_time;
      A.block<sizeForceTangent, sizeForceTangent>(contactForceIndexTangent(i), contactForceIndexTangent(i)) =
          J_contactForce_pl_at_same_time * J_pl_contactForce + J_contactForce_R_at_same_time * J_R_contactForce
          + J_contactForce_vl_at_same_time * J_vl_contactForce
          + J_contactForce_omega_at_same_time * J_omega_contactForce;
      A.block<sizeForceTangent, sizeTorqueTangent>(contactForceIndexTangent(i), contactTorqueIndexTangent(i)) =
          J_contactForce_R_at_same_time * J_R_contactTorque + J_contactForce_omega_at_same_time * J_omega_contactTorque;

      // Jacobians of the contacts torque
      Vector3 angVelSum =
          predictedWorldCentroidStateOri * (i->centroidContactKine.angVel() + predictedWorldCentroidStateAngVel);
      Vector3 ex = Vector3(1, 0, 0);
      Vector3 ey = Vector3(0, 1, 0);
      Vector3 ez = Vector3(0, 0, 1);
      Orientation RRefContactToWorld =
          predictedWorldCentroidStateOri * i->centroidContactKine.orientation * predictedStateContactOri.inverse();
      Orientation RWorldToRefContact = RRefContactToWorld.inverse();

      Matrix3 Vk = -ex * ez.transpose()
                       * (kine::skewSymmetric(RRefContactToWorld.toMatrix3() * ey)
                          + RWorldToRefContact.toMatrix3() * kine::skewSymmetric(ey))
                   - ey * ex.transpose()
                         * (kine::skewSymmetric(RRefContactToWorld.toMatrix3() * ez)
                            + RWorldToRefContact.toMatrix3() * kine::skewSymmetric(ez))
                   - ez * ey.transpose()
                         * (kine::skewSymmetric(RRefContactToWorld.toMatrix3() * ex)
                            + RWorldToRefContact.toMatrix3() * kine::skewSymmetric(ex));
      Matrix3 J_contactTorque_R_at_same_time =
          -(contactWorldOri.toMatrix3()
            * (kine::skewSymmetric(2 * i->angularStiffness
                                       * (kine::rotationMatrixToRotationVector(RRefContactToWorld.toMatrix3()
                                                                               - RWorldToRefContact.toMatrix3()))
                                   + i->angularDamping * angVelSum)
               + 2 * i->angularStiffness * Vk - i->angularDamping * kine::skewSymmetric(angVelSum)));
      Matrix3 J_contactTorque_omega_at_same_time =
          -(contactWorldOri.toMatrix3() * i->angularDamping * predictedWorldCentroidStateOri.toMatrix3());
      Matrix3 J_contactTorque_contactOri_at_same_time =
          2 * (contactWorldOri.toMatrix3() * i->angularStiffness * (predictedWorldCentroidStateOri.toMatrix3() * Vk));

      A.block<sizeTorqueTangent, sizeOriTangent>(contactTorqueIndexTangent(i), oriIndexTangent()) =
          J_contactTorque_R_at_same_time;
      A.block<sizeTorqueTangent, sizeAngVelTangent>(contactTorqueIndexTangent(i), angVelIndexTangent()) =
          J_contactTorque_R_at_same_time * J_R_omega + J_contactTorque_omega_at_same_time * J_omega_omega;
      A.block<sizeTorqueTangent, sizeTorqueTangent>(contactTorqueIndexTangent(i), unmodeledTorqueIndexTangent()) =
          J_contactTorque_R_at_same_time * J_R_ext_torque + J_contactTorque_omega_at_same_time * J_omega_ext_torque;
      A.block<sizeTorqueTangent, sizeForceTangent>(contactTorqueIndexTangent(i), contactOriIndexTangent(i)) =
          J_contactTorque_contactOri_at_same_time;
      A.block<sizeTorqueTangent, sizeForceTangent>(contactTorqueIndexTangent(i), contactForceIndexTangent(i)) =
          J_contactTorque_R_at_same_time * J_R_contactForce + J_contactTorque_omega_at_same_time * J_omega_contactForce;
      A.block<sizeTorqueTangent, sizeTorqueTangent>(contactTorqueIndexTangent(i), contactTorqueIndexTangent(i)) =
          J_contactTorque_R_at_same_time * J_R_contactTorque
          + J_contactTorque_omega_at_same_time * J_omega_contactTorque;
    }
  }

  return A;
}

Matrix KineticsObserver::computeCMatrix_()
{
  return ekf_.getCMatrixFD(worldCentroidStateVectorDx_);
}

void KineticsObserver::convertUserToCentroidFrame_(const Kinematics & userKine,
                                                   Kinematics & centroidKine,
                                                   TimeIndex k_data)
{
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
                                                                           TimeIndex k_data)
{
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

/*
void KineticsObserver::convertUserToCentroidFrame_(const LocalKinematics & userKine, LocalKinematics & centroidKine,
TimeIndex k_data)
{
  BOOST_ASSERT((com_.getTime() == k_data && com_.getTime() == comd_.getTime() && com_.getTime() == comdd_.getTime()) &&
"The Center of Mass must be actualized before the conversion"); centroidKine.position = userKine.position() - com_(); if
(userKine.linVel.isSet())
  {
    centroidKine.linVel = userKine.linVel() - comd_();
  }
  if (userKine.linAcc.isSet())
  {
    centroidKine.linAcc = userKine.linAcc() - comdd_();
  }
}
*/

void KineticsObserver::updateLocalKineAndContacts_()
{
  worldCentroidStateKinematics_.fromVector(worldCentroidStateVector_.segment<sizeStateKine>(kineIndex()),
                                           flagsStateKine);
  for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      i->worldRefPose.fromVector(worldCentroidStateVector_.segment<sizePose>(contactPosIndex(i)), flagsContactKine);
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
  // std::cout << std::endl << "force: " << std::endl << force << std::endl;
  force += worldCentroidStateVector.segment<sizeForce>(unmodeledWrenchIndex());
  // std::cout << std::endl << "force2: " << std::endl << force << std::endl;
  torque += worldCentroidStateVector.segment<sizeForce>(unmodeledTorqueIndex());

  for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      Kinematics & centroidContactKinei = i->centroidContactKine;
      Vector3 centroidContactForcei =
          centroidContactKinei.orientation * worldCentroidStateVector.segment<sizeForce>(contactForceIndex(i));
      // std::cout << std::endl << "centroidContactKinei.orientation: " << std::endl <<
      // centroidContactKinei.orientation.toRotationVector() << std::endl; std::cout << std::endl <<
      // "worldCentroidStateVector.segment<sizeForce>(contactForceIndex(i)): " << std::endl <<
      // worldCentroidStateVector.segment<sizeForce>(contactForceIndex(i)) << std::endl;
      force += centroidContactForcei;
      // std::cout << std::endl << "force3: " << std::endl << force << std::endl;
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

void KineticsObserver::computeRecursiveGlobalAccelerations_(Kinematics & predictedWorldCentroidKinematics)
{
  BOOST_ASSERT(false && "this function must be checked");
  Vector3 predictedVEContactForces = Vector3::Zero();
  Vector3 predictedVEContactTorques = Vector3::Zero();

  LocalKinematics predictedLocalWorldCentroidKinematics(predictedWorldCentroidKinematics);

  computeContactForces_(predictedLocalWorldCentroidKinematics, predictedVEContactForces, predictedVEContactTorques);

  Vector3 diffForces =
      predictedVEContactForces - initialVEContactForces_; // computing the increment of force occuring during the
                                                          // current integration step of the rungeKutta
  Vector3 diffTorques = predictedVEContactTorques - initialVEContactTorques_;

  Vector3 & currentStepContactForces = diffForces;
  Vector3 & currentStepContactTorques = diffTorques;

  currentStepContactForces += initTotalCentroidForce_; // Corresponds to the increment contactForce
  currentStepContactTorques += initTotalCentroidTorque_;

  predictedLocalWorldCentroidKinematics.linAcc =
      predictedLocalWorldCentroidKinematics.orientation * (currentStepContactForces / mass_) - cst::gravity;

  predictedLocalWorldCentroidKinematics.angAcc =
      predictedLocalWorldCentroidKinematics.orientation.toMatrix3() * I_().inverse()
      * (currentStepContactTorques
         - Id_() * predictedLocalWorldCentroidKinematics.orientation.toMatrix3().transpose()
               * predictedLocalWorldCentroidKinematics.angVel()
         - sigmad_()
         - (predictedLocalWorldCentroidKinematics.orientation.toMatrix3().transpose()
            * predictedLocalWorldCentroidKinematics.angVel())
               .cross(I_() * predictedLocalWorldCentroidKinematics.orientation.toMatrix3().transpose()
                          * predictedLocalWorldCentroidKinematics.angVel()
                      + sigma_()));
}

void KineticsObserver::computeRecursiveLocalAccelerations_(LocalKinematics & predictedWorldCentroidKinematics)
{
  Kinematics globWorldCentroidStateKinematics(predictedWorldCentroidKinematics);

  Vector3 predictedVEContactForces = Vector3::Zero();
  Vector3 predictedVEContactTorques = Vector3::Zero();

  computeContactForces_(predictedWorldCentroidKinematics, predictedVEContactForces, predictedVEContactTorques);

  Vector3 diffForces =
      predictedVEContactForces - initialVEContactForces_; // computing the increment of force occuring during the
                                                          // current integration step of the rungeKutta
  Vector3 diffTorques = predictedVEContactTorques - initialVEContactTorques_;

  Vector3 & currentStepContactForces = diffForces;
  Vector3 & currentStepContactTorques = diffTorques;

  currentStepContactForces += initTotalCentroidForce_; // Corresponds to the increment contactForce
  currentStepContactTorques += initTotalCentroidTorque_;

  predictedWorldCentroidKinematics.linAcc =
      (currentStepContactForces / mass_)
      - predictedWorldCentroidKinematics.orientation.toMatrix3().transpose() * cst::gravity;
  predictedWorldCentroidKinematics.angAcc =
      I_().inverse()
      * (currentStepContactTorques - Id_() * predictedWorldCentroidKinematics.angVel() - sigmad_()
         - predictedWorldCentroidKinematics.angVel().cross(I_() * predictedWorldCentroidKinematics.angVel()
                                                           + sigma_()));
}

void KineticsObserver::computeContactForce_(VectorContactIterator i,
                                            LocalKinematics & worldCentroidStateKinematics,
                                            Kinematics & worldReferenceContactPose,
                                            Vector3 & contactForce,
                                            Vector3 & contactTorque)
{
  Contact & contact = *i;

  Kinematics & centroidContactKine = contact.centroidContactKine; // the kinematics of the contact in the centroid's
                                                                  // frame, expressed in the centroid's frame
  Kinematics & worldContactPose = contact.temp.worldContactPose;
  worldContactPose.setToProductNoAlias(
      Kinematics(worldCentroidStateKinematics),
      centroidContactKine); // the kinematics of the contact in the world frame, expressed in the contact's frame

  contactForce = worldCentroidStateKinematics.orientation.toMatrix3().transpose()
                 * (contact.linearStiffness * (worldReferenceContactPose.position() - worldContactPose.position())
                    - contact.linearDamping * worldContactPose.linVel());

  contactTorque = worldCentroidStateKinematics.orientation.toMatrix3().transpose()
                  * (-0.5 * contact.angularStiffness
                         * (worldContactPose.orientation.toQuaternion()
                            * worldReferenceContactPose.orientation.toQuaternion().inverse())
                               .vec()
                     - contact.angularDamping * worldContactPose.angVel());
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

      Kinematics & centroidContactKine = contact.centroidContactKine; // the kinematics of the contact in the centroid's
                                                                      // frame, expressed in the centroid's frame
      Kinematics & worldContactPose = contact.temp.worldContactPose;
      Kinematics & worldReferenceContactPose = contact.worldRefPose;

      worldContactPose.setToProductNoAlias(
          Kinematics(worldCentroidStateKinematics),
          centroidContactKine); // the kinematics of the contact in the world frame, expressed in the contact's frame

      Vector3 centroidContactForce =
          worldCentroidStateKinematics.orientation.toMatrix3().transpose()
          * (contact.linearStiffness * (worldReferenceContactPose.position() - worldContactPose.position())
             - contact.linearDamping * worldContactPose.linVel());
      contactForce += centroidContactForce;

      Vector3 centroidContactTorque = worldCentroidStateKinematics.orientation.toMatrix3().transpose()
                                      * (-0.5 * contact.angularStiffness
                                             * (worldContactPose.orientation.toQuaternion()
                                                * worldReferenceContactPose.orientation.toQuaternion().inverse())
                                                   .vec()
                                         - contact.angularDamping * worldContactPose.angVel());

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

  int currentMeasurementSize = sizeIMUSignal * currentIMUSensorNumber_ + sizeWrench * numberOfContactRealSensors_;

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
  }
}

Vector KineticsObserver::stateDynamics(const Vector & xInput, const Vector & /*unused*/, TimeIndex)
{
  Vector x = xInput;

  initTotalCentroidForce_ =
      additionalForce_; // initialization of the total force at the centroid with the input additional forces.
  initTotalCentroidTorque_ = additionalTorque_;

  addUnmodeledAndContactWrench_(x, initTotalCentroidForce_,
                                initTotalCentroidTorque_); // adding the previously estimated contact and unmodeled
                                                           // forces to obtain the total force at the centroid

  LocalKinematics worldCentroidStateKinematics(x.segment<sizeStateKine>(kineIndex()), flagsStateKine);

  /// The accelerations are about to be computed so we set them to "initialized"
  worldCentroidStateKinematics.linAcc.set(true);
  worldCentroidStateKinematics.angAcc.set(true);

  Vector3 & linacc = worldCentroidStateKinematics.linAcc(); /// reference (Vector3&)
  Vector3 & angacc = worldCentroidStateKinematics.angAcc(); /// reference

  Kinematics globWorldCentroidStateKinematics = Kinematics(worldCentroidStateKinematics);

  computeLocalAccelerations_(worldCentroidStateKinematics, initTotalCentroidForce_, initTotalCentroidTorque_, linacc,
                             angacc);

  LocalKinematics kineTest = worldCentroidStateKinematics;
  if(withRungeKutta_)
  {
    initialVEContactForces_ = Vector3::Zero();
    initialVEContactTorques_ = Vector3::Zero();
    computeContactForces_(worldCentroidStateKinematics, initialVEContactForces_, initialVEContactTorques_);

    worldCentroidStateKinematics.integrateRungeKutta4(dt_, *this);
    kineTest.integrate(dt_);
  }
  else
  {
    worldCentroidStateKinematics.integrate(dt_);
  }

  x.segment<sizeStateKine>(kineIndex()) = worldCentroidStateKinematics.toVector(flagsStateKine);

  globWorldCentroidStateKinematics = Kinematics(worldCentroidStateKinematics);

  for(VectorContactIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      Kinematics & centroidContactKine = i->centroidContactKine;
      Kinematics worldContactRefPose; // not using the variable belonging to Contact as this variable must change only
                                      // at the end of the update
      worldContactRefPose.fromVector(x.segment<sizePose>(contactPosIndex(i)), flagsPoseKine);

      Matrix3 & Kpt = i->linearStiffness;
      Matrix3 & Kdt = i->linearDamping;
      Matrix3 & Kpr = i->angularStiffness;
      Matrix3 & Kdr = i->angularDamping;

      Kinematics & worldContactPose =
          i->temp.worldContactPose; // the positon of the contact in the world frame, expressed in the contact's frame
      worldContactPose.setToProductNoAlias(globWorldCentroidStateKinematics, centroidContactKine);

      x.segment<sizeForce>(contactForceIndex(i)) =
          -(worldContactPose.orientation.toMatrix3().transpose()
            * (Kpt * (worldContactPose.position() - worldContactRefPose.position()) + Kdt * worldContactPose.linVel()));

      x.segment<sizeTorque>(contactTorqueIndex(i)) =
          -2
          * (worldContactPose.orientation.toMatrix3().transpose()
             * (Kpr
                    * kine::vectorComponent((worldContactPose.orientation.toQuaternion()
                                             * worldContactRefPose.orientation.toQuaternion().inverse()))
                + Kdr * worldContactPose.angVel()));
    }
  }

  if(processNoise_ != 0x0)
  {
    processNoise_->getNoisy(x);
  }

  return x;
}

Vector KineticsObserver::measureDynamics(const Vector & x, const Vector & /*unused*/, TimeIndex k)
{

  Vector y(getMeasurementSize());

  Vector3 forceCentroid = additionalForce_;
  Vector3 torqueCentroid = additionalTorque_;

  addUnmodeledAndContactWrench_(x, forceCentroid, torqueCentroid);

  LocalKinematics worldCentroidStateKinematics(x.segment<sizeStateKine>(kineIndex()), flagsStateKine);

  /// The accelerations are about to be computed so we set them to "initialized"
  worldCentroidStateKinematics.linAcc.set(true);
  worldCentroidStateKinematics.angAcc.set(true);

  Vector3 & linacc = worldCentroidStateKinematics.linAcc();
  Vector3 & angacc = worldCentroidStateKinematics.angAcc();
  // std::cout << std::endl << "torqueCentroid: " << std::endl << torqueCentroid << std::endl;
  computeLocalAccelerations_(worldCentroidStateKinematics, forceCentroid, torqueCentroid, linacc, angacc);
  // std::cout << std::endl << "linacc2: " << std::endl << linacc << std::endl;
  // std::cout << std::endl << "worldCentroidStateKinematics: " << std::endl << worldCentroidStateKinematics <<
  // std::endl;

  LocalKinematics & worldImuKinematics = opt_.locKine;

  predictedAccelerometersGravityComponent_
      .clear(); // just for debug, to be removed. Clears the gravity component of the measurement for each accelerometer
  predictedWorldIMUsLinAcc_.clear();
  predictedAccelerometers_.clear();

  // std::cout << std::endl << "y1_: " << std::endl << y << std::endl;
  // std::cout << std::endl << "worldCentroidStateKinematics" << ": " << std::endl << worldCentroidStateKinematics <<
  // std::endl;
  for(VectorIMUConstIterator i = imuSensors_.begin(); i != imuSensors_.end(); ++i)
  {
    if(i->time == k_data_)
    {

      const IMU & imu = *i;
      worldImuKinematics.setToProductNoAlias(
          worldCentroidStateKinematics,
          imu.centroidImuKinematics); // the kinematics of the IMU in the world frame, expressed in the IMU's frame
      // std::cout << std::endl << "imu.centroidImuKinematics" << i->measIndex << ": " << std::endl <<
      // imu.centroidImuKinematics << std::endl; std::cout << std::endl << "worldImuKinematics_" << i->measIndex << ": "
      // << std::endl << worldImuKinematics << std::endl;
      const Matrix3 & worldImuOri = worldImuKinematics.orientation.toMatrix3();

      /// accelerometer
      y.segment<sizeAcceleroSignal>(imu.measIndex).noalias() =
          worldImuKinematics.linAcc() + worldImuOri.transpose() * cst::gravity;

      predictedWorldIMUsLinAcc_.push_back(worldImuKinematics.linAcc());
      predictedAccelerometersGravityComponent_.push_back(worldImuOri.transpose() * cst::gravity);
      predictedAccelerometers_.push_back(y.segment<sizeAcceleroSignal>(imu.measIndex));
      // std::cout << std::endl << "y_: " << std::endl << y.segment<sizeAcceleroSignal>(imu.measIndex) << std::endl;

      /// gyrometer
      y.segment<sizeGyroSignal>(imu.measIndex + sizeAcceleroSignal).noalias() = worldImuKinematics.angVel();
    }
  }
  // std::cout << std::endl << "y2_: " << std::endl << y << std::endl;

  for(VectorContactConstIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet && i->time == k_data_ && i->withRealSensor)
    {
      y.segment<sizeWrench>(i->measIndex) = x.segment<sizeWrench>(contactWrenchIndex(i));
    }
  }
  // std::cout << std::endl << "y3_: " << std::endl << y << std::endl;

  if(absPoseSensor_.time == k)
  {
    y.segment<sizePose>(absPoseSensor_.measIndex) = worldCentroidStateKinematics.toVector(flagsPoseKine);
  }

  if(measurementNoise_ != 0x0)
  {
    measurementNoise_->getNoisy(y);
  }
  // std::cout << std::endl << "y4_: " << std::endl << y << std::endl;
  return y;
}

void KineticsObserver::setInitWorldCentroidStateVector(const Vector & initStateVector)
{
  worldCentroidStateVector_.setZero();
  setStateVector(initStateVector, false);
  oldWorldCentroidStateVector_ = worldCentroidStateVector_;
}

void KineticsObserver::setAllCovariances(const Matrix3 & statePositionInitCovariance,
                                         const Matrix3 & stateOriInitCovariance,
                                         const Matrix3 & stateLinVelInitCovariance,
                                         const Matrix3 & stateAngVelInitCovariance,
                                         const Matrix3 & gyroBiasInitCovariance,
                                         const Matrix6 & unmodeledWrenchInitCovariance,
                                         const Matrix12 & contactInitCovariance,
                                         const Matrix3 & statePositionProcessCovariance,
                                         const Matrix3 & stateOriProcessCovariance,
                                         const Matrix3 & stateLinVelProcessCovariance,
                                         const Matrix3 & stateAngVelProcessCovariance,
                                         const Matrix3 & gyroBiasProcessCovariance,
                                         const Matrix6 & unmodeledWrenchProcessCovariance,
                                         const Matrix12 & contactProcessCovariance,
                                         const Matrix3 & positionSensorCovariance,
                                         const Matrix3 & orientationSensorCoVariance,
                                         const Matrix3 & acceleroSensorCovariance,
                                         const Matrix3 & gyroSensorCovariance,
                                         const Matrix6 & contactSensorCovariance)
{
  statePosInitCovMat_ = statePositionInitCovariance;
  stateOriInitCovMat_ = stateOriInitCovariance;
  stateLinVelInitCovMat_ = stateLinVelInitCovariance;
  stateAngVelInitCovMat_ = stateAngVelInitCovariance;
  gyroBiasInitCovMat_ = gyroBiasInitCovariance;
  unmodeledWrenchInitCovMat_ = unmodeledWrenchInitCovariance;
  contactInitCovMatDefault_ = contactInitCovariance;

  statePosProcessCovMat_ = statePositionProcessCovariance;
  stateOriProcessCovMat_ = stateOriProcessCovariance;
  stateLinVelProcessCovMat_ = stateLinVelProcessCovariance;
  stateAngVelProcessCovMat_ = stateAngVelProcessCovariance;
  gyroBiasProcessCovMat_ = gyroBiasProcessCovariance;
  unmodeledWrenchProcessCovMat_ = unmodeledWrenchProcessCovariance;
  contactProcessCovMatDefault_ = contactProcessCovariance;

  absPoseSensorCovMatDefault_ =
      blockMat6(positionSensorCovariance, Matrix3::Zero(), Matrix3::Zero(), orientationSensorCoVariance);
  acceleroCovMatDefault_ = acceleroSensorCovariance;
  gyroCovMatDefault_ = gyroSensorCovariance;
  contactWrenchSensorCovMatDefault_ = contactSensorCovariance;

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

  // ekf_.setStateCovariance(ekf_.getPmatrixZero());
  ekf_.setQ(ekf_.getQmatrixZero());
  ekf_.setR(ekf_.getRmatrixZero());

  resetStateCovarianceMat();
  resetProcessCovarianceMat();
}

Matrix displayVectorWithIndex(const Vector & vec1, const int & numbIndexes) // to be removed
{
  Eigen::VectorXd indexes(numbIndexes); // to be removed
  // Eigen::VectorXd indexes(57); //to be removed
  for(int ind = 0; ind < indexes.size(); ind++)
  {
    indexes(ind) = ind;
  }
  Matrix C(vec1.rows(), vec1.cols() + indexes.cols());
  C << vec1, indexes;
  return C;
}

int KineticsObserver::getIMUMeasIndexByNum(const int & num) const
{
  return imuSensors_[num].measIndex;
}

int KineticsObserver::getContactMeasIndexByNum(const int & num) const
{
  return contacts_[num].measIndex;
}

bool KineticsObserver::getContactIsSetByNum(const int & num) const
{
  if(num >= contacts_.size() || contacts_.size() == 0)
  {
    return false;
  }
  else
  {
    return contacts_[num].isSet;
  }
}

const std::vector<KineticsObserver::Kinematics> KineticsObserver::getContactPoses() const
{
  std::vector<Kinematics> contactKine;
  for(VectorContactConstIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      const Kinematics & centroidContactKine = i->centroidContactKine;
      const Kinematics & worldContactRefPose = i->worldRefPose;

      Kinematics worldContactKin;
      worldContactKin.setToProductNoAlias(getGlobalCentroidKinematics(), centroidContactKine);
      contactKine.push_back(worldContactRefPose);
      contactKine.push_back(worldContactKin);
    }
    else
    {
      contactKine.emplace_back();
      contactKine.emplace_back();
    }
  }
  return contactKine;
}

const double & KineticsObserver::getMass() const
{
  return mass_;
}

///////////////////////////////////////////////////////////////////////
/// -------------------Tests implementation-------------
///////////////////////////////////////////////////////////////////////

void KineticsObserver::computeLocalAccelerationsForJacobian_(const Vector & x, Vector & acceleration)
{
  Vector3 totalCentroidForce =
      additionalForce_; // initialization of the total force at the centroid with the input additional forces.
  Vector3 totalCentroidTorque = additionalTorque_;

  addUnmodeledAndContactWrench_(x, totalCentroidForce,
                                totalCentroidTorque); // adding the previously estimated contact and unmodeled
                                                      // forces to obtain the total force at the centroid
  LocalKinematics worldCentroidStateKinematics(x, flagsStateKine);

  acceleration.segment<3>(0) =
      (totalCentroidForce / mass_) - worldCentroidStateKinematics.orientation.toMatrix3().transpose() * cst::gravity;

  acceleration.segment<3>(3) =
      I_().inverse()
      * (totalCentroidTorque - Id_() * worldCentroidStateKinematics.angVel() - sigmad_()
         - worldCentroidStateKinematics.angVel().cross(I_() * worldCentroidStateKinematics.angVel() + sigma_()));
}

Matrix KineticsObserver::compareAccelerationsJacobians(const Vector & dx)
{
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

  /* Finite differences Jacobian */
  Matrix accJacobianFD = Matrix::Zero(6, stateTangentSize_);

  Vector accBar = Vector6::Zero();
  Vector accBarIncremented = Vector6::Zero();
  Vector accBarDiff = Vector6::Zero();

  Vector x = ekf_.getCurrentEstimatedState();
  Vector xIncrement = ekf_.getCurrentEstimatedState();

  computeLocalAccelerationsForJacobian_(x, accBar);

  xIncrement.resize(stateTangentSize_);

  for(Index i = 0; i < stateTangentSize_; ++i)
  {
    xIncrement.setZero();
    xIncrement[i] = dx[i];

    stateSum(x, xIncrement, x);

    computeLocalAccelerationsForJacobian_(x, accBarIncremented);

    accBarDiff = accBarIncremented - accBar;

    accBarDiff /= dx[i];

    accJacobianFD.col(i) = accBarDiff;

    x = ekf_.getCurrentEstimatedState();
  }

  /* Analytical jacobian */

  LocalKinematics worldCentroidKinematics(x, flagsStateKine);
  Matrix accJacobianAnalytical = Matrix::Zero(6, stateTangentSize_);
  Matrix3 I_inv = I_().inverse();

  // Jacobians of the linear acceleration
  accJacobianAnalytical.block<3, sizeOriTangent>(0, oriIndexTangent()) =
      -cst::gravityConstant
      * (worldCentroidKinematics.orientation.toMatrix3().transpose() * kine::skewSymmetric(Vector3(0, 0, 1)));
  accJacobianAnalytical.block<3, sizeForceTangent>(0, unmodeledForceIndexTangent()) =
      Matrix::Identity(sizeLinAccTangent, sizeTorqueTangent) / mass_;

  // Jacobians of the angular acceleration
  accJacobianAnalytical.block<3, sizeTorqueTangent>(3, unmodeledTorqueIndexTangent()) = I_inv;
  accJacobianAnalytical.block<3, sizeAngVelTangent>(3, angVelIndexTangent()) =
      I_inv
      * (kine::skewSymmetric(I_() * worldCentroidKinematics.angVel()) - Id_()
         - kine::skewSymmetric(worldCentroidKinematics.angVel()) * I_() + kine::skewSymmetric(sigma_()));

  // Jacobians with respect to the contacts
  for(VectorContactConstIterator i = contacts_.begin(); i != contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      // Jacobian of the linar acceleration with respect to the contact force
      accJacobianAnalytical.block<3, sizeForceTangent>(0, contactForceIndexTangent(i)) =
          (1 / mass_) * i->centroidContactKine.orientation.toMatrix3();
      // Jacobian of the angular acceleration with respect to the contact force
      accJacobianAnalytical.block<3, sizeTorqueTangent>(3, contactForceIndexTangent(i)) =
          (I_inv * kine::skewSymmetric(i->centroidContactKine.position()))
          * (i->centroidContactKine.orientation).toMatrix3();
      // Jacobian of the angular acceleration with respect to the contact torque
      accJacobianAnalytical.block<3, sizeTorqueTangent>(3, contactTorqueIndexTangent(i)) =
          I_inv * i->centroidContactKine.orientation.toMatrix3();
    }
  }

  std::cout << std::endl << "Analytical : " << std::endl << accJacobianAnalytical.format(CleanFmt) << std::endl;
  std::cout << std::endl << "FD : " << std::endl << accJacobianFD.format(CleanFmt) << std::endl;

  /* Comparison */

  for(int i = 0; i < accJacobianAnalytical.rows(); i++)
  {
    for(int j = 0; j < accJacobianAnalytical.cols(); j++)
    {
      if(abs(accJacobianAnalytical(i, j) - accJacobianFD(i, j))
                 / std::max(abs(accJacobianAnalytical(i, j)), abs(accJacobianFD(i, j))) * 100
             > 20
         && abs(accJacobianAnalytical(i, j) - accJacobianFD(i, j)) != 0)
      {
        std::cout << std::endl
                  << "\033[1;31m"
                  << "error indexes: " << std::endl
                  << "(" << i << "," << j << "):  Analytic : " << accJacobianAnalytical(i, j)
                  << "    FD : " << accJacobianFD(i, j) << "    Relative error : "
                  << abs(accJacobianAnalytical(i, j) - accJacobianFD(i, j))
                         / std::max(abs(accJacobianAnalytical(i, j)), abs(accJacobianFD(i, j))) * 100
                  << " % "
                  << "\033[0m\n"
                  << std::endl;
      }
    }
  }

  return accJacobianAnalytical - accJacobianFD;
}

Matrix KineticsObserver::compareAnalyticalAndFDJacobians(double threshold)
{
  const Vector & dx = worldCentroidStateVectorDx_;
  Matrix A_analytic = computeAMatrix_();
  Matrix A_FD = ekf_.getAMatrixFD(dx);

  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  std::cout << std::endl << "Analytical : " << std::endl << A_analytic.format(CleanFmt) << std::endl;
  std::cout << std::endl << "FD : " << std::endl << A_FD.format(CleanFmt) << std::endl;

  bool stopIT = false;

  std::cout << std::endl
            << "\033[1;34m"
            << "New iteration"
            << "\033[0m\n"
            << std::endl;

  for(int i = 0; i < A_analytic.rows(); i++)
  {
    for(int j = 0; j < A_analytic.cols(); j++)
    {
      if(abs(A_analytic(i, j) - A_FD(i, j)) / std::max(abs(A_analytic(i, j)), abs(A_FD(i, j))) * 100 > threshold
         && abs(A_analytic(i, j) - A_FD(i, j)) != 0)
      {
        std::cout << std::endl
                  << "\033[1;31m"
                  << "error indexes: " << std::endl
                  << "(" << i << "," << j << "):  Analytic : " << A_analytic(i, j) << "    FD : " << A_FD(i, j)
                  << "    Relative error : "
                  << abs(A_analytic(i, j) - A_FD(i, j)) / std::max(abs(A_analytic(i, j)), abs(A_FD(i, j))) * 100
                  << " % "
                  << "\033[0m\n"
                  << std::endl;
        stopIT = true;
      }
      else
      {
        /*
        std::cout << std::endl
                  << "good indexes: " << std::endl
                  << "(" << i << "," << j << "):  Analytic : " << A_analytic(i, j) << "    FD : " << A_FD(i, j)
                  << "    Relative error : " << abs(A_analytic(i, j) - A_FD(i, j)) / std::max(abs(A_analytic(i,
        j)), abs(A_FD(i, j))) * 100
                  << " % " << std::endl;

                  */
      }
    }
  }
  return A_analytic - A_FD;
}

} // namespace stateObservation
