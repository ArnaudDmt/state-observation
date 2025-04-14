#include <Eigen/Eigenvalues>
#include <iostream>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>

using namespace stateObservation;
using namespace kine;

double dt_ = 0.005;

Vector3 processPos_1_ = Vector3(3.0, 0.4, 1.8) * 1e-13;
Vector3 processPos_2_ = Vector3(0.2, 4.3, 1.9) * 1e-13;
Vector3 processPos_3_ = Vector3(2.5, 2.7, 0.9) * 1e-13;

double lin_stiffness_ = (double)rand() / RAND_MAX * 1e4;
double lin_damping_ = (double)rand() / RAND_MAX * 5 * 1e1;
double ang_stiffness_ = (double)rand() / RAND_MAX * 1e3;
double ang_damping_ = (double)rand() / RAND_MAX * 1e1;

Matrix3 K1_ = lin_stiffness_ * Matrix3::Identity();
Matrix3 K2_ = lin_damping_ * Matrix3::Identity();
Matrix3 K3_ = ang_stiffness_ * Matrix3::Identity();
Matrix3 K4_ = ang_damping_ * Matrix3::Identity();

double lin_stiffness_2_ = (double)rand() / RAND_MAX * 1e4;
double lin_damping_2_ = (double)rand() / RAND_MAX * 5 * 1e1;
double ang_stiffness_2_ = (double)rand() / RAND_MAX * 1e3;
double ang_damping_2_ = (double)rand() / RAND_MAX * 1e1;

Matrix3 K1_2_ = lin_stiffness_2_ * Matrix3::Identity();
Matrix3 K2_2_ = lin_damping_2_ * Matrix3::Identity();
Matrix3 K3_2_ = ang_stiffness_2_ * Matrix3::Identity();
Matrix3 K4_2_ = ang_damping_2_ * Matrix3::Identity();

Vector3 gyroBias1_ = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

Vector3 com_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 com_d_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 com_dd_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 position_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation ori_;
Vector3 linvel_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 angvel_ = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

Vector3 extForces_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 extTorques_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 centroidIMUPos1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidIMUOri1_;
Vector3 centroidIMULinVel1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidIMUAngVel1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidIMULinAcc1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidIMUAngAcc1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 worldContactPos1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation worldContactOri1_;
Vector3 centroidContactPos1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidContactOri1_;
Vector3 centroidContactLinVel1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidContactAngVel1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 contactForces1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() * 100;
Vector3 contactTorques1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() * 10;

Vector3 worldContactPos2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation worldContactOri2_;
Vector3 centroidContactPos2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidContactOri2_;
Vector3 centroidContactLinVel2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidContactAngVel2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 contactForces2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() * 100;
Vector3 contactTorques2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() * 10;

Matrix3 inertiaMatrix_ = tools::ProbabilityLawSimulation::getUniformMatrix<Matrix3>();
Matrix3 inertiaMatrix_d_ = tools::ProbabilityLawSimulation::getGaussianMatrix<Matrix3>();
Vector3 angularMomentum_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 angularMomentum_d_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Eigen::IOFormat CleanFmt_(4, 0, ", ", "\n", "[", "]");

int testKineticsObserverCodeAccessor(int errorcode)
{
  KineticsObserver ko_1_(1, 1);

  double error = 0;
  double dt = 0.001;

  double mass = 100;
  KineticsObserver o(4, 2);

  Vector x0(o.getStateSize());
  x0.setZero();
  Vector xf(x0);
  Vector xs(x0);

  o.setSamplingTime(dt);

  LocalKinematics stateKine;

  stateKine.position.set() << 0.1, 0, 0.7;
  stateKine.orientation = Vector3(0, 0, 0);
  stateKine.linVel.set().setZero();
  stateKine.angVel.set().setZero();

  o.setStateVector(x0);

  o.setWorldCentroidStateKinematics(stateKine);
  o.setGyroBias(Vector3(1, 2, 3));

  Vector6 wrench;
  wrench << 4, 5, 6, 7, 8, 9;

  o.setStateUnmodeledWrench(wrench);

  Vector x = o.getCurrentStateVector();
  stateObservation::TimeIndex index = o.getStateVectorTimeIndex();

  Kinematics contactKine;
  contactKine.position.set() << 0, 0.1, 0;
  contactKine.orientation.setZeroRotation();

  o.addContact(contactKine, 0);

  Matrix3 linStiffness, angStiffness, linDamping, angDamping;
  linStiffness.setZero();
  linStiffness.diagonal().setConstant(50000);

  angStiffness.setZero();
  angStiffness.diagonal().setConstant(400);

  linDamping.setZero();
  linDamping.diagonal().setConstant(500);

  angDamping.setZero();
  angDamping.diagonal().setConstant(20);

  contactKine.position.set() << 0, -0.1, 0;
  o.addContact(contactKine, 3, linStiffness, linDamping, angStiffness, angDamping);

  Matrix12 initialCov, processCov;

  initialCov.setZero();
  initialCov.diagonal().setConstant(0.01);
  processCov.setZero();
  processCov.diagonal().setConstant(0.0001);

  contactKine.position.set() << 1, 0.1, 0;
  Index i = o.addContact(contactKine, initialCov, processCov);

  (void)i; /// avoid warning in release mode
  assert(i == 1);

  contactKine.position.set() << 1, -0.1, 0;
  o.addContact(contactKine, initialCov, processCov, 2, linDamping, linStiffness, angStiffness, angDamping);

  // std::cout << index << " " << x.transpose() << std::endl;

  o.update();

  LocalKinematics k = o.getLocalCentroidKinematics();

  // std::cout << k;

  // std::cout << o.kineIndex() << " " << o.posIndex() << " " << o.oriIndex() << " " << o.linVelIndex() << " "
  //           << o.angVelIndex() << " " << o.gyroBiasIndex(0) << " " << o.gyroBiasIndex(1) << " "
  //           << o.unmodeledWrenchIndex() << " " << o.unmodeledForceIndex() << " " << o.unmodeledTorqueIndex() << " "
  //           << o.contactsIndex() << " " << o.contactIndex(0) << " " << o.contactKineIndex(0) << " "
  //           << o.contactPosIndex(0) << " " << o.contactOriIndex(0) << " " << o.contactForceIndex(0) << " "
  //           << o.contactTorqueIndex(0) << " " << o.contactWrenchIndex(0) << " " <<

  //     o.contactIndex(1) << " " << o.contactKineIndex(1) << " " << o.contactPosIndex(1) << " " << o.contactOriIndex(1)
  //           << " " << o.contactForceIndex(1) << " " << o.contactTorqueIndex(1) << " " << o.contactWrenchIndex(1) << "
  //           "
  //           <<

  //     o.contactIndex(2) << " " << o.contactKineIndex(2) << " " << o.contactPosIndex(2) << " " << o.contactOriIndex(2)
  //           << " " << o.contactForceIndex(2) << " " << o.contactTorqueIndex(2) << " " << o.contactWrenchIndex(2) << "
  //           "
  //           <<

  //     o.contactIndex(3) << " " << o.contactKineIndex(3) << " " << o.contactPosIndex(3) << " " << o.contactOriIndex(3)
  //           << " " << o.contactForceIndex(3) << " " << o.contactTorqueIndex(3) << " " << o.contactWrenchIndex(3) << "
  //           "
  //           << std::endl;

  // std::cout << o.kineIndexTangent() << " " << o.posIndexTangent() << " " << o.oriIndexTangent() << " "
  //           << o.linVelIndexTangent() << " " << o.angVelIndexTangent() << " " << o.gyroBiasIndexTangent(0) << " "
  //           << o.gyroBiasIndexTangent(1) << " " << o.unmodeledWrenchIndexTangent() << " "
  //           << o.unmodeledForceIndexTangent() << " " << o.unmodeledTorqueIndexTangent() << " "
  //           << o.contactsIndexTangent() << " " <<

  //     o.contactIndexTangent(0) << " " << o.contactKineIndexTangent(0) << " " << o.contactPosIndexTangent(0) << " "
  //           << o.contactOriIndexTangent(0) << " " << o.contactForceIndexTangent(0) << " "
  //           << o.contactTorqueIndexTangent(0) << " " << o.contactWrenchIndexTangent(0) << " " <<

  //     o.contactIndexTangent(1) << " " << o.contactKineIndexTangent(1) << " " << o.contactPosIndexTangent(1) << " "
  //           << o.contactOriIndexTangent(1) << " " << o.contactForceIndexTangent(1) << " "
  //           << o.contactTorqueIndexTangent(1) << " " << o.contactWrenchIndexTangent(1) << " " <<

  //     o.contactIndexTangent(2) << " " << o.contactKineIndexTangent(2) << " " << o.contactPosIndexTangent(2) << " "
  //           << o.contactOriIndexTangent(2) << " " << o.contactForceIndexTangent(2) << " "
  //           << o.contactTorqueIndexTangent(2) << " " << o.contactWrenchIndexTangent(2) << " " <<

  //     o.contactIndexTangent(3) << " " << o.contactKineIndexTangent(3) << " " << o.contactPosIndexTangent(3) << " "
  //           << o.contactOriIndexTangent(3) << " " << o.contactForceIndexTangent(3) << " "
  //           << o.contactTorqueIndexTangent(3) << " " << o.contactWrenchIndexTangent(3) << " " << std::endl;

  o.setWithUnmodeledWrench(true);
  o.setWithAccelerationEstimation(true);
  o.setWithGyroBias(true);

  Matrix3 acceleroCov, gyroCov;

  acceleroCov = Matrix3::Identity() * 1e-4;
  gyroCov = Matrix3::Identity() * 1e-8;

  o.setIMUDefaultCovarianceMatrix(acceleroCov, gyroCov);

  Matrix6 wrenchCov;

  wrenchCov << Matrix3::Identity() * 1e-0, Matrix3::Zero(), Matrix3::Zero(), Matrix3::Identity() * 1e-4;

  o.setContactWrenchSensorDefaultCovarianceMatrix(wrenchCov);

  o.setMass(mass);

  Vector state1 = o.getEKF().stateVectorRandom();
  Vector state2 = o.getEKF().stateVectorRandom();
  Vector statediff;
  o.stateDifference(state1, state2, statediff);
  Vector state3;
  o.stateSum(state2, statediff, state3);

  // std::cout << state1.transpose() << std::endl;
  // std::cout << state3.transpose() << std::endl;

  Matrix statecomp(state1.size(), 2);

  statecomp << state1, state3;

  // std::cout << statecomp << std::endl;

  // std::cout << "Sum error" << (error = o.stateDifference(state1, state3).norm()) << std::endl;

  state2 = o.getEKF().stateVectorRandom();
  statediff = o.getEKF().stateTangentVectorRandom();

  Vector statediff_bis;
  o.stateSum(state2, statediff, state1);
  o.stateDifference(state1, state2, statediff_bis);

  // std::cout << statediff.transpose() << std::endl;
  // std::cout << statediff_bis.transpose() << std::endl;

  Matrix statecompdiff(statediff.size(), 2);

  statecompdiff << statediff, statediff_bis;

  // std::cout << statecompdiff << std::endl;

  std::cout << "DIff error" << (error += (statediff - statediff_bis).norm()) << std::endl;

  if(error > 1e-8)
  {
    return errorcode;
  }

  o.clearContacts();

  return 0;
}

int testContactRestPoseCovariance_1contact(int errorcode)
{
  KineticsObserver ko_1_(1, 1);

  Vector stateVector_;

  ko_1_.setSamplingTime(dt_);
  ko_1_.setWithUnmodeledWrench(true);
  ko_1_.setWithGyroBias(false);

  inertiaMatrix_ = inertiaMatrix_ * inertiaMatrix_.transpose();
  inertiaMatrix_d_ = inertiaMatrix_d_ * inertiaMatrix_d_.transpose();

  ori_.setRandom();

  /* Kinetics Observer 1 initialization */

  centroidIMUOri1_.setRandom();
  Kinematics centroidIMUPose1_;
  centroidIMUPose1_.position = centroidIMUPos1_;
  centroidIMUPose1_.orientation = centroidIMUOri1_;
  centroidIMUPose1_.linVel = centroidIMULinVel1_;
  centroidIMUPose1_.angVel = centroidIMUAngVel1_;
  centroidIMUPose1_.linAcc = centroidIMULinAcc1_;
  centroidIMUPose1_.angAcc = centroidIMUAngAcc1_;

  worldContactOri1_.setRandom();
  Kinematics worldContactPose1_;
  worldContactPose1_.position = worldContactPos1_;
  worldContactPose1_.orientation = worldContactOri1_;
  centroidContactOri1_.setRandom();
  Kinematics centroidContactPose1_;
  centroidContactPose1_.position = centroidContactPos1_;
  centroidContactPose1_.orientation = centroidContactOri1_;
  centroidContactPose1_.linVel = centroidContactLinVel1_;
  centroidContactPose1_.angVel = centroidContactAngVel1_;

  ko_1_.setCenterOfMass(com_, com_d_, com_dd_);
  ko_1_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose1_, 0);

  ko_1_.setCoMAngularMomentum(angularMomentum_, angularMomentum_d_);
  ko_1_.setCoMInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_1_.addContact(worldContactPose1_, 0, K1_, K2_, K3_, K4_);
  ko_1_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose1_, 0);

  stateVector_.resize(position_.size() + 4 + linvel_.size() + angvel_.size() + gyroBias1_.size() + extForces_.size()
                      + extTorques_.size()
                      + 1
                            * (worldContactPos1_.size() + worldContactOri1_.toVector4().size() + contactForces1_.size()
                               + contactTorques1_.size()));
  stateVector_ << position_, ori_.toVector4(), linvel_, angvel_, gyroBias1_, extForces_, extTorques_, worldContactPos1_,
      worldContactOri1_.toVector4(), contactForces1_, contactTorques1_;

  ko_1_.setInitWorldCentroidStateVector(stateVector_);

  ko_1_.update();

  Eigen::MatrixXd contact1_Q_temp = Eigen::MatrixXd::Zero(3, 3);
  contact1_Q_temp.block(0, 0, 3, 3) =
      ko_1_.getEKF().getQ().block(ko_1_.contactPosIndexTangent(0), ko_1_.contactPosIndexTangent(0), 3, 3);

  // std::cout << std::endl << "Contact1: " << std::endl << contact1_Q_temp.format(CleanFmt_) << std::endl;

  // std::cout << std::endl
  //           << "################################################### New iter "
  //              "###################################################"
  //           << std::endl;

  ko_1_.setCenterOfMass(com_, com_d_, com_dd_);
  ko_1_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose1_, 0);

  ko_1_.setCoMAngularMomentum(angularMomentum_, angularMomentum_d_);
  ko_1_.setCoMInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_1_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose1_, 0);

  Matrix3 processPos = processPos_1_.asDiagonal();

  ko_1_.setContactProcessCovMat(0, &processPos);

  ko_1_.update();

  Eigen::MatrixXd contact1_Q = Eigen::MatrixXd::Zero(3, 3);
  contact1_Q.block(0, 0, 3, 3) =
      ko_1_.getEKF().getQ().block(ko_1_.contactPosIndexTangent(0), ko_1_.contactPosIndexTangent(0), 3, 3);

  // std::cout << std::endl << "Contact1: " << std::endl << contact1_Q.format(CleanFmt_) << std::endl;

  return 0;
}

int testContactRestPoseCovariance_2contacts(int errorcode)
{
  KineticsObserver ko_2_(2, 1);

  Vector stateVector_;

  ko_2_.setSamplingTime(dt_);
  ko_2_.setWithUnmodeledWrench(true);
  ko_2_.setWithGyroBias(false);

  inertiaMatrix_ = inertiaMatrix_ * inertiaMatrix_.transpose();
  inertiaMatrix_d_ = inertiaMatrix_d_ * inertiaMatrix_d_.transpose();

  ori_.setRandom();

  /* Kinetics Observer 1 initialization */

  centroidIMUOri1_.setRandom();
  Kinematics centroidIMUPose1_;
  centroidIMUPose1_.position = centroidIMUPos1_;
  centroidIMUPose1_.orientation = centroidIMUOri1_;
  centroidIMUPose1_.linVel = centroidIMULinVel1_;
  centroidIMUPose1_.angVel = centroidIMUAngVel1_;
  centroidIMUPose1_.linAcc = centroidIMULinAcc1_;
  centroidIMUPose1_.angAcc = centroidIMUAngAcc1_;

  worldContactOri1_.setRandom();
  Kinematics worldContactPose1_;
  worldContactPose1_.position = worldContactPos1_;
  worldContactPose1_.orientation = worldContactOri1_;
  centroidContactOri1_.setRandom();
  Kinematics centroidContactPose1_;
  centroidContactPose1_.position = centroidContactPos1_;
  centroidContactPose1_.orientation = centroidContactOri1_;
  centroidContactPose1_.linVel = centroidContactLinVel1_;
  centroidContactPose1_.angVel = centroidContactAngVel1_;

  Kinematics worldContactPose2_ = worldContactPose1_;
  Kinematics centroidContactPose2_ = centroidContactPose1_;

  ko_2_.setCenterOfMass(com_, com_d_, com_dd_);
  ko_2_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose1_, 0);

  ko_2_.setCoMAngularMomentum(angularMomentum_, angularMomentum_d_);
  ko_2_.setCoMInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_2_.addContact(worldContactPose1_, 0, K1_, K2_, K3_, K4_);
  ko_2_.addContact(worldContactPose2_, 1, K1_, K2_, K3_, K4_);
  ko_2_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose1_, 0);
  ko_2_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose2_, 1);

  stateVector_.resize(position_.size() + 4 + linvel_.size() + angvel_.size() + gyroBias1_.size() + extForces_.size()
                      + extTorques_.size()
                      + 2
                            * (worldContactPos1_.size() + worldContactOri1_.toVector4().size() + contactForces1_.size()
                               + contactTorques1_.size()));
  stateVector_ << position_, ori_.toVector4(), linvel_, angvel_, gyroBias1_, extForces_, extTorques_, worldContactPos1_,
      worldContactOri1_.toVector4(), contactForces1_, contactTorques1_, worldContactPos1_,
      worldContactOri1_.toVector4(), contactForces2_, contactTorques2_;

  ko_2_.setInitWorldCentroidStateVector(stateVector_);

  if(!(ko_2_.getEKF().getQ().eigenvalues().real().array() > -1e-8).all())
  {
    std::cout << std::endl
              << "1: The process covariance matrix is not positive definite at the beginning!" << std::endl;
    std::cout << std::endl << "eigen values: " << ko_2_.getEKF().getQ().eigenvalues().real().transpose() << std::endl;
    return errorcode;
  }

  if(!(ko_2_.getStateCovarianceMat().eigenvalues().real().array() > -1e-8).all())
  {
    std::cout << std::endl << "1: The state covariance matrix is not positive definite at the beginning!" << std::endl;
    std::cout << std::endl
              << "eigen values: " << ko_2_.getStateCovarianceMat().eigenvalues().real().transpose() << std::endl;
    return errorcode;
  }

  ko_2_.update();

  Eigen::MatrixXd contact1_Q_temp_pos = Eigen::MatrixXd::Zero(3, 6);
  contact1_Q_temp_pos.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactPosIndexTangent(0), ko_2_.contactPosIndexTangent(0), 3, 3);
  contact1_Q_temp_pos.block(0, 3, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactPosIndexTangent(0), ko_2_.contactPosIndexTangent(1), 3, 3);

  Eigen::MatrixXd contact1_Q_temp_ori = Eigen::MatrixXd::Zero(3, 6);
  contact1_Q_temp_ori.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactOriIndexTangent(0), ko_2_.contactOriIndexTangent(0), 3, 3);
  contact1_Q_temp_ori.block(0, 3, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactOriIndexTangent(0), ko_2_.contactOriIndexTangent(1), 3, 3);

  Eigen::MatrixXd contact2_Q_temp_pos = Eigen::MatrixXd::Zero(3, 6);
  contact2_Q_temp_pos.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactPosIndexTangent(1), ko_2_.contactPosIndexTangent(0), 3, 3);
  contact2_Q_temp_pos.block(0, 3, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactPosIndexTangent(1), ko_2_.contactPosIndexTangent(1), 3, 3);

  Eigen::MatrixXd contact2_Q_temp_ori = Eigen::MatrixXd::Zero(3, 6);
  contact2_Q_temp_ori.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactOriIndexTangent(1), ko_2_.contactOriIndexTangent(0), 3, 3);
  contact2_Q_temp_ori.block(0, 3, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactOriIndexTangent(1), ko_2_.contactOriIndexTangent(1), 3, 3);

  //   std::cout << std::endl << "Contact1 pos: " << std::endl << contact1_Q_temp_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2 pos: " << std::endl << contact2_Q_temp_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact1 ori: " << std::endl << contact1_Q_temp_ori.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2 ori: " << std::endl << contact2_Q_temp_ori.format(CleanFmt_) << std::endl;

  if(!(ko_2_.getEKF().getQ().eigenvalues().real().array() > -1e-8).all())
  {
    std::cout << std::endl << "2: The process covariance matrix is no longer positive definite!" << std::endl;
    std::cout << std::endl << "eigen values: " << ko_2_.getEKF().getQ().eigenvalues().real().transpose() << std::endl;
    return errorcode;
  }

  if(!(ko_2_.getStateCovarianceMat().eigenvalues().real().array() > -1e-8).all())
  {
    std::cout << std::endl << "2: The state covariance matrix is no longer positive definite!" << std::endl;
    std::cout << std::endl
              << "eigen values: " << ko_2_.getStateCovarianceMat().eigenvalues().real().transpose() << std::endl;
    return errorcode;
  }

  std::cout << std::endl
            << "################################################### New iter "
               "###################################################"
            << std::endl;

  ko_2_.setCenterOfMass(com_, com_d_, com_dd_);
  ko_2_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose1_, 0);

  ko_2_.setCoMAngularMomentum(angularMomentum_, angularMomentum_d_);
  ko_2_.setCoMInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_2_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose1_, 0);
  ko_2_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose2_, 1);

  Matrix3 processPos1 = processPos_1_.asDiagonal();
  Matrix3 processPos2 = processPos_2_.asDiagonal();

  ko_2_.setContactProcessCovMat(0, &processPos1, &processPos1);
  ko_2_.setContactProcessCovMat(1, &processPos2, &processPos2);

  ko_2_.update();

  Eigen::MatrixXd contact1_Q_pos = Eigen::MatrixXd::Zero(3, 6);
  contact1_Q_pos.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactPosIndexTangent(0), ko_2_.contactPosIndexTangent(0), 3, 3);
  contact1_Q_pos.block(0, 3, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactPosIndexTangent(0), ko_2_.contactPosIndexTangent(1), 3, 3);

  Eigen::MatrixXd contact1_Q_ori = Eigen::MatrixXd::Zero(3, 6);
  contact1_Q_ori.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactOriIndexTangent(0), ko_2_.contactOriIndexTangent(0), 3, 3);
  contact1_Q_ori.block(0, 3, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactOriIndexTangent(0), ko_2_.contactOriIndexTangent(1), 3, 3);

  Eigen::MatrixXd contact2_Q_pos = Eigen::MatrixXd::Zero(3, 6);
  contact2_Q_pos.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactPosIndexTangent(1), ko_2_.contactPosIndexTangent(0), 3, 3);
  contact2_Q_pos.block(0, 3, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactPosIndexTangent(1), ko_2_.contactPosIndexTangent(1), 3, 3);

  Eigen::MatrixXd contact2_Q_ori = Eigen::MatrixXd::Zero(3, 6);
  contact2_Q_ori.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactOriIndexTangent(1), ko_2_.contactOriIndexTangent(0), 3, 3);
  contact2_Q_ori.block(0, 3, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactOriIndexTangent(1), ko_2_.contactOriIndexTangent(1), 3, 3);

  Eigen::MatrixXd contacts_Q_pos = Eigen::MatrixXd::Zero(6, 6);
  contacts_Q_pos.block<3, 6>(0, 0) = contact1_Q_pos;
  contacts_Q_pos.block<3, 6>(3, 0) = contact2_Q_pos;

  Eigen::MatrixXd contacts_Q_pos_analytic = Eigen::MatrixXd::Zero(6, 6);
  contacts_Q_pos_analytic(0, 0) = 0.25 * processPos1(0, 0) + 0.25 * processPos2(0, 0);
  contacts_Q_pos_analytic(0, 3) = -0.25 * processPos1(0, 0) - 0.25 * processPos2(0, 0);
  contacts_Q_pos_analytic(1, 1) = 0.25 * processPos1(1, 1) + 0.25 * processPos2(1, 1);
  contacts_Q_pos_analytic(1, 4) = -0.25 * processPos1(1, 1) - 0.25 * processPos2(1, 1);
  contacts_Q_pos_analytic(2, 2) = 0.25 * processPos1(2, 2) + 0.25 * processPos2(2, 2);
  contacts_Q_pos_analytic(2, 5) = -0.25 * processPos1(2, 2) - 0.25 * processPos2(2, 2);
  contacts_Q_pos_analytic(3, 0) = -0.25 * processPos1(0, 0) - 0.25 * processPos2(0, 0);
  contacts_Q_pos_analytic(3, 3) = 0.25 * processPos1(0, 0) + 0.25 * processPos2(0, 0);
  contacts_Q_pos_analytic(4, 1) = -0.25 * processPos1(1, 1) - 0.25 * processPos2(1, 1);
  contacts_Q_pos_analytic(4, 4) = 0.25 * processPos1(1, 1) + 0.25 * processPos2(1, 1);
  contacts_Q_pos_analytic(5, 2) = -0.25 * processPos1(2, 2) - 0.25 * processPos2(2, 2);
  contacts_Q_pos_analytic(5, 5) = 0.25 * processPos1(2, 2) + 0.25 * processPos2(2, 2);

  Eigen::MatrixXd contacts_Q_ori = Eigen::MatrixXd::Zero(6, 6);
  contacts_Q_ori.block<3, 6>(0, 0) = contact1_Q_ori;
  contacts_Q_ori.block<3, 6>(3, 0) = contact2_Q_ori;

  Eigen::MatrixXd contacts_Q_ori_analytic = Eigen::MatrixXd::Zero(6, 6);
  // contacts_Q_ori_analytic(0, 0) = processPos1(0, 0);
  // contacts_Q_ori_analytic(1, 1) = processPos1(1, 1);
  contacts_Q_ori_analytic(2, 2) = 0.25 * processPos1(2, 2) + 0.25 * processPos2(2, 2);
  contacts_Q_ori_analytic(2, 5) = -0.25 * processPos1(2, 2) - 0.25 * processPos2(2, 2);
  // contacts_Q_ori_analytic(3, 3) = processPos2(0, 0);
  // contacts_Q_ori_analytic(4, 4) = processPos2(1, 1);
  contacts_Q_ori_analytic(5, 2) = -0.25 * processPos1(2, 2) - 0.25 * processPos2(2, 2);
  contacts_Q_ori_analytic(5, 5) = 0.25 * processPos1(2, 2) + 0.25 * processPos2(2, 2);

  //   std::cout << std::endl << "Contact1 pos: " << std::endl << contact1_Q_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2 pos: " << std::endl << contact2_Q_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact1 ori: " << std::endl << contact1_Q_ori.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2 ori: " << std::endl << contact2_Q_ori.format(CleanFmt_) << std::endl;

  if((contacts_Q_pos - contacts_Q_pos_analytic).norm() > 1e-16)
  {
    std::cout << std::endl
              << "Error, the numerical matrix for the rest position process doesn't match the analytical one."
              << std::endl;
    std::cout << std::endl << "contacts_Q_pos: " << std::endl << contacts_Q_pos.format(CleanFmt_) << std::endl;
    std::cout << std::endl
              << "contacts_Q_pos_analytic: " << std::endl
              << contacts_Q_pos_analytic.format(CleanFmt_) << std::endl;

    return errorcode;
  }

  if((contacts_Q_ori - contacts_Q_ori_analytic).norm() > 1e-16)
  {
    std::cout << std::endl
              << "Error, the numerical matrix for the orientation doesn't match the analytical one." << std::endl;
    std::cout << std::endl << "contacts_Q_ori: " << std::endl << contacts_Q_ori.format(CleanFmt_) << std::endl;
    std::cout << std::endl
              << "contacts_Q_ori_analytic: " << std::endl
              << contacts_Q_ori_analytic.format(CleanFmt_) << std::endl;

    return errorcode;
  }

  if(!(ko_2_.getEKF().getQ().eigenvalues().real().array() > -1e-8).all())
  {
    std::cout << std::endl << "3: The process covariance matrix is no longer positive definite!" << std::endl;
    std::cout << std::endl << "eigen values: " << ko_2_.getEKF().getQ().eigenvalues().real().transpose() << std::endl;
    return errorcode;
  }

  if(!(ko_2_.getStateCovarianceMat().eigenvalues().real().array() > -1e-8).all())
  {
    std::cout << std::endl << "3: The state covariance matrix is no longer positive definite!" << std::endl;
    std::cout << std::endl
              << "eigen values: " << ko_2_.getStateCovarianceMat().eigenvalues().real().transpose() << std::endl;
    return errorcode;
  }

  std::cout << std::endl
            << "################################################### New iter: we remove the contact 2 "
               "###################################################"
            << std::endl;

  /* We remove the last contact to verify that we will have the same result that when we had two contacts */

  // we save this index as it cannot be accessed once the contact is removed
  Index contact2PosIndex = ko_2_.contactPosIndexTangent(1);
  Index contact2OriIndex = ko_2_.contactOriIndexTangent(1);

  ko_2_.setCenterOfMass(com_, com_d_, com_dd_);
  ko_2_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose1_, 0);

  ko_2_.setCoMAngularMomentum(angularMomentum_, angularMomentum_d_);
  ko_2_.setCoMInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_2_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose1_, 0);
  ko_2_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose2_, 1);

  // we remove the contact 2
  ko_2_.removeContact(1);
  ko_2_.update();

  contact1_Q_pos.setZero();
  contact1_Q_pos.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactPosIndexTangent(0), ko_2_.contactPosIndexTangent(0), 3, 3);
  contact1_Q_pos.block(0, 3, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactPosIndexTangent(0), contact2PosIndex, 3, 3);

  contact1_Q_ori.setZero();
  contact1_Q_ori.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactOriIndexTangent(0), ko_2_.contactOriIndexTangent(0), 3, 3);
  contact1_Q_ori.block(0, 3, 3, 3) =
      ko_2_.getEKF().getQ().block(ko_2_.contactOriIndexTangent(0), contact2OriIndex, 3, 3);

  contact2_Q_pos.setZero();
  contact2_Q_pos.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(contact2PosIndex, ko_2_.contactPosIndexTangent(0), 3, 3);
  contact2_Q_pos.block(0, 3, 3, 3) = ko_2_.getEKF().getQ().block(contact2PosIndex, contact2PosIndex, 3, 3);

  contact2_Q_ori.setZero();
  contact2_Q_ori.block(0, 0, 3, 3) =
      ko_2_.getEKF().getQ().block(contact2OriIndex, ko_2_.contactOriIndexTangent(0), 3, 3);
  contact2_Q_ori.block(0, 3, 3, 3) = ko_2_.getEKF().getQ().block(contact2OriIndex, contact2OriIndex, 3, 3);

  //   std::cout << std::endl << "Contact1: " << std::endl << contact1_Q_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2: " << std::endl << contact2_Q_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact1: " << std::endl << contact1_Q_ori.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2: " << std::endl << contact2_Q_ori.format(CleanFmt_) << std::endl;

  // This process must be zero as there is only one contact left
  Eigen::MatrixXd contact1_Q_pos_analytic = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd contact1_Q_ori_analytic = Eigen::MatrixXd::Zero(3, 3);

  if((contact1_Q_pos.block<3, 3>(0, 0) - contact1_Q_pos_analytic).norm() > 1e-16)
  {
    std::cout << std::endl
              << "Error, the numerical matrix for the rest position process doesn't match the analytical one."
              << std::endl;
    std::cout << std::endl << "contact1_Q_pos: " << std::endl << contact1_Q_pos.format(CleanFmt_) << std::endl;
    std::cout << std::endl
              << "contact1_Q_pos_analytic: " << std::endl
              << contact1_Q_pos_analytic.format(CleanFmt_) << std::endl;

    return errorcode;
  }

  if((contact1_Q_ori.block<3, 3>(0, 0) - contact1_Q_ori_analytic).norm() > 1e-16)
  {
    std::cout << std::endl
              << "Error, the numerical matrix for the orientation doesn't match the analytical one." << std::endl;
    std::cout << std::endl << "contact1_Q_ori: " << std::endl << contact1_Q_ori.format(CleanFmt_) << std::endl;
    std::cout << std::endl
              << "contact1_Q_ori_analytic: " << std::endl
              << contact1_Q_ori_analytic.format(CleanFmt_) << std::endl;

    return errorcode;
  }

  if(!(ko_2_.getStateCovarianceMat().eigenvalues().real().array() > -1e-8).all())
  {
    std::cout << std::endl << "4: The state covariance matrix is no longer positive definite" << std::endl;
    std::cout << std::endl
              << "eigen values: " << ko_2_.getStateCovarianceMat().eigenvalues().real().transpose() << std::endl;
    return errorcode;
  }
  return 0;
}

int testContactRestPoseCovariance_3contacts(int errorcode)
{
  KineticsObserver ko_3_(3, 1);

  Vector stateVector_;

  ko_3_.setSamplingTime(dt_);
  ko_3_.setWithUnmodeledWrench(true);
  ko_3_.setWithGyroBias(false);

  inertiaMatrix_ = inertiaMatrix_ * inertiaMatrix_.transpose();
  inertiaMatrix_d_ = inertiaMatrix_d_ * inertiaMatrix_d_.transpose();

  ori_.setRandom();

  /* Kinetics Observer 1 initialization */

  centroidIMUOri1_.setRandom();
  Kinematics centroidIMUPose1_;
  centroidIMUPose1_.position = centroidIMUPos1_;
  centroidIMUPose1_.orientation = centroidIMUOri1_;
  centroidIMUPose1_.linVel = centroidIMULinVel1_;
  centroidIMUPose1_.angVel = centroidIMUAngVel1_;
  centroidIMUPose1_.linAcc = centroidIMULinAcc1_;
  centroidIMUPose1_.angAcc = centroidIMUAngAcc1_;

  worldContactOri1_.setRandom();
  Kinematics worldContactPose1_;
  worldContactPose1_.position = worldContactPos1_;
  worldContactPose1_.orientation = worldContactOri1_;
  centroidContactOri1_.setRandom();
  Kinematics centroidContactPose1_;
  centroidContactPose1_.position = centroidContactPos1_;
  centroidContactPose1_.orientation = centroidContactOri1_;
  centroidContactPose1_.linVel = centroidContactLinVel1_;
  centroidContactPose1_.angVel = centroidContactAngVel1_;

  Kinematics worldContactPose2_ = worldContactPose1_;
  Kinematics centroidContactPose2_ = centroidContactPose1_;
  Kinematics worldContactPose3_ = worldContactPose1_;
  Kinematics centroidContactPose3_ = centroidContactPose1_;
  Vector3 contactForces3_ = contactForces1_;
  Vector3 contactTorques3_ = contactTorques1_;

  ko_3_.setCenterOfMass(com_, com_d_, com_dd_);
  ko_3_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose1_, 0);

  ko_3_.setCoMAngularMomentum(angularMomentum_, angularMomentum_d_);
  ko_3_.setCoMInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_3_.addContact(worldContactPose1_, 0, K1_, K2_, K3_, K4_);
  ko_3_.addContact(worldContactPose2_, 1, K1_, K2_, K3_, K4_);
  ko_3_.addContact(worldContactPose3_, 2, K1_, K2_, K3_, K4_);
  ko_3_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose1_, 0);
  ko_3_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose2_, 1);
  ko_3_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose3_, 2);

  stateVector_.resize(position_.size() + 4 + linvel_.size() + angvel_.size() + gyroBias1_.size() + extForces_.size()
                      + extTorques_.size()
                      + 3
                            * (worldContactPos1_.size() + worldContactOri1_.toVector4().size() + contactForces1_.size()
                               + contactTorques1_.size()));
  stateVector_ << position_, ori_.toVector4(), linvel_, angvel_, gyroBias1_, extForces_, extTorques_, worldContactPos1_,
      worldContactOri1_.toVector4(), contactForces1_, contactTorques1_, worldContactPos1_,
      worldContactOri1_.toVector4(), contactForces2_, contactTorques2_, worldContactPos1_,
      worldContactOri1_.toVector4(), contactForces3_, contactTorques3_;

  ko_3_.setInitWorldCentroidStateVector(stateVector_);

  ko_3_.update();

  Eigen::MatrixXd contact1_Q_pos = Eigen::MatrixXd::Zero(3, 9);
  contact1_Q_pos.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(0), ko_3_.contactPosIndexTangent(0), 3, 3);
  contact1_Q_pos.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(0), ko_3_.contactPosIndexTangent(1), 3, 3);
  contact1_Q_pos.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(0), ko_3_.contactPosIndexTangent(2), 3, 3);

  Eigen::MatrixXd contact2_Q_pos = Eigen::MatrixXd::Zero(3, 9);
  contact2_Q_pos.setZero();
  contact2_Q_pos.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(1), ko_3_.contactPosIndexTangent(0), 3, 3);
  contact2_Q_pos.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(1), ko_3_.contactPosIndexTangent(1), 3, 3);
  contact2_Q_pos.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(1), ko_3_.contactPosIndexTangent(2), 3, 3);

  Eigen::MatrixXd contact3_Q_pos = Eigen::MatrixXd::Zero(3, 9);
  contact3_Q_pos.setZero();
  contact3_Q_pos.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(2), ko_3_.contactPosIndexTangent(0), 3, 3);
  contact3_Q_pos.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(2), ko_3_.contactPosIndexTangent(1), 3, 3);
  contact3_Q_pos.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(2), ko_3_.contactPosIndexTangent(2), 3, 3);

  Eigen::MatrixXd contact1_Q_ori = Eigen::MatrixXd::Zero(3, 9);
  contact1_Q_ori.setZero();
  contact1_Q_ori.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(0), ko_3_.contactOriIndexTangent(0), 3, 3);
  contact1_Q_ori.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(0), ko_3_.contactOriIndexTangent(1), 3, 3);
  contact1_Q_ori.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(0), ko_3_.contactOriIndexTangent(2), 3, 3);

  Eigen::MatrixXd contact2_Q_ori = Eigen::MatrixXd::Zero(3, 9);
  contact2_Q_ori.setZero();
  contact2_Q_ori.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(1), ko_3_.contactOriIndexTangent(0), 3, 3);
  contact2_Q_ori.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(1), ko_3_.contactOriIndexTangent(1), 3, 3);
  contact2_Q_ori.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(1), ko_3_.contactOriIndexTangent(2), 3, 3);

  Eigen::MatrixXd contact3_Q_ori = Eigen::MatrixXd::Zero(3, 9);
  contact3_Q_ori.setZero();
  contact3_Q_ori.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(2), ko_3_.contactOriIndexTangent(0), 3, 3);
  contact3_Q_ori.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(2), ko_3_.contactOriIndexTangent(1), 3, 3);
  contact3_Q_ori.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(2), ko_3_.contactOriIndexTangent(2), 3, 3);

  //   std::cout << std::endl << "Contact1 pos process: " << std::endl << contact1_Q_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2 pos process: " << std::endl << contact2_Q_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact3 pos process: " << std::endl << contact3_Q_pos.format(CleanFmt_) << std::endl;

  //   std::cout << std::endl << "Contact1 ori process: " << std::endl << contact1_Q_ori.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2 ori process: " << std::endl << contact2_Q_ori.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact3 ori process: " << std::endl << contact3_Q_ori.format(CleanFmt_) << std::endl;

  std::cout << std::endl
            << "################################################### New iter (changing the variances) "
               "###################################################"
            << std::endl;

  ko_3_.setCenterOfMass(com_, com_d_, com_dd_);
  ko_3_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose1_, 0);

  ko_3_.setCoMAngularMomentum(angularMomentum_, angularMomentum_d_);
  ko_3_.setCoMInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_3_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose1_, 0);
  ko_3_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose2_, 1);
  ko_3_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose3_, 2);

  Matrix3 processPos1 = processPos_1_.asDiagonal();
  Matrix3 processPos2 = processPos_2_.asDiagonal();
  Matrix3 processPos3 = processPos_3_.asDiagonal();

  ko_3_.setContactProcessCovMat(0, &processPos1, &processPos1);
  ko_3_.setContactProcessCovMat(1, &processPos2, &processPos2);
  ko_3_.setContactProcessCovMat(2, &processPos3, &processPos3);

  if(!(ko_3_.getStateCovarianceMat().eigenvalues().real().array() > -1e-8).all())
  {
    std::cout << std::endl << "123: The state covariance matrix is no longer positive definite" << std::endl;
    std::cout << std::endl
              << "eigen values: " << ko_3_.getStateCovarianceMat().eigenvalues().real().transpose() << std::endl;
    return errorcode;
  }

  ko_3_.update();

  if(!(ko_3_.getStateCovarianceMat().eigenvalues().real().array() > -1e-8).all())
  {
    std::cout << std::endl << "345: The state covariance matrix is no longer positive definite" << std::endl;
    std::cout << std::endl
              << "eigen values: " << ko_3_.getStateCovarianceMat().eigenvalues().real().transpose() << std::endl;
    return errorcode;
  }

  contact1_Q_pos.setZero();
  contact1_Q_pos.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(0), ko_3_.contactPosIndexTangent(0), 3, 3);
  contact1_Q_pos.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(0), ko_3_.contactPosIndexTangent(1), 3, 3);
  contact1_Q_pos.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(0), ko_3_.contactPosIndexTangent(2), 3, 3);

  contact2_Q_pos.setZero();
  contact2_Q_pos.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(1), ko_3_.contactPosIndexTangent(0), 3, 3);
  contact2_Q_pos.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(1), ko_3_.contactPosIndexTangent(1), 3, 3);
  contact2_Q_pos.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(1), ko_3_.contactPosIndexTangent(2), 3, 3);

  contact3_Q_pos.setZero();
  contact3_Q_pos.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(2), ko_3_.contactPosIndexTangent(0), 3, 3);
  contact3_Q_pos.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(2), ko_3_.contactPosIndexTangent(1), 3, 3);
  contact3_Q_pos.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(2), ko_3_.contactPosIndexTangent(2), 3, 3);

  contact1_Q_ori.setZero();
  contact1_Q_ori.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(0), ko_3_.contactOriIndexTangent(0), 3, 3);
  contact1_Q_ori.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(0), ko_3_.contactOriIndexTangent(1), 3, 3);
  contact1_Q_ori.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(0), ko_3_.contactOriIndexTangent(2), 3, 3);

  contact2_Q_ori.setZero();
  contact2_Q_ori.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(1), ko_3_.contactOriIndexTangent(0), 3, 3);
  contact2_Q_ori.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(1), ko_3_.contactOriIndexTangent(1), 3, 3);
  contact2_Q_ori.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(1), ko_3_.contactOriIndexTangent(2), 3, 3);

  contact3_Q_ori.setZero();
  contact3_Q_ori.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(2), ko_3_.contactOriIndexTangent(0), 3, 3);
  contact3_Q_ori.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(2), ko_3_.contactOriIndexTangent(1), 3, 3);
  contact3_Q_ori.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(2), ko_3_.contactOriIndexTangent(2), 3, 3);

  //   std::cout << std::endl << "Contact1 pos process: " << std::endl << contact1_Q_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2 pos process: " << std::endl << contact2_Q_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact3 pos process: " << std::endl << contact3_Q_pos.format(CleanFmt_) << std::endl;

  //   std::cout << std::endl << "Contact1 ori process: " << std::endl << contact1_Q_ori.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2 ori process: " << std::endl << contact2_Q_ori.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact3 ori process: " << std::endl << contact3_Q_ori.format(CleanFmt_) << std::endl;

  Eigen::MatrixXd contacts_Q_pos = Eigen::MatrixXd::Zero(9, 9);
  contacts_Q_pos.block<3, 9>(0, 0) = contact1_Q_pos;
  contacts_Q_pos.block<3, 9>(3, 0) = contact2_Q_pos;
  contacts_Q_pos.block<3, 9>(6, 0) = contact3_Q_pos;

  Eigen::MatrixXd contacts_Q_pos_analytic = Eigen::MatrixXd::Zero(9, 9);
  contacts_Q_pos_analytic(0, 0) =
      (4.0 / 9.0) * processPos1(0, 0) + (1.0 / 9.0) * processPos2(0, 0) + (1.0 / 9.0) * processPos3(0, 0);
  contacts_Q_pos_analytic(0, 3) =
      -(2.0 / 9.0) * processPos1(0, 0) - (2.0 / 9.0) * processPos2(0, 0) + (1.0 / 9.0) * processPos3(0, 0);
  contacts_Q_pos_analytic(0, 6) =
      -(2.0 / 9.0) * processPos1(0, 0) + (1.0 / 9.0) * processPos2(0, 0) - (2.0 / 9.0) * processPos3(0, 0);

  contacts_Q_pos_analytic(1, 1) =
      (4.0 / 9.0) * processPos1(1, 1) + (1.0 / 9.0) * processPos2(1, 1) + (1.0 / 9.0) * processPos3(1, 1);
  contacts_Q_pos_analytic(1, 4) =
      -(2.0 / 9.0) * processPos1(1, 1) - (2.0 / 9.0) * processPos2(1, 1) + (1.0 / 9.0) * processPos3(1, 1);
  contacts_Q_pos_analytic(1, 7) =
      -(2.0 / 9.0) * processPos1(1, 1) + (1.0 / 9.0) * processPos2(1, 1) - (2.0 / 9.0) * processPos3(1, 1);

  contacts_Q_pos_analytic(2, 2) =
      (4.0 / 9.0) * processPos1(2, 2) + (1.0 / 9.0) * processPos2(2, 2) + (1.0 / 9.0) * processPos3(2, 2);
  contacts_Q_pos_analytic(2, 5) =
      -(2.0 / 9.0) * processPos1(2, 2) - (2.0 / 9.0) * processPos2(2, 2) + (1.0 / 9.0) * processPos3(2, 2);
  contacts_Q_pos_analytic(2, 8) =
      -(2.0 / 9.0) * processPos1(2, 2) + (1.0 / 9.0) * processPos2(2, 2) - (2.0 / 9.0) * processPos3(2, 2);

  contacts_Q_pos_analytic(3, 0) =
      -(2.0 / 9.0) * processPos1(0, 0) - (2.0 / 9.0) * processPos2(0, 0) + (1.0 / 9.0) * processPos3(0, 0);
  contacts_Q_pos_analytic(3, 3) =
      (1.0 / 9.0) * processPos1(0, 0) + (4.0 / 9.0) * processPos2(0, 0) + (1.0 / 9.0) * processPos3(0, 0);
  contacts_Q_pos_analytic(3, 6) =
      (1.0 / 9.0) * processPos1(0, 0) - (2.0 / 9.0) * processPos2(0, 0) - (2.0 / 9.0) * processPos3(0, 0);

  contacts_Q_pos_analytic(4, 1) =
      -(2.0 / 9.0) * processPos1(1, 1) - (2.0 / 9.0) * processPos2(1, 1) + (1.0 / 9.0) * processPos3(1, 1);
  contacts_Q_pos_analytic(4, 4) =
      (1.0 / 9.0) * processPos1(1, 1) + (4.0 / 9.0) * processPos2(1, 1) + (1.0 / 9.0) * processPos3(1, 1);
  contacts_Q_pos_analytic(4, 7) =
      (1.0 / 9.0) * processPos1(1, 1) - (2.0 / 9.0) * processPos2(1, 1) - (2.0 / 9.0) * processPos3(1, 1);

  contacts_Q_pos_analytic(5, 2) =
      -(2.0 / 9.0) * processPos1(2, 2) - (2.0 / 9.0) * processPos2(2, 2) + (1.0 / 9.0) * processPos3(2, 2);
  contacts_Q_pos_analytic(5, 5) =
      (1.0 / 9.0) * processPos1(2, 2) + (4.0 / 9.0) * processPos2(2, 2) + (1.0 / 9.0) * processPos3(2, 2);
  contacts_Q_pos_analytic(5, 8) =
      (1.0 / 9.0) * processPos1(2, 2) - (2.0 / 9.0) * processPos2(2, 2) - (2.0 / 9.0) * processPos3(2, 2);

  contacts_Q_pos_analytic(6, 0) =
      -(2.0 / 9.0) * processPos1(0, 0) + (1.0 / 9.0) * processPos2(0, 0) - (2.0 / 9.0) * processPos3(0, 0);
  contacts_Q_pos_analytic(6, 3) =
      (1.0 / 9.0) * processPos1(0, 0) - (2.0 / 9.0) * processPos2(0, 0) - (2.0 / 9.0) * processPos3(0, 0);
  contacts_Q_pos_analytic(6, 6) =
      (1.0 / 9.0) * processPos1(0, 0) + (1.0 / 9.0) * processPos2(0, 0) + (4.0 / 9.0) * processPos3(0, 0);

  contacts_Q_pos_analytic(7, 1) =
      -(2.0 / 9.0) * processPos1(1, 1) + (1.0 / 9.0) * processPos2(1, 1) - (2.0 / 9.0) * processPos3(1, 1);
  contacts_Q_pos_analytic(7, 4) =
      (1.0 / 9.0) * processPos1(1, 1) - (2.0 / 9.0) * processPos2(1, 1) - (2.0 / 9.0) * processPos3(1, 1);
  contacts_Q_pos_analytic(7, 7) =
      (1.0 / 9.0) * processPos1(1, 1) + (1.0 / 9.0) * processPos2(1, 1) + (4.0 / 9.0) * processPos3(1, 1);

  contacts_Q_pos_analytic(8, 2) =
      -(2.0 / 9.0) * processPos1(2, 2) + (1.0 / 9.0) * processPos2(2, 2) - (2.0 / 9.0) * processPos3(2, 2);
  contacts_Q_pos_analytic(8, 5) =
      (1.0 / 9.0) * processPos1(2, 2) - (2.0 / 9.0) * processPos2(2, 2) - (2.0 / 9.0) * processPos3(2, 2);
  contacts_Q_pos_analytic(8, 8) =
      (1.0 / 9.0) * processPos1(2, 2) + (1.0 / 9.0) * processPos2(2, 2) + (4.0 / 9.0) * processPos3(2, 2);

  Eigen::MatrixXd contacts_Q_ori = Eigen::MatrixXd::Zero(9, 9);
  contacts_Q_ori.block<3, 9>(0, 0) = contact1_Q_ori;
  contacts_Q_ori.block<3, 9>(3, 0) = contact2_Q_ori;
  contacts_Q_ori.block<3, 9>(6, 0) = contact3_Q_ori;

  Eigen::MatrixXd contacts_Q_ori_analytic = Eigen::MatrixXd::Zero(9, 9);
  //   contacts_Q_ori_analytic(0, 0) = processPos1(0, 0);
  //   contacts_Q_ori_analytic(1, 1) = processPos1(1, 1);

  contacts_Q_ori_analytic(2, 2) =
      (4.0 / 9.0) * processPos1(2, 2) + (1.0 / 9.0) * processPos2(2, 2) + (1.0 / 9.0) * processPos3(2, 2);
  contacts_Q_ori_analytic(2, 5) =
      -(2.0 / 9.0) * processPos1(2, 2) - (2.0 / 9.0) * processPos2(2, 2) + (1.0 / 9.0) * processPos3(2, 2);
  contacts_Q_ori_analytic(2, 8) =
      -(2.0 / 9.0) * processPos1(2, 2) + (1.0 / 9.0) * processPos2(2, 2) - (2.0 / 9.0) * processPos3(2, 2);

  //   contacts_Q_ori_analytic(3, 3) = processPos2(0, 0);
  //   contacts_Q_ori_analytic(4, 4) = processPos2(1, 1);

  contacts_Q_ori_analytic(5, 2) =
      -(2.0 / 9.0) * processPos1(2, 2) - (2.0 / 9.0) * processPos2(2, 2) + (1.0 / 9.0) * processPos3(2, 2);
  contacts_Q_ori_analytic(5, 5) =
      (1.0 / 9.0) * processPos1(2, 2) + (4.0 / 9.0) * processPos2(2, 2) + (1.0 / 9.0) * processPos3(2, 2);
  contacts_Q_ori_analytic(5, 8) =
      (1.0 / 9.0) * processPos1(2, 2) - (2.0 / 9.0) * processPos2(2, 2) - (2.0 / 9.0) * processPos3(2, 2);

  //   contacts_Q_ori_analytic(6, 6) = processPos3(0, 0);
  //   contacts_Q_ori_analytic(7, 7) = processPos3(1, 1);

  contacts_Q_ori_analytic(8, 2) =
      -(2.0 / 9.0) * processPos1(2, 2) + (1.0 / 9.0) * processPos2(2, 2) - (2.0 / 9.0) * processPos3(2, 2);
  contacts_Q_ori_analytic(8, 5) =
      (1.0 / 9.0) * processPos1(2, 2) - (2.0 / 9.0) * processPos2(2, 2) - (2.0 / 9.0) * processPos3(2, 2);
  contacts_Q_ori_analytic(8, 8) =
      (1.0 / 9.0) * processPos1(2, 2) + (1.0 / 9.0) * processPos2(2, 2) + (4.0 / 9.0) * processPos3(2, 2);

  if((contacts_Q_pos - contacts_Q_pos_analytic).norm() > 1e-15)
  {
    std::cout << std::endl
              << "Error, the numerical matrix for the rest position process doesn't match the analytical one."
              << std::endl;
    std::cout << std::endl << "contacts_Q_pos: " << std::endl << contacts_Q_pos.format(CleanFmt_) << std::endl;
    std::cout << std::endl
              << "contacts_Q_pos_analytic: " << std::endl
              << contacts_Q_pos_analytic.format(CleanFmt_) << std::endl;

    return errorcode;
  }
  if((contacts_Q_ori - contacts_Q_ori_analytic).norm() > 1e-15)
  {
    std::cout << std::endl
              << "Error, the numerical matrix for the rest orientation process doesn't match the analytical one."
              << std::endl;
    std::cout << std::endl << "contacts_Q_ori: " << std::endl << contacts_Q_ori.format(CleanFmt_) << std::endl;
    std::cout << std::endl
              << "contacts_Q_ori_analytic: " << std::endl
              << contacts_Q_ori_analytic.format(CleanFmt_) << std::endl;

    return errorcode;
  }

  std::cout << std::endl
            << "################################################### New iter: we remove the contact 2 "
               "###################################################"
            << std::endl;

  /* We remove the last contact to verify that we will have the same result as when we had two contacts */

  // we save this index as it cannot be accessed once the contact is removed
  Index contact2IndexPos = ko_3_.contactPosIndexTangent(2);
  Index contact2IndexOri = ko_3_.contactOriIndexTangent(2);

  ko_3_.setCenterOfMass(com_, com_d_, com_dd_);
  ko_3_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose1_, 0);

  ko_3_.setCoMAngularMomentum(angularMomentum_, angularMomentum_d_);
  ko_3_.setCoMInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_3_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose1_, 0);
  ko_3_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose2_, 1);

  std::cout << std::endl
            << "eigenvalues before removing: " << ko_3_.getStateCovarianceMat().eigenvalues().transpose() << std::endl;
  if(!(ko_3_.getStateCovarianceMat().eigenvalues().real().array() > -1e-8).all())
  {
    std::cout << std::endl << "The state covariance matrix is not positive definite at the beginning!" << std::endl;
    return errorcode;
  }
  // we remove the contact 2
  ko_3_.removeContact(2);
  ko_3_.update();

  contact1_Q_pos.setZero();
  contact1_Q_pos.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(0), ko_3_.contactPosIndexTangent(0), 3, 3);
  contact1_Q_pos.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(0), ko_3_.contactPosIndexTangent(1), 3, 3);
  contact1_Q_pos.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(0), contact2IndexPos, 3, 3);

  contact2_Q_pos.setZero();
  contact2_Q_pos.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(1), ko_3_.contactPosIndexTangent(0), 3, 3);
  contact2_Q_pos.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(1), ko_3_.contactPosIndexTangent(1), 3, 3);
  contact2_Q_pos.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactPosIndexTangent(1), contact2IndexPos, 3, 3);

  contact3_Q_pos.setZero();
  contact3_Q_pos.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(contact2IndexPos, ko_3_.contactPosIndexTangent(0), 3, 3);
  contact3_Q_pos.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(contact2IndexPos, ko_3_.contactPosIndexTangent(1), 3, 3);
  contact3_Q_pos.block(0, 6, 3, 3) = ko_3_.getEKF().getQ().block(contact2IndexPos, contact2IndexPos, 3, 3);

  contact1_Q_ori.setZero();
  contact1_Q_ori.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(0), ko_3_.contactOriIndexTangent(0), 3, 3);
  contact1_Q_ori.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(0), ko_3_.contactOriIndexTangent(1), 3, 3);
  contact1_Q_ori.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(0), contact2IndexOri, 3, 3);

  contact2_Q_ori.setZero();
  contact2_Q_ori.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(1), ko_3_.contactOriIndexTangent(0), 3, 3);
  contact2_Q_ori.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(1), ko_3_.contactOriIndexTangent(1), 3, 3);
  contact2_Q_ori.block(0, 6, 3, 3) =
      ko_3_.getEKF().getQ().block(ko_3_.contactOriIndexTangent(1), contact2IndexOri, 3, 3);

  contact3_Q_ori.setZero();
  contact3_Q_ori.block(0, 0, 3, 3) =
      ko_3_.getEKF().getQ().block(contact2IndexOri, ko_3_.contactOriIndexTangent(0), 3, 3);
  contact3_Q_ori.block(0, 3, 3, 3) =
      ko_3_.getEKF().getQ().block(contact2IndexOri, ko_3_.contactOriIndexTangent(1), 3, 3);
  contact3_Q_ori.block(0, 6, 3, 3) = ko_3_.getEKF().getQ().block(contact2IndexOri, contact2IndexOri, 3, 3);

  //   std::cout << std::endl << "Contact1 pos process: " << std::endl << contact1_Q_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2 pos process: " << std::endl << contact2_Q_pos.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact3 pos process: " << std::endl << contact3_Q_pos.format(CleanFmt_) << std::endl;

  //   std::cout << std::endl << "Contact1 ori process: " << std::endl << contact1_Q_ori.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact2 ori process: " << std::endl << contact2_Q_ori.format(CleanFmt_) << std::endl;
  //   std::cout << std::endl << "Contact3 ori process: " << std::endl << contact3_Q_ori.format(CleanFmt_) << std::endl;

  Eigen::MatrixXd contacts_Q_pos_bis = Eigen::MatrixXd::Zero(6, 6);
  contacts_Q_pos_bis.block<3, 6>(0, 0) = contact1_Q_pos.block(0, 0, 3, 6);
  contacts_Q_pos_bis.block<3, 6>(3, 0) = contact2_Q_pos.block(0, 0, 3, 6);

  Eigen::MatrixXd contacts_Q_pos_analytic_bis = Eigen::MatrixXd::Zero(6, 6);
  contacts_Q_pos_analytic_bis(0, 0) = 0.25 * processPos1(0, 0) + 0.25 * processPos2(0, 0);
  contacts_Q_pos_analytic_bis(0, 3) = -0.25 * processPos1(0, 0) - 0.25 * processPos2(0, 0);
  contacts_Q_pos_analytic_bis(1, 1) = 0.25 * processPos1(1, 1) + 0.25 * processPos2(1, 1);
  contacts_Q_pos_analytic_bis(1, 4) = -0.25 * processPos1(1, 1) - 0.25 * processPos2(1, 1);
  contacts_Q_pos_analytic_bis(2, 2) = 0.25 * processPos1(2, 2) + 0.25 * processPos2(2, 2);
  contacts_Q_pos_analytic_bis(2, 5) = -0.25 * processPos1(2, 2) - 0.25 * processPos2(2, 2);
  contacts_Q_pos_analytic_bis(3, 0) = -0.25 * processPos1(0, 0) - 0.25 * processPos2(0, 0);
  contacts_Q_pos_analytic_bis(3, 3) = 0.25 * processPos1(0, 0) + 0.25 * processPos2(0, 0);
  contacts_Q_pos_analytic_bis(4, 1) = -0.25 * processPos1(1, 1) - 0.25 * processPos2(1, 1);
  contacts_Q_pos_analytic_bis(4, 4) = 0.25 * processPos1(1, 1) + 0.25 * processPos2(1, 1);
  contacts_Q_pos_analytic_bis(5, 2) = -0.25 * processPos1(2, 2) - 0.25 * processPos2(2, 2);
  contacts_Q_pos_analytic_bis(5, 5) = 0.25 * processPos1(2, 2) + 0.25 * processPos2(2, 2);

  Eigen::MatrixXd contacts_Q_ori_bis = Eigen::MatrixXd::Zero(6, 6);
  contacts_Q_ori_bis.block<3, 6>(0, 0) = contact1_Q_ori.block(0, 0, 3, 6);
  contacts_Q_ori_bis.block<3, 6>(3, 0) = contact2_Q_ori.block(0, 0, 3, 6);

  Eigen::MatrixXd contacts_Q_ori_analytic_bis = Eigen::MatrixXd::Zero(6, 6);
  //   contacts_Q_ori_analytic_bis(0, 0) = processPos1(0, 0);
  //   contacts_Q_ori_analytic_bis(1, 1) = processPos1(1, 1);
  contacts_Q_ori_analytic_bis(2, 2) = 0.25 * processPos1(2, 2) + 0.25 * processPos2(2, 2);
  contacts_Q_ori_analytic_bis(2, 5) = -0.25 * processPos1(2, 2) - 0.25 * processPos2(2, 2);
  //   contacts_Q_ori_analytic_bis(3, 3) = processPos2(0, 0);
  //   contacts_Q_ori_analytic_bis(4, 4) = processPos2(1, 1);
  contacts_Q_ori_analytic_bis(5, 2) = -0.25 * processPos1(2, 2) - 0.25 * processPos2(2, 2);
  contacts_Q_ori_analytic_bis(5, 5) = 0.25 * processPos1(2, 2) + 0.25 * processPos2(2, 2);

  if((contacts_Q_pos_bis - contacts_Q_pos_analytic_bis).norm() > 1e-16)
  {
    std::cout << std::endl
              << "Error, the numerical matrix for the rest position process doesn't match the analytical one."
              << std::endl;
    std::cout << std::endl << "contacts_Q_pos_bis: " << std::endl << contacts_Q_pos_bis.format(CleanFmt_) << std::endl;
    std::cout << std::endl
              << "contacts_Q_pos_analytic_bis: " << std::endl
              << contacts_Q_pos_analytic_bis.format(CleanFmt_) << std::endl;

    return errorcode;
  }
  if((contacts_Q_ori_bis - contacts_Q_ori_analytic_bis).norm() > 1e-16)
  {
    std::cout << std::endl
              << "Error, the numerical matrix for the rest orientation process doesn't match the analytical one."
              << std::endl;
    std::cout << std::endl << "contacts_Q_ori_bis: " << std::endl << contacts_Q_ori_bis.format(CleanFmt_) << std::endl;
    std::cout << std::endl
              << "contacts_Q_ori_analytic_bis: " << std::endl
              << contacts_Q_ori_analytic_bis.format(CleanFmt_) << std::endl;

    return errorcode;
  }

  if(!(ko_3_.getStateCovarianceMat().eigenvalues().real().array() > -1e-8).all())
  {
    std::cout << std::endl << "The state covariance matrix is no longer positive definite" << std::endl;
    std::cout << std::endl
              << "eigen values: " << ko_3_.getStateCovarianceMat().eigenvalues().real().transpose() << std::endl;
    return errorcode;
  }

  return 0;
}

int main()
{

  int returnVal;

  if((returnVal = testKineticsObserverCodeAccessor(1)))
  {
    std::cout << "Kinetics Observer test failed, code : 1" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "Kinetics Observer test succeeded" << std::endl;
  }

  std::cout << "testContactRestPoseCovariance_1contact started" << std::endl;
  if((returnVal = testContactRestPoseCovariance_1contact(2)))
  {
    std::cout << "testContactRestPoseCovariance_1contact failed, code : 2" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testContactRestPoseCovariance_1contact succeeded" << std::endl;
  }

  std::cout << "testContactRestPoseCovariance_2contact started" << std::endl;
  if((returnVal = testContactRestPoseCovariance_2contacts(3)))
  {
    std::cout << "testContactRestPoseCovariance_2contacts failed, code : 3" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testContactRestPoseCovariance_2contacts succeeded" << std::endl;
  }

  std::cout << "testContactRestPoseCovariance_3contact started" << std::endl;
  if((returnVal = testContactRestPoseCovariance_3contacts(4)))
  {
    std::cout << "testContactRestPoseCovariance_3contacts failed, code : 4" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testContactRestPoseCovariance_3contacts succeeded" << std::endl;
  }

  std::cout << "test succeeded" << std::endl;
  return 0;
}
