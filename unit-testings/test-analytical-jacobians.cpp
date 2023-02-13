#include <bitset>
#include <iostream>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

using namespace stateObservation;
using namespace kine;

double dt_ = 0.005;

double lin_stiffness_ = (double)rand() / RAND_MAX * 1e5;
double lin_damping_ = (double)rand() / RAND_MAX * 5 * 1e1;
double ang_stiffness_ = (double)rand() / RAND_MAX * 1e5;
double ang_damping_ = (double)rand() / RAND_MAX * 5 * 1e1;

Matrix3 K1 = lin_stiffness_ * Matrix3::Identity();
Matrix3 K2 = lin_damping_ * Matrix3::Identity();
Matrix3 K3 = ang_stiffness_ * Matrix3::Identity();
Matrix3 K4 = ang_damping_ * Matrix3::Identity();

Vector3 com = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 com_d = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 com_dd = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 position = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation ori(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

Vector3 gyroBias1 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

Vector3 extForces = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 extTorques = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 worldContactPos1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation worldContactOri1(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 centroidContactPos1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidContactOri1(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 centroidContactLinVel1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidContactAngVel1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 contactForces1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 contactTorques1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Matrix3 inertiaMatrix_ = tools::ProbabilityLawSimulation::getUniformMatrix<Matrix3>();
Matrix3 inertiaMatrix_d_ = tools::ProbabilityLawSimulation::getGaussianMatrix<Matrix3>();
Vector3 angularMomentum = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 angularMomentum_d = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector stateVector_(position.size() + 4 + linvel.size() + angvel.size() + gyroBias1.size() + extForces.size()
                    + extTorques.size() + worldContactPos1.size() + worldContactOri1.toVector4().size()
                    + contactForces1.size() + contactTorques1.size());

stateObservation::KineticsObserver ko_(1, 1);
Vector dx_;

int testAccelerationsJacobians(int errcode) // 1
{
  double threshold = 1e-4;

  Matrix accDiffJacobian = ko_.compareAccelerationsJacobians(dx_);

  std::cout << "Error between the analytical and the finite differences A Jacobian: " << accDiffJacobian.norm()
            << std::endl;

  if(accDiffJacobian.norm() > threshold)
  {
    return errcode;
  }
}

int testAnalyticalAJacobian(int errcode) // 1
{
  double threshold = 1e-4;

  std::cout << std::endl << "LocalKine before integration : " << std::endl << "gyhuhji" << std::endl;
  Matrix A_Jacobian = ko_.compareAnalyticalAndFDJacobians(15, dx_, false);

  std::cout << "Error between Runge Kutta integrations and consecutive integrations for LocalKinematics with constant "
               "acceleration: "
            << A_Jacobian.norm() << std::endl;

  if(A_Jacobian.norm() > threshold)
  {
    return errcode;
  }
}

void testOrientationsJacobians(const Vector & dx)
{
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

  /* Finite differences Jacobian */
  Matrix rotationJacobianDeltaFD = Matrix::Zero(3, 3);

  Vector currentState = ko_.getEKF().getCurrentEstimatedState();
  Vector accelerations = Vector6::Zero();
  ko_.computeLocalAccelerationsForJacobian_(currentState, accelerations);
  LocalKinematics kineTestOri(currentState);

  kineTestOri.linAcc = accelerations.segment<3>(0);
  kineTestOri.angAcc = accelerations.segment<3>(3);

  Orientation oriBar = Orientation::zeroRotation();
  Orientation oriBarIncremented = Orientation::zeroRotation();
  Vector3 oriBarDiff = Vector3::Zero();

  oriBar = kineTestOri.orientation;
  oriBarIncremented = kineTestOri.orientation;

  // Vector3 dt_x_omega(10000, 24265, 589);
  Vector3 dt_x_omega = dt_ * kineTestOri.angVel() + dt_ * dt_ / 2 * kineTestOri.angAcc();

  oriBar.integrateRightSide(dt_x_omega);

  Vector3 xIncrement = Vector3::Zero();

  for(Index i = 0; i < 3; ++i)
  {
    xIncrement.setZero();
    xIncrement[i] = dx[i];

    Vector3 incremented_dt_x_omega = dt_x_omega + xIncrement;

    oriBarIncremented.integrateRightSide(incremented_dt_x_omega);

    oriBarDiff = oriBar.differentiate(oriBarIncremented);

    oriBarDiff /= dx[i];

    rotationJacobianDeltaFD.col(i) = oriBarDiff;
    oriBarIncremented = kineTestOri.orientation;
  }

  Matrix compareWRTDelta2 =
      2.0 / dt_x_omega.norm()
      * (((dt_x_omega.norm() - 2.0 * sin(dt_x_omega.norm() / 2.0)) / (2.0 * dt_x_omega.squaredNorm()))
             * kineTestOri.orientation.toMatrix3() * dt_x_omega * dt_x_omega.transpose()
         + sin(dt_x_omega.norm() / 2.0) * kineTestOri.orientation.toMatrix3()
               * kine::rotationVectorToRotationMatrix(dt_x_omega / 2.0));

  Matrix compareWRTDelta3 = kineTestOri.orientation.toMatrix3();

  std::cout << std::endl << "v : " << std::endl << dt_x_omega << std::endl;

  std::cout << std::endl << "Ori Jacobian delta : " << std::endl << rotationJacobianDeltaFD << std::endl;
  // std::cout << std::endl << "Ori Jacobian delta comparison 1 : " << std::endl << compareWRTDelta1 << std::endl;
  // std::cout << std::endl << "Ori Jacobian delta comparison 2 : " << std::endl << compareWRTDelta2 << std::endl;
  std::cout << std::endl << "compareWRTDelta3 : " << std::endl << compareWRTDelta3 << std::endl;
}

/*
void testContactTorqueWRTOrientationJacobian(const Vector & dx)
{
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

Matrix torqueOriFD = Matrix::Zero(3, 3);

Vector predictedState = ko_.getEKF().getLastPrediction();
Vector accelerations = Vector6::Zero();
ko_.computeLocalAccelerationsForJacobian_(predictedState, accelerations);

kineTestOri.linAcc = accelerations.segment<3>(0);
kineTestOri.angAcc = accelerations.segment<3>(3);

Vector3 torqueBar = Vector3::Zero();
Vector3 torqueBarIncremented = Vector3::Zero();
Vector3 torqueBarDiff = Vector3::Zero();

Vector3 force = Vector3::Zero();
torqueBar = currentState.segment<3>(ko_.contactTorqueIndex(0));
torqueBarIncremented = currentState.segment<3>(ko_.contactTorqueIndex(0));

Kinematics worldContactRefPose;
worldContactRefPose.fromVector(currentState.segment<7>(ko_.contactPosIndex(0)), ko_.flagsPoseKine);

LocalKinematics stateLocalKine = ko_.estimateAccelerations();
ko_.computeContactForce_(0, stateLocalKine, worldContactRefPose, force, torqueBar);

Vector3 xIncrement = Vector3::Zero();
Vector incrementedX = currentState;

for(Index i = 0; i < 3; ++i)
{
  xIncrement.setZero();
  xIncrement[i] = dx[i];

  incrementedX = currentState + xIncrement;

  worldContactRefPose.fromVector(incrementedX.segment<7>(ko_.contactPosIndex(0)), ko_.flagsPoseKine);

  stateLocalKine = ko_.estimateAccelerations();
  ko_.computeContactForce_(0, stateLocalKine, worldContactRefPose, force, torqueBar);
  oriBarIncremented.integrateRightSide(incremented_dt_x_omega);

  oriBarDiff = oriBar.differentiate(oriBarIncremented);

  oriBarDiff /= dx[i];

  torqueOriFD.col(i) = oriBarDiff;
  stateLocalKine = ko_.estimateAccelerations();
}
}
*/

int main()
{
  int returnVal;
  int errorcode = 0;

  /* Kinetics Observer initialization */
  ori.setRandom();
  worldContactOri1.setRandom();

  Kinematics worldContactPose1;
  worldContactPose1.position = worldContactPos1;
  worldContactPose1.orientation = worldContactOri1;
  Kinematics centroidContactPose1;
  centroidContactPose1.position = centroidContactPos1;
  centroidContactPose1.orientation = centroidContactOri1;
  centroidContactPose1.linVel = centroidContactLinVel1;
  centroidContactPose1.angVel = centroidContactAngVel1;

  // K1.setZero();
  K2.setZero();

  ko_.setCenterOfMass(com, com_d, com_dd);

  ko_.setSamplingTime(dt_);
  ko_.setWithUnmodeledWrench(true);
  ko_.useRungeKutta(false);

  ko_.setAngularMomentum(angularMomentum, angularMomentum_d);
  inertiaMatrix_ = inertiaMatrix_ * inertiaMatrix_.transpose();
  inertiaMatrix_d_ = inertiaMatrix_d_ * inertiaMatrix_d_.transpose();

  ko_.setInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_.addContact(worldContactPose1, 0, K1, K2, K3, K4);
  ko_.updateContactWithNoSensor(centroidContactPose1, 0);

  // angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / (dt * dt);
  /*
  position.setZero();
  angvel.setZero();
  contactForces1.setZero();
  contactTorques1.setZero();
  extTorques.setZero();
  linvel.setZero();
  extForces.setZero();
  angularMomentum.setZero();
  angularMomentum_d.setZero();
  inertiaMatrix_d.setZero();
  ori.setZeroRotation();
  ko_.setInertiaMatrix(inertiaMatrix, inertiaMatrix_d);
  ko_.setAngularMomentum(angularMomentum, angularMomentum_d);
  */
  // extTorques = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() * 10000;

  stateVector_ << position, ori.toVector4(), linvel, angvel, gyroBias1, extForces, extTorques, worldContactPos1,
      worldContactOri1.toVector4(), contactForces1, contactTorques1;

  std::cout << std::endl << "State vector : " << std::endl << stateVector_ << std::endl;

  ko_.setInitWorldCentroidStateVector(stateVector_);

  ko_.getEKF().updateStatePrediction();

  dx_.resize(ko_.getStateTangentSize());
  dx_.setConstant(1e-6);

  testOrientationsJacobians(dx_);

  if((returnVal = testAccelerationsJacobians(++errorcode)))
  {
    std::cout << "testAccelerationsJacobians Failed, error code: " << returnVal << std::endl;
  }
  else
  {
    std::cout << "testAccelerationsJacobians succeeded" << std::endl;
  }

  if((returnVal = testAnalyticalAJacobian(++errorcode)))
  {
    std::cout << "testAnalyticalAJacobian Failed, error code: " << returnVal << std::endl;
  }
  else
  {
    std::cout << "testAnalyticalAJacobian succeeded" << std::endl;
  }

  std::cout << "test succeeded" << std::endl;
  return 0;
}