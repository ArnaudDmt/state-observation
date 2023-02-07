#include <bitset>
#include <iostream>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

using namespace stateObservation;
using namespace kine;

double lin_stiffness_ = (double)rand() / RAND_MAX * 1e5;
double lin_damping_ = (double)rand() / RAND_MAX * 5 * 1e1;
double ang_stiffness_ = (double)rand() / RAND_MAX * 1e5;
double ang_damping_ = (double)rand() / RAND_MAX * 5 * 1e1;

Matrix3 K1 = lin_stiffness_ * Matrix3::Identity();
Matrix3 K2 = lin_damping_ * Matrix3::Identity();
Matrix3 K3 = ang_stiffness_ * Matrix3::Identity();
Matrix3 K4 = ang_damping_ * Matrix3::Identity();

Vector3 position = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation ori(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

Vector3 gyroBias1 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;
Vector3 gyroBias2 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

Vector3 extForces = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 extTorques = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 worldContactPos1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation worldContactOri1(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 centroidContactPos1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidContactOri1(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 contactForces1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 contactTorques1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 worldContactPos2 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation worldContactOri2(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 centroidContactPos2 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidContactOri2(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 contactForces2 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 contactTorques2 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Matrix3 inertiaMatrix = tools::ProbabilityLawSimulation::getUniformMatrix<Matrix3>();
Matrix3 inertiaMatrix_d = tools::ProbabilityLawSimulation::getGaussianMatrix<Matrix3>();
Vector3 angularMomentum = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 angularMomentum_d = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector stateVector_(position.size() + ori.toVector4().size() + linvel.size() + angvel.size() + gyroBias1.size()
                    + gyroBias2.size() + extForces.size() + extTorques.size() + worldContactPos1.size()
                    + worldContactOri1.toVector4().size() + contactForces1.size() + contactTorques1.size()
                    + worldContactPos2.size() + worldContactOri2.toVector4().size() + contactForces2.size()
                    + contactTorques2.size());

stateObservation::KineticsObserver ko_(2, 2);
Vector dx_;

int testAccelerationsJacobians(int errcode) // 1
{
  double threshold = 1e-4;

  Matrix accJacobian = ko_.compareAccelerationsJacobians(dx_);

  std::cout << "Error between Runge Kutta integrations and consecutive integrations for LocalKinematics with constant "
               "acceleration: "
            << sqrt(accJacobian.squaredNorm()) << std::endl;

  if(sqrt(accJacobian.squaredNorm()) > threshold)
  {
    return errcode;
  }
}

int testAnalyticalAJacobian(int errcode) // 1
{
  double threshold = 1e-4;

  Matrix A_Jacobian = ko_.compareAnalyticalAndFDJacobians(15);

  std::cout << "Error between Runge Kutta integrations and consecutive integrations for LocalKinematics with constant "
               "acceleration: "
            << sqrt(A_Jacobian.squaredNorm()) << std::endl;

  if(sqrt(A_Jacobian.squaredNorm()) > threshold)
  {
    return errcode;
  }
}

int main()
{
  int returnVal;
  int errorcode = 0;

  /* Kinetics Observer initialization */
  Kinematics worldContactPose1;
  worldContactPose1.position = worldContactPos1;
  worldContactPose1.orientation = worldContactOri1;
  Kinematics centroidContactPose1;
  centroidContactPose1.position = centroidContactPos1;
  centroidContactPose1.orientation = centroidContactOri1;

  ko_.addContact(worldContactPose1, 0, K1, K2, K3, K4);
  ko_.updateContactWithNoSensor(centroidContactPose1, 0);

  Kinematics worldContactPose2;
  worldContactPose2.position = worldContactPos2;
  worldContactPose2.orientation = worldContactOri2;
  Kinematics centroidContactPose2;
  centroidContactPose2.position = centroidContactPos2;
  centroidContactPose2.orientation = centroidContactOri2;

  ko_.addContact(worldContactPose2, 1, K1, K2, K3, K4);
  ko_.updateContactWithNoSensor(centroidContactPose2, 1);

  ko_.setSamplingTime(0.05);
  ko_.setWithUnmodeledWrench(true);
  ko_.setAngularMomentum(angularMomentum, angularMomentum_d);
  ko_.setInertiaMatrix(inertiaMatrix, inertiaMatrix_d);

  stateVector_ << position, ori.toVector4(), linvel, angvel, gyroBias1, gyroBias2, extForces, extTorques,
      worldContactPos1, worldContactPos2, worldContactOri1.toVector4(), worldContactOri2.toVector4(), contactForces1,
      contactForces2, contactTorques1, contactTorques2;
  ko_.setInitWorldCentroidStateVector(stateVector_);

  ko_.getEKF().updateStatePrediction();

  dx_.resize(ko_.getStateTangentSize());
  dx_.setConstant(1e-6);

  if((returnVal = testAccelerationsJacobians(++errorcode)))
  {
    std::cout << "testAccelerationsJacobians Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testAccelerationsJacobians succeeded" << std::endl;
  }

  if((returnVal = testAnalyticalAJacobian(++errorcode)))
  {
    std::cout << "testAnalyticalAJacobian Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testAnalyticalAJacobian succeeded" << std::endl;
  }

  std::cout << "test succeeded" << std::endl;
  return 0;
}