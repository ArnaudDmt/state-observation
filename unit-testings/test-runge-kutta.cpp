#include <bitset>
#include <iostream>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

using namespace stateObservation;
using namespace kine;

/// @brief test rotationMatrix2Angle
///
/// @param errorCode
/// @return int

double lin_stiffness_ = (double)rand() / RAND_MAX * 1e5;
double lin_damping_ = (double)rand() / RAND_MAX * 5 * 1e1;
double ang_stiffness_ = (double)rand() / RAND_MAX * 1e5;
double ang_damping_ = (double)rand() / RAND_MAX * 5 * 1e1;

Matrix3 K1 = lin_stiffness_ * Matrix3::Identity();
Matrix3 K2 = lin_damping_ * Matrix3::Identity();
Matrix3 K3 = ang_stiffness_ * Matrix3::Identity();
Matrix3 K4 = ang_damping_ * Matrix3::Identity();

struct FunctorConstAccLocKine : kine::LocalKinematics::RecursiveAccelerationFunctorBase
{
  FunctorConstAccLocKine() {}

  void computeRecursiveLocalAccelerations_(LocalKinematics & locKine)
  {
    locKine.linAcc = linAcc;
    locKine.angAcc = angAcc;
  }
  Vector3 linAcc;
  Vector3 angAcc;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct FunctorVarAccLocKine : kine::LocalKinematics::RecursiveAccelerationFunctorBase
{
  FunctorVarAccLocKine() {}

  void computeRecursiveLocalAccelerations_(LocalKinematics & locKine)
  {
    Kinematics kine(locKine);
    kine.linAcc = -K1 * locKine.position() - K2 * locKine.linVel();
    kine.angAcc = -K3 * locKine.orientation.toRotationVector() - K4 * locKine.angVel();
    locKine = kine;
  }
};

struct FunctorConstAccKine : kine::Kinematics::RecursiveAccelerationFunctorBase
{
  FunctorConstAccKine() {}

  void computeRecursiveGlobalAccelerations_(Kinematics & kine)
  {
    kine.linAcc = linAcc;
    kine.angAcc = angAcc;
  }
  Vector3 linAcc;
  Vector3 angAcc;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct FunctorVarAccKine : kine::Kinematics::RecursiveAccelerationFunctorBase
{
  FunctorVarAccKine() {}

  void computeRecursiveGlobalAccelerations_(Kinematics & kine)
  {
    kine.linAcc = -K1 * kine.position() - K2 * kine.linVel();
    kine.angAcc = -K3 * kine.orientation.toRotationVector() - K4 * kine.angVel();
  }
};

struct FunctorConstAccKineSameAsLocKine : kine::Kinematics::RecursiveAccelerationFunctorBase
{
  FunctorConstAccKineSameAsLocKine() {}

  void computeRecursiveGlobalAccelerations_(Kinematics & kine)
  {
    kine.linAcc = kine.orientation * linAcc; // we consider the acceleration is constant in the local frame
    kine.angAcc = kine.orientation * angAcc;
  }
  Vector3 linAcc;
  Vector3 angAcc;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

int testRungeKuttaLocalKinematicsConstantAcceleration(int errcode) // 1
{
  FunctorConstAccLocKine functorAccConst;

  functorAccConst.linAcc = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
  functorAccConst.angAcc = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();

  double dt = 0.005;
  double threshold = 1e-4;
  double err = 0;

  LocalKinematics k;

  Vector3 pos = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
  kine::Orientation ori(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
  Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
  Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

  k.position = pos;
  k.orientation = ori;
  k.linVel = linvel;
  k.angVel = angvel;

  functorAccConst.computeRecursiveLocalAccelerations_(k);

  LocalKinematics rk(k);

  rk.integrateRungeKutta4(dt, functorAccConst);

  int maxT = 10;
  for(int i = 0; i < maxT; i++)
  {
    k.integrate(dt / maxT);
  }

  LocalKinematics diff = rk * k.getInverse();

  if(diff.position.isSet())
  {
    err += diff.position().squaredNorm();
  }
  if(diff.orientation.isSet())
  {
    err += diff.orientation.toRotationVector().squaredNorm();
  }
  if(diff.linVel.isSet())
  {
    err += diff.linVel().squaredNorm();
  }
  if(diff.angVel.isSet())
  {
    err += diff.angVel().squaredNorm();
  }

  err = sqrt(err);

  std::cout << "Error between Runge Kutta integrations and consecutive integrations for LocalKinematics with constant "
               "acceleration: "
            << err << std::endl;

  if(err > threshold)
  {
    return errcode;
  }
  return 0;
}

int testRungeKuttaKinematicsConstantAcceleration(int errcode) // 2
{
  FunctorConstAccKine functorAccConst;

  functorAccConst.linAcc = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
  functorAccConst.angAcc = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();

  double dt = 0.005;
  double threshold = 1e-4;
  double err = 0;

  Kinematics k;

  Vector3 pos = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
  kine::Orientation ori(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
  Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
  Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

  k.position = pos;
  k.orientation = ori;
  k.linVel = linvel;
  k.angVel = angvel;

  functorAccConst.computeRecursiveGlobalAccelerations_(k);

  Kinematics rk(k);

  rk.integrateRungeKutta4(dt, functorAccConst);

  int maxT = 10;
  for(int i = 0; i < maxT; i++)
  {
    k.integrate(dt / maxT);
  }

  Kinematics diff = rk * k.getInverse();

  if(diff.position.isSet())
  {
    err += diff.position().squaredNorm();
  }
  if(diff.orientation.isSet())
  {
    err += diff.orientation.toRotationVector().squaredNorm();
  }
  if(diff.linVel.isSet())
  {
    err += diff.linVel().squaredNorm();
  }
  if(diff.angVel.isSet())
  {
    err += diff.angVel().squaredNorm();
  }
  err = sqrt(err);

  std::cout << "Error between Runge Kutta integrations and consecutive integrations for Kinematics with constant "
               "acceleration: "
            << err << std::endl;

  if(err > threshold)
  {
    return errcode;
  }
  return 0;
}

int testRungeKuttaLocalKinematicsVariableAcceleration(int errcode) // 3
{
  FunctorVarAccLocKine functorVarAcc;

  lin_stiffness_ = (double)rand() / RAND_MAX * 1e1;
  lin_damping_ = (double)rand() / RAND_MAX * 5 * 1e-2;
  ang_stiffness_ = (double)rand() / RAND_MAX * 1e1;
  ang_damping_ = (double)rand() / RAND_MAX * 5 * 1e0;

  K1 = lin_stiffness_ * Matrix3::Identity();
  K2 = lin_damping_ * Matrix3::Identity();
  K3 = ang_stiffness_ * Matrix3::Identity();
  K4 = ang_damping_ * Matrix3::Identity();

  double dt = 0.005;
  double threshold = 1e-4;
  double err = 0;

  LocalKinematics k;

  Vector3 pos = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
  kine::Orientation ori(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
  Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
  Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

  k.position = pos;
  k.orientation = ori;
  k.linVel = linvel;
  k.angVel = angvel;

  functorVarAcc.computeRecursiveLocalAccelerations_(k);

  LocalKinematics rk(k);

  rk.integrateRungeKutta4(dt, functorVarAcc);

  int maxT = 10;
  for(int i = 0; i < maxT; i++)
  {
    k.integrate(dt / maxT);
    functorVarAcc.computeRecursiveLocalAccelerations_(k);
  }

  LocalKinematics diff = rk * k.getInverse();

  if(diff.position.isSet())
  {
    err += diff.position().squaredNorm();
  }
  if(diff.orientation.isSet())
  {
    err += diff.orientation.toRotationVector().squaredNorm();
  }
  if(diff.linVel.isSet())
  {
    err += diff.linVel().squaredNorm();
  }
  if(diff.angVel.isSet())
  {
    err += diff.angVel().squaredNorm();
  }
  err = sqrt(err);

  std::cout << "Error between Runge Kutta integrations and consecutive integrations for LocalKinematics with variable "
               "acceleration: "
            << err << std::endl;

  if(err > threshold)
  {
    return errcode;
  }
  return 0;
}

int testRungeKuttaKinematicsVariableAcceleration(int errcode) // 4
{
  FunctorVarAccKine functorVarAcc;

  lin_stiffness_ = (double)rand() / RAND_MAX * 3e2;
  lin_damping_ = (double)rand() / RAND_MAX * 5 * 5e-1;
  ang_stiffness_ = (double)rand() / RAND_MAX * 3e2;
  ang_damping_ = (double)rand() / RAND_MAX * 5 * 5e-2;

  K1 = lin_stiffness_ * Matrix3::Identity();
  K2 = lin_damping_ * Matrix3::Identity();
  K3 = ang_stiffness_ * Matrix3::Identity();
  K4 = ang_damping_ * Matrix3::Identity();

  double dt = 0.005;
  double threshold = 1e-4;
  double err = 0;

  Kinematics k;

  Vector3 pos = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
  kine::Orientation ori(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
  Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
  Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

  k.position = pos;
  k.orientation = ori;
  k.linVel = linvel;
  k.angVel = angvel;

  functorVarAcc.computeRecursiveGlobalAccelerations_(k);

  Kinematics rk(k);

  rk.integrateRungeKutta4(dt, functorVarAcc);

  int maxT = 10;
  for(int i = 0; i < maxT; i++)
  {
    k.integrate(dt / maxT);
    functorVarAcc.computeRecursiveGlobalAccelerations_(k);
  }

  Kinematics diff = rk * k.getInverse();

  if(diff.position.isSet())
  {
    err += diff.position().squaredNorm();
  }
  if(diff.orientation.isSet())
  {
    err += diff.orientation.toRotationVector().squaredNorm();
  }
  if(diff.linVel.isSet())
  {
    err += diff.linVel().squaredNorm();
  }
  if(diff.angVel.isSet())
  {
    err += diff.angVel().squaredNorm();
  }
  err = sqrt(err);

  std::cout << "Error between Runge Kutta integrations and consecutive integrations for Kinematics with variable "
               "acceleration: "
            << err << std::endl;

  if(err > threshold)
  {
    return errcode;
  }
  return 0;
}

int testRungeKuttaKinematicsVsLocalKinematicsConstantAcceleration(int errcode) // 5
{
  FunctorConstAccLocKine functorConstAccLocKine;
  FunctorConstAccKineSameAsLocKine functorConstAcctKine;

  functorConstAccLocKine.linAcc = functorConstAcctKine.linAcc =
      tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
  functorConstAccLocKine.angAcc = functorConstAcctKine.angAcc =
      tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();

  double dt = 0.005;
  double threshold = 1e-4;
  double err = 0;
  int iterations = 1;

  LocalKinematics lk;

  Vector3 pos = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
  kine::Orientation ori(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
  Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
  Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

  lk.position = pos;
  lk.orientation = ori;
  lk.linVel = linvel;
  lk.angVel = angvel;

  Kinematics k(lk);

  functorConstAccLocKine.computeRecursiveLocalAccelerations_(lk);
  functorConstAcctKine.computeRecursiveGlobalAccelerations_(k);

  for(int i = 0; i < iterations; i++) lk.integrateRungeKutta4(dt, functorConstAccLocKine);
  k.integrateRungeKutta4(dt, functorConstAcctKine);

  Kinematics diff = Kinematics(lk) * k.getInverse();

  if(diff.position.isSet())
  {
    err += diff.position().squaredNorm();
  }
  if(diff.orientation.isSet())
  {
    err += diff.orientation.toRotationVector().squaredNorm();
  }
  if(diff.linVel.isSet())
  {
    err += diff.linVel().squaredNorm();
  }
  if(diff.angVel.isSet())
  {
    err += diff.angVel().squaredNorm();
  }
  err = sqrt(err);

  std::cout << "Error between Runge Kutta integrations for Kinematics vs LocalKinematics with constant acceleration: "
            << err << std::endl;

  if(err > threshold)
  {
    return errcode;
  }
  return 0;
}

int testRungeKuttaKinematicsVsLocalKinematicsVariableAcceleration(int errcode) // 6
{
  FunctorVarAccKine functorVarAccKine;
  FunctorVarAccLocKine functorVarAccLocKine;

  lin_stiffness_ = (double)rand() / RAND_MAX * 3e2;
  lin_damping_ = (double)rand() / RAND_MAX * 5 * 5e-1;
  ang_stiffness_ = (double)rand() / RAND_MAX * 3e2;
  ang_damping_ = (double)rand() / RAND_MAX * 5 * 5e-2;

  K1 = lin_stiffness_ * Matrix3::Identity();
  K2 = lin_damping_ * Matrix3::Identity();
  K3 = ang_stiffness_ * Matrix3::Identity();
  K4 = ang_damping_ * Matrix3::Identity();

  double dt = 0.005;
  double threshold = 1e-4;
  double err = 0;
  int iterations = 1;

  LocalKinematics lk;

  Vector3 pos = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 40;
  kine::Orientation ori(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 25));
  Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 40;
  Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 25;

  lk.position = pos;
  lk.orientation = ori;
  lk.linVel = linvel;
  lk.angVel = angvel;

  Kinematics k(lk);

  functorVarAccLocKine.computeRecursiveLocalAccelerations_(lk);
  functorVarAccKine.computeRecursiveGlobalAccelerations_(k);

  for(int i = 0; i < iterations; i++)
  {
    lk.integrateRungeKutta4(dt, functorVarAccLocKine);
    k.integrateRungeKutta4(dt, functorVarAccKine);
  }

  Kinematics diff = Kinematics(lk) * k.getInverse();

  if(diff.position.isSet())
  {
    err += diff.position().squaredNorm();
  }
  if(diff.orientation.isSet())
  {
    err += diff.orientation.toRotationVector().squaredNorm();
  }
  if(diff.linVel.isSet())
  {
    err += diff.linVel().squaredNorm();
  }
  if(diff.angVel.isSet())
  {
    err += diff.angVel().squaredNorm();
  }
  err = sqrt(err);

  std::cout << "Error between Runge Kutta integrations for Kinematics vs LocalKinematics with variable acceleration: "
            << err << std::endl;

  if(err > threshold)
  {
    return errcode;
  }
  return 0;
}

int main()
{
  int returnVal;
  int errorcode = 0;

  if((returnVal = testRungeKuttaLocalKinematicsConstantAcceleration(++errorcode)))
  {
    std::cout << "testRungeKuttaLocalKinematicsConstantAcceleration Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testRungeKuttaLocalKinematicsConstantAcceleration succeeded" << std::endl;
  }

  if((returnVal = testRungeKuttaKinematicsConstantAcceleration(++errorcode)))
  {
    std::cout << "testRungeKuttaKinematicsConstantAcceleration Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testRungeKuttaKinematicsConstantAcceleration succeeded" << std::endl;
  }

  if((returnVal = testRungeKuttaLocalKinematicsVariableAcceleration(++errorcode)))
  {
    std::cout << "testRungeKuttaLocalKinematicsVariableAcceleration Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testRungeKuttaLocalKinematicsVariableAcceleration succeeded" << std::endl;
  }

  if((returnVal = testRungeKuttaKinematicsVariableAcceleration(++errorcode)))
  {
    std::cout << "testRungeKuttaKinematicsVariableAcceleration Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testRungeKuttaKinematicsVariableAcceleration succeeded" << std::endl;
  }

  if((returnVal = testRungeKuttaKinematicsVsLocalKinematicsConstantAcceleration(++errorcode)))
  {
    std::cout << "testRungeKuttaKinematicsVsLocalKinematicsConstantAcceleration Failed, error code: " << returnVal
              << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testRungeKuttaKinematicsVsLocalKinematicsConstantAcceleration succeeded" << std::endl;
  }

  if((returnVal = testRungeKuttaKinematicsVsLocalKinematicsVariableAcceleration(++errorcode)))
  {
    std::cout << "testRungeKuttaKinematicsVsLocalKinematicsVariableAcceleration Failed, error code: " << returnVal
              << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testRungeKuttaKinematicsVsLocalKinematicsVariableAcceleration succeeded" << std::endl;
  }

  std::cout << "test succeeded" << std::endl;
  return 0;
}