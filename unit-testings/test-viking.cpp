#include "state-observation/tools/definitions.hpp"
#include "state-observation/tools/rigid-body-kinematics.hpp"
#include <iostream>

#include <state-observation/observer/viking.hpp>

using namespace stateObservation;
using namespace kine;

struct Traj
{
  Traj() : kine(LocalKinematics::zeroKinematics(kine::LocalKinematics::Flags::all)) {}
  void iterate(double dt, bool inMotion = true)
  {
    Vector3 linJerk = tools::ProbabilityLawSimulation::getUniformMatrix(3, 1, -0.2, 0.2) * 10;
    Vector3 angJerk = tools::ProbabilityLawSimulation::getUniformMatrix(3, 1, -0.2, 0.2) * 10;

    if(inMotion)
    {
      kine.linAcc() += linJerk * dt;
      kine.angAcc() += angJerk * dt;
    }
    else
    {
      double K = 20;
      kine.linAcc() = -K * kine.linVel();
      kine.angAcc() = -K * kine.angVel();
    }
    kine.integrate(dt);
  }
  Vector3 getX2()
  {
    return kine.orientation.toMatrix3().transpose() * Vector3::UnitZ();
  }
  Vector3 getPos()
  {
    return kine.orientation.toMatrix3() * kine.position();
  }
  Matrix3 getOri()
  {
    return kine.orientation.toMatrix3();
  }
  Vector getYa()
  {
    return kine.linAcc() + cst::gravityConstant * getX2();
  }
  LocalKinematics & operator()()
  {
    return kine;
  }
  LocalKinematics kine;
};

int testWithoutPosAndOriMeasurement(int errorcode)
{
  double simTime = 1.00;
  double dt = 0.0005;
  int nbIters = int(simTime / dt);

  double err;
  Viking viking(dt, 5, 1, 1, 1, false);
  Traj traj;

  viking.initEstimator(traj().linVel(), traj.getX2(), Vector3::Zero(), traj().orientation.toVector4(),
                       traj().position());

  for(int i = 0; i < nbIters; i++)
  {
    traj.iterate(dt);

    viking.setInput(traj().linVel(), traj.getYa(), traj().angVel(), i);
    // viking.addPosOriMeasurement(traj().position, traj.getOri(), 1, 1, 1);
    viking.getEstimatedState(i + 1);

    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX1> x1_hat = viking.getEstimatedLocLinVel();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX2> x2_hat = viking.getEstimatedTilt();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeOri> q_hat = viking.getEstimatedOrientation();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizePos> pl_hat = viking.getEstimatedLocPosition();

    std::cout << std::endl << "Estimated 2: " << x1_hat.transpose() << std::endl;
    std::cout << std::endl << "Simulated 2: " << traj().linVel().transpose() << std::endl;

    Orientation finalOri_hat;
    finalOri_hat.fromVector4(q_hat);

    Matrix3 oriError = finalOri_hat.toMatrix3() * traj.getOri().transpose();
    Vector3 oriErrorVector = kine::skewSymmetricToRotationVector(oriError - oriError.transpose()) / 2.0;

    err = (x1_hat - traj().linVel()).squaredNorm();
    if(err > 1e-4)
    {
      std::cout << std::endl << "The local velocity estimate is incorrect." << std::endl;
      std::cout << std::endl << "Estimated: " << x1_hat.transpose() << std::endl;
      std::cout << std::endl << "Simulated: " << traj().linVel().transpose() << std::endl;
      return errorcode;
    }

    err = (x2_hat - traj.getX2()).squaredNorm();
    if(err > 1e-4)
    {
      std::cout << std::endl << "The tilt estimate is incorrect." << std::endl;
      std::cout << std::endl << "Estimated: " << x2_hat.transpose() << std::endl;
      std::cout << std::endl << "Simulated: " << traj.getX2().transpose() << std::endl;

      return errorcode;
    }

    err = (pl_hat - traj.getPos()).squaredNorm();
    if(err > 1e-4)
    {
      std::cout << std::endl << "The position estimate is incorrect." << std::endl;
      std::cout << std::endl << "Estimated: " << pl_hat.transpose() << std::endl;
      std::cout << std::endl << "Simulated: " << traj.getPos().transpose() << std::endl;
      return errorcode;
    }

    err = (oriErrorVector).squaredNorm();
    if(err > 1e-4)
    {
      std::cout << std::endl << "The orientation estimate is incorrect." << std::endl;
      std::cout << std::endl << "Error vector: " << oriErrorVector.transpose() << std::endl;
      return errorcode;
    }
  }

  return 0;
}

int testWithPosAndOriMeasurement(int errorcode)
{
  Ajouter mesures orientation et position

      double simTime = 1.00;
  double dt = 0.0005;
  int nbIters = int(simTime / dt);

  double err;
  Viking viking(dt, 5, 1, 1, 1, false);
  Traj traj;

  viking.initEstimator(traj().linVel(), traj.getX2(), Vector3::Zero(), traj().orientation.toVector4(),
                       traj().position());

  for(int i = 0; i < nbIters; i++)
  {
    traj.iterate(dt);

    viking.setInput(traj().linVel(), traj.getYa(), traj().angVel(), i);
    // viking.addPosOriMeasurement(traj().position, traj.getOri(), 1, 1, 1);
    viking.getEstimatedState(i + 1);

    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX1> x1_hat = viking.getEstimatedLocLinVel();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX2> x2_hat = viking.getEstimatedTilt();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeOri> q_hat = viking.getEstimatedOrientation();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizePos> pl_hat = viking.getEstimatedLocPosition();

    std::cout << std::endl << "Estimated 2: " << x1_hat.transpose() << std::endl;
    std::cout << std::endl << "Simulated 2: " << traj().linVel().transpose() << std::endl;

    Orientation finalOri_hat;
    finalOri_hat.fromVector4(q_hat);

    Matrix3 oriError = finalOri_hat.toMatrix3() * traj.getOri().transpose();
    Vector3 oriErrorVector = kine::skewSymmetricToRotationVector(oriError - oriError.transpose()) / 2.0;

    err = (x1_hat - traj().linVel()).squaredNorm();
    if(err > 1e-4)
    {
      std::cout << std::endl << "The local velocity estimate is incorrect." << std::endl;
      std::cout << std::endl << "Estimated: " << x1_hat.transpose() << std::endl;
      std::cout << std::endl << "Simulated: " << traj().linVel().transpose() << std::endl;
      return errorcode;
    }

    err = (x2_hat - traj.getX2()).squaredNorm();
    if(err > 1e-4)
    {
      std::cout << std::endl << "The tilt estimate is incorrect." << std::endl;
      std::cout << std::endl << "Estimated: " << x2_hat.transpose() << std::endl;
      std::cout << std::endl << "Simulated: " << traj.getX2().transpose() << std::endl;

      return errorcode;
    }

    err = (pl_hat - traj.getPos()).squaredNorm();
    if(err > 1e-4)
    {
      std::cout << std::endl << "The position estimate is incorrect." << std::endl;
      std::cout << std::endl << "Estimated: " << pl_hat.transpose() << std::endl;
      std::cout << std::endl << "Simulated: " << traj.getPos().transpose() << std::endl;
      return errorcode;
    }

    err = (oriErrorVector).squaredNorm();
    if(err > 1e-4)
    {
      std::cout << std::endl << "The orientation estimate is incorrect." << std::endl;
      std::cout << std::endl << "Error vector: " << oriErrorVector.transpose() << std::endl;
      return errorcode;
    }
  }

  return 0;
}

int main()
{
  int returnVal;
  int errorcode = 1;

  // std::cout << "Starting testWithPosMeasurement" << std::endl;
  // if((returnVal = testWithPosMeasurement(errorcode)))
  // {
  //   std::cout << "testWithPosMeasurement failed!" << errorcode << std::endl;
  //   return returnVal;
  // }
  // else
  // {
  //   std::cout << "testWithPosMeasurement succeeded" << std::endl;
  // }
  // errorcode++;

  // return 0;
  std::cout << "Starting testWithoutPosAndOriMeasurement" << std::endl;
  if((returnVal = testWithoutPosAndOriMeasurement(errorcode)))
  {
    std::cout << "testWithoutPosAndOriMeasurement failed!" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithoutPosAndOriMeasurement succeeded" << std::endl;
  }
  errorcode++;

  std::cout << "Starting testWithPosAndOriMeasurement" << std::endl;
  if((returnVal = testWithPosAndOriMeasurement(errorcode)))
  {
    std::cout << "testWithPosAndOriMeasurement failed!" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithPosAndOriMeasurement succeeded" << std::endl;
  }
  errorcode++;

  std::cout << "Test Viking succeeded" << std::endl;
  return 0;
}
