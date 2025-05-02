#include "state-observation/observer/tilt-estimator.hpp"
#include "state-observation/tools/definitions.hpp"
#include "state-observation/tools/rigid-body-kinematics.hpp"
#include <iostream>

using namespace stateObservation;
using namespace kine;

struct Traj
{
  Traj() : kine(LocalKinematics::zeroKinematics(kine::LocalKinematics::Flags::all)) {}

  void iterate(double dt, bool inMotion = true)
  {
    Vector3 linJerk = tools::ProbabilityLawSimulation::getUniformMatrix(3, 1, -0.2, 0.2) * 1;
    Vector3 angJerk = tools::ProbabilityLawSimulation::getUniformMatrix(3, 1, -0.2, 0.2) * 1;

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

int testTiltObserver(int errorcode)
{
  double simTime = 10.00;
  double dt = 0.005;
  int nbIters = int(simTime / dt);

  double err;
  TiltEstimator tilt(5, 2, 2, dt);
  Traj traj;

  Vector3 x2_init = traj.getX2();
  tilt.initEstimator(traj().linVel(), x2_init, x2_init);

  for(int i = 0; i < nbIters; i++)
  {
    traj.iterate(dt);

    tilt.setMeasurement(traj().linVel(), traj.getYa(), traj().angVel(), i + 1);
    tilt.getEstimatedState(i + 1);

    err = (tilt.getCurrentEstimatedState().segment<3>(0) - traj().linVel()).squaredNorm();
    if(err > 1e-4)
    {
      std::cout << std::endl << "The local velocity estimate is incorrect." << std::endl;
      std::cout << std::endl << "Estimated: " << tilt.getCurrentEstimatedState().segment<3>(0).transpose() << std::endl;
      std::cout << std::endl << "Simulated: " << traj().linVel().transpose() << std::endl;
      return errorcode;
    }

    err = (tilt.getCurrentEstimatedState().segment<3>(6) - traj.getX2()).squaredNorm();
    if(err > 1e-4)
    {
      std::cout << std::endl << "The tilt estimate is incorrect." << std::endl;
      std::cout << std::endl << "Estimated: " << tilt.getCurrentEstimatedState().segment<3>(6).transpose() << std::endl;
      std::cout << std::endl << "Simulated: " << traj.getX2().transpose() << std::endl;

      return errorcode;
    }
  }

  return 0;
}

int main()
{
  int returnVal;
  int errorcode = 1;

  std::cout << "Starting testTiltObserver" << std::endl;
  if((returnVal = testTiltObserver(errorcode)))
  {
    std::cout << "testTiltObserver failed!" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testTiltObserver succeeded" << std::endl;
  }
  errorcode++;

  std::cout << "Test Tilt Observer succeeded" << std::endl;
  return 0;
}
