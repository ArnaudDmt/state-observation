#include "state-observation/tools/definitions.hpp"
#include "state-observation/tools/rigid-body-kinematics.hpp"
#include <iostream>
#include <state-observation/tools/odometry/legged-odometry-manager.hpp>

using namespace stateObservation;
using namespace kine;
// using namespace stateObservation::odometry;
// using LeggedOdometryManager = stateObservation::odometry::LeggedOdometryManager;

struct Traj
{
  Traj() : kine(LocalKinematics::zeroKinematics(kine::LocalKinematics::Flags::all))
  {
    kine.position = Vector3(0.0, 0.0, 0.8);
  }

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

int testLeggedOdometry(int errorcode)
{
  double simTime = 10.00;
  double dt = 0.005;
  int nbIters = int(simTime / dt);
  Traj traj;

  stateObservation::odometry::LeggedOdometryManager odometryManager_(dt); // manager for the legged odometry

  odometry::LeggedOdometryManager::Configuration odomConfig(measurements::stringToOdometryType("6D"));
  odometryManager_.init(odomConfig, traj.kine.toVector(Kinematics::Flags::pose));

  Kinematics kine;
  kine.position = Vector3(0.0, 0.0, 0.8);
  kine.orientation.setZeroRotation();
  for(int i = 0; i < nbIters; i++)
  {
    traj.iterate(dt);
    Kinematics zeroKine = Kinematics::zeroKinematics(Kinematics::Flags::pose);
    stateObservation::odometry::LeggedOdometryManager::ContactInputData test(zeroKine, 1.0);

    std::unordered_set<std::string> contactList;
    if(i % 2 == 0)
    {
      contactList.insert("Contact1");
    }
    else
    {
      contactList.insert("Contact2");
    }

    odometryManager_.initLoop(contactList,
                              stateObservation::odometry::LeggedOdometryManager::ContactUpdateFunctions<>());

    odometryManager_.run(
        odometry::LeggedOdometryManager::KineParams(kine).attitudeMeas(traj.kine.orientation.toMatrix3()));
  }

  return 0;
}

int main()
{
  int returnVal;
  int errorcode = 1;

  std::cout << "Starting testLeggedOdometry" << std::endl;
  if((returnVal = testLeggedOdometry(errorcode)))
  {
    std::cout << "testLeggedOdometry failed!" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testLeggedOdometry succeeded" << std::endl;
  }
  errorcode++;

  std::cout << "Test Legged Odometry succeeded" << std::endl;
  return 0;
}
