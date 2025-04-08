#include <Eigen/Eigenvalues>
#include <iostream>

#include <state-observation/observer/viking.hpp>

using namespace stateObservation;
using namespace kine;

// case we estimate the new state as soon as the new measurement is given
int testVikingRealTimeEstimation(int errorcode)
{
  double err = 0.0;
  Viking viking(0.005, 1, 1, 1, 1);

  Vector3 initPosition = Vector3::Zero();
  Vector3 initX1 = Vector3::Zero();
  Matrix3 initRmat;
  initRmat << 0, 0, 1, 0, 1, 1, -1, 0, 0; // rotation by 90 degrees around y
  Orientation initOri(initRmat);
  Vector3 initX2prime = initOri.toMatrix3().transpose() * Vector3::UnitZ();

  Vector initState(initX1.size() + initX2prime.size() + initPosition.size() + 4); // 4 for quaternion

  initState.segment(0, initX1.size()) = initX1;
  initState.segment(initX1.size(), initX2prime.size()) = initX2prime;
  initState.segment(initX1.size() + initX2prime.size(), initPosition.size()) = initPosition;
  initState.tail(4) = initOri.toVector4(); // Assuming .toVector4() returns Eigen::Vector4d

  viking.initEstimator(initState);

  for(int i = 0; i < 50; i++)
  {
    TimeIndex k = viking.getCurrentTime();
    viking.setMeasurement(Vector3::Zero(), cst::gravityConstant * initX2prime, Vector3::Zero(), k + 1);
    viking.getEstimatedState(k + 1);
  }

  err = (viking.getCurrentEstimatedState() - initState).norm();
  if(err > 1e-15)
  {
    std::cout << std::endl << "The obtained vector does not match the initial one." << std::endl;
    return errorcode;
  }

  return 0;
}

int testVikingAwaitingEstimation(int errorcode)
{
  Viking viking(0.005, 1, 1, 1, 1);

  Vector3 initPosition = Vector3::Zero();
  Vector3 initX1 = Vector3::Zero();
  Matrix3 initRmat;
  initRmat << 0, 0, 1, 0, 1, 1, -1, 0, 0; // rotation by 90 degrees around y
  Orientation initOri(initRmat);
  Vector3 initX2prime = initOri.toMatrix3().transpose() * Vector3::UnitZ();

  viking.initEstimator(initX1, initX2prime, initPosition, initOri.toVector4());

  int targetIter = 50;
  for(int i = 0; i < targetIter; i++)
  {
    viking.setMeasurement(Vector3::Zero(), -cst::gravity, Vector3::Zero(), i + 1);
  }

  viking.getEstimatedState(targetIter);

  return 0;
}

int main()
{
  int returnVal;
  int errorcode = 1;

  std::cout << "Starting testVikingRealTimeEstimation" << errorcode << std::endl;
  if((returnVal = testVikingRealTimeEstimation(errorcode)))
  {
    std::cout << "testVikingRealTimeEstimation failed, code : " << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testVikingRealTimeEstimation succeeded" << std::endl;
  }

  errorcode++;

  if((returnVal = testVikingAwaitingEstimation(errorcode)))
  {
    std::cout << "Starting testVikingAwaitingEstimation" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testVikingAwaitingEstimation succeeded" << std::endl;
  }

  std::cout << "Test Viking succeeded" << std::endl;
  return 0;
}
