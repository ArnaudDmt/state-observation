#include <iostream>

#include <state-observation/observer/viking.hpp>

using namespace stateObservation;
using namespace kine;

int testWithoutPosAndOriMeasurement(int)
{
  Viking viking(0.005, 1, 1, 1, 1);

  Vector3 initPosition = Vector3::Zero();
  Orientation initOri = kine::Orientation::randomRotation();
  Vector3 initX1 = Vector3::Zero();
  Vector3 initX2prime = initOri.toMatrix3().transpose() * Vector3::UnitZ();
  Vector3 initBias = Vector3::Zero();

  viking.initEstimator(initX1, initX2prime, initBias, initOri.toVector4(), initPosition);

  int targetIter = 1000;
  for(int i = 0; i < targetIter; i++)
  {
    viking.setInput(Vector3::Zero(), cst::gravityConstant * initX2prime, Vector3::Zero(), i + 1);
    viking.getEstimatedState(i + 1);
  }

  return 0;
}

int testWithPosAndOriMeasurement(int errorcode)
{
  Viking viking(0.005, 1, 1, 1, 1);

  Vector3 initPosition = Vector3::Zero();
  Orientation initOri = kine::Orientation::randomRotation();
  Vector3 initX1 = Vector3::Zero();
  Vector3 initX2prime = initOri.toMatrix3().transpose() * Vector3::UnitZ();
  Vector3 initBias = Vector3::Zero();

  viking.initEstimator(initX1, initX2prime, initBias, initOri.toVector4(), initPosition);

  Matrix3 oriMeas = kine::rotationVectorToRotationMatrix(randomAngle() * Vector3::UnitZ());
  Vector3 posMeas = Vector3::Random();

  int targetIter = 1000;
  for(int i = 0; i < targetIter; i++)
  {
    viking.setInput(Vector3::Zero(), cst::gravityConstant * initX2prime, Vector3::Zero(), i);
    viking.addPosOriMeasurement(posMeas, oriMeas, 50, 50);
    viking.getEstimatedState(i + 1);
  }

  // initOri becomes the final estimated orientation
  initOri.fromVector4(viking.getCurrentEstimatedState().segment<4>(9));
  // initPos becomes the final estimated orientation
  initPosition = viking.getCurrentEstimatedState().tail(3);

  double err = pow(
      kine::rotationMatrixToYawAxisAgnostic(initOri.toMatrix3()) - kine::rotationMatrixToYawAxisAgnostic(oriMeas), 2);
  err += pow((posMeas - initPosition).norm(), 2);
  if(err > 1e-15)
  {
    std::cout << std::endl << "The correction from the orientation measurement doesn't work." << std::endl;
    return errorcode;
  }

  return 0;
}

int main()
{
  int returnVal;
  int errorcode = 1;

  std::cout << "Starting testOrientationMeasurement" << std::endl;
  if((returnVal = testWithPosAndOriMeasurement(errorcode)))
  {
    std::cout << "testOrientationMeasurement failed!" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testOrientationMeasurement succeeded" << std::endl;
  }
  errorcode++;

  std::cout << "Test Viking succeeded" << std::endl;
  return 0;
}
