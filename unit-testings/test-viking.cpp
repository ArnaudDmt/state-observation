#include <iostream>

#include <state-observation/observer/viking.hpp>

using namespace stateObservation;
using namespace kine;

int testOrientationMeasurement(int errorcode)
{
  Viking viking(0.005, 1, 1, 1, 1);

  Vector3 initPosition = Vector3::Zero();
  Vector3 initX1 = Vector3::Zero();
  Orientation initOri = kine::Orientation::randomRotation();
  Vector3 initX2prime = initOri.toMatrix3().transpose() * Vector3::UnitZ();

  viking.initEstimator(initX1, initX2prime, initPosition, initOri.toVector4());

  Matrix3 measOri = kine::rotationVectorToRotationMatrix(randomAngle() * Vector3::UnitZ());

  int targetIter = 1000;
  for(int i = 0; i < targetIter; i++)
  {
    viking.setMeasurement(Vector3::Zero(), cst::gravityConstant * initX2prime, Vector3::Zero(), i + 1);
    viking.addOrientationMeasurement(measOri, 50);
    viking.getEstimatedState(i + 1);
  }

  // initOri becomes the final estimated orientation
  initOri.fromVector4(viking.getCurrentEstimatedState().tail(4));

  double err = pow(
      kine::rotationMatrixToYawAxisAgnostic(initOri.toMatrix3()) - kine::rotationMatrixToYawAxisAgnostic(measOri), 2);
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

  if((returnVal = testOrientationMeasurement(errorcode)))
  {
    std::cout << "Starting testOrientationMeasurement" << errorcode << std::endl;
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
