#include <state-observation/dynamical-system/imu-dynamical-system.hpp>

#include <state-observation/tools/miscellaneous-algorithms.hpp>

namespace stateObservation
{

constexpr double IMUDynamicalSystem::one_;

IMUDynamicalSystem::IMUDynamicalSystem(bool withGyroBias)
: processNoise_(0x0), dt_(1), orientationVector_(Vector3::Zero()), quaternion_(Quaternion::Identity()),
  withGyroBias_(withGyroBias)
{
#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
  std::cout << std::endl << "IMUFixedContactDynamicalSystem Constructor" << std::endl;
#endif // STATEOBSERVATION_VERBOUS_CONSTRUCTOR
       // ctor
  sensor_.setWithGyroBias(withGyroBias);
  updatestatesize();
}

IMUDynamicalSystem::~IMUDynamicalSystem()
{
  // dtor
}

Vector IMUDynamicalSystem::stateDynamics(const Vector & x, const std::any & input, TimeIndex)
{
  assertStateVector_(x);
  const Vector & u = convert_input<Vector>(input);

  Vector3 position = x.segment<3>(indexes::pos);
  Vector3 velocity = x.segment<3>(indexes::linVel);
  Vector3 acceleration = x.segment<3>(indexes::linAcc);

  Vector3 orientationV = x.segment<3>(indexes::ori);
  Vector3 angularVelocity = x.segment<3>(indexes::angVel);
  Vector3 angularAcceleration = x.segment<3>(indexes::angAcc);

  Vector3 gyroBias;
  if(withGyroBias_)
  {
    gyroBias = x.segment<3>(indexes::gyroBias);
  }

  Quaternion orientation = computeQuaternion_(orientationV);

  kine::integrateKinematics(position, velocity, acceleration, orientation, angularVelocity, angularAcceleration, dt_);

  // x_{k+1}
  Vector xk1 = Vector::Zero(getStateSize(), 1);

  xk1.segment<3>(indexes::pos) = position;
  xk1.segment<3>(indexes::linVel) = velocity;

  AngleAxis orientationAA(orientation);

  orientationV = orientationAA.angle() * orientationAA.axis();

  xk1.segment<3>(indexes::ori) = orientationV;
  xk1.segment<3>(indexes::angVel) = angularVelocity;

  // inputs
  Vector3 linearJerkInput;
  Vector3 angularJerkInput;
  if(u.size() == 0)
  {
    linearJerkInput.setZero();
    angularJerkInput.setZero();
  }
  else
  {
    assertInputVector_(u);
    linearJerkInput = u.head<3>();
    angularJerkInput = u.tail<3>();
  }

  xk1.segment<3>(indexes::linAcc) += linearJerkInput * dt_;
  xk1.segment<3>(indexes::angAcc) += angularJerkInput * dt_;

  if(withGyroBias_)
  {
    xk1.segment<3>(indexes::gyroBias) = gyroBias * one_;
  }

  if(processNoise_ != 0x0)
    return processNoise_->getNoisy(xk1);
  else
    return xk1;
}

Quaternion IMUDynamicalSystem::computeQuaternion_(const Vector3 & x)
{
  if(orientationVector_ != x)
  {
    quaternion_ = kine::rotationVectorToAngleAxis(x);
    orientationVector_ = x;
  }

  return quaternion_;
}

Vector IMUDynamicalSystem::measureDynamics(const Vector & x, const std::any &, TimeIndex k)
{
  assertStateVector_(x);

  Vector3 acceleration = x.segment<3>(indexes::linAcc);

  Vector3 orientationV = x.segment<3>(indexes::ori);
  Vector3 angularVelocity = x.segment<3>(indexes::angVel);

  Quaternion q = computeQuaternion_(orientationV);

  Vector v;

  if(withGyroBias_)
  {
    v = Vector::Zero(13, 1);
  }
  else
  {
    v = Vector::Zero(10, 1);
  }

  v.head<4>() = q.coeffs();

  v.segment<3>(4) = acceleration;
  v.segment<3>(7) = angularVelocity;

  if(withGyroBias_)
  {
    v.segment<3>(10) = x.segment<3>(indexes::gyroBias);
  }

  sensor_.setState(v, k);

  return sensor_.getMeasurements();
}

void IMUDynamicalSystem::setProcessNoise(NoiseBase * n)
{
  processNoise_ = n;
}

void IMUDynamicalSystem::resetProcessNoise()
{
  processNoise_ = 0x0;
}

void IMUDynamicalSystem::setMeasurementNoise(NoiseBase * n)
{
  sensor_.setNoise(n);
}
void IMUDynamicalSystem::resetMeasurementNoise()
{
  sensor_.resetNoise();
}

void IMUDynamicalSystem::setSamplingPeriod(double dt)
{
  dt_ = dt;
}

Index IMUDynamicalSystem::getStateSize() const
{
  return statesize_;
}

Index IMUDynamicalSystem::getInputSize() const
{
  return inputSize_;
}

bool IMUDynamicalSystem::checkInputvector(const Vector & v)
{
  return (v.rows() == getInputSize() && v.cols() == 1);
}

Index IMUDynamicalSystem::getMeasurementSize() const
{
  return measurementSize_;
}

NoiseBase * IMUDynamicalSystem::getProcessNoise() const
{
  return processNoise_;
}

NoiseBase * IMUDynamicalSystem::getMeasurementNoise() const
{
  return sensor_.getNoise();
}

void IMUDynamicalSystem::setWithGyroBias(bool withGyroBias)
{
  withGyroBias_ = withGyroBias;
  sensor_.setWithGyroBias(withGyroBias);
  updatestatesize();
}
} // namespace stateObservation
