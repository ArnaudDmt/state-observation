#include <state-observation/flexibility-estimation/imu-fixed-contact-dynamical-system.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

namespace stateObservation
{
namespace flexibilityEstimation
{
using namespace stateObservation;

IMUFixedContactDynamicalSystem::IMUFixedContactDynamicalSystem(double dt)
: processNoise_(0x0), dt_(dt), orientationVector_(Vector3::Zero()), quaternion_(Quaternion::Identity()),
  measurementSize_(measurementSizeBase_)
{
#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
  std::cout << std::endl << "IMUFixedContactDynamicalSystem Constructor" << std::endl;
#endif // STATEOBSERVATION_VERBOUS_CONSTRUCTOR
}

IMUFixedContactDynamicalSystem::~IMUFixedContactDynamicalSystem()
{
  // dtor
}

Vector IMUFixedContactDynamicalSystem::stateDynamics(const Vector & x, const InputBase &, TimeIndex)
{
  assertStateVector_(x);

  Vector3 positionFlex(x.segment(indexes::pos, 3));
  Vector3 velocityFlex(x.segment(indexes::linVel, 3));
  Vector3 accelerationFlex(x.segment(indexes::linAcc, 3));

  Vector3 orientationFlexV(x.segment(indexes::ori, 3));
  Vector3 angularVelocityFlex(x.segment(indexes::angVel, 3));
  Vector3 angularAccelerationFlex(x.segment(indexes::angAcc, 3));

  Quaternion orientationFlex(computeQuaternion_(orientationFlexV));

  kine::integrateKinematics(positionFlex, velocityFlex, accelerationFlex, orientationFlex, angularVelocityFlex,
                            angularAccelerationFlex, dt_);

  // x_{k+1}
  Vector xk1(x);

  xk1.segment(indexes::pos, 3) = positionFlex;
  xk1.segment(indexes::linVel, 3) = velocityFlex;

  AngleAxis orientationAA(orientationFlex);
  orientationFlexV = orientationAA.angle() * orientationAA.axis();

  xk1.segment(indexes::ori, 3) = orientationFlexV;
  xk1.segment(indexes::angVel, 3) = angularVelocityFlex;

  if(processNoise_ != 0x0)
    return processNoise_->getNoisy(xk1);
  else
    return xk1;
}

Quaternion IMUFixedContactDynamicalSystem::computeQuaternion_(const Vector3 & x)
{
  if(orientationVector_ != x)
  {
    orientationVector_ = x;
    quaternion_ = kine::rotationVectorToAngleAxis(x);
  }

  return quaternion_;
}

Vector IMUFixedContactDynamicalSystem::measureDynamics(const Vector & x, const InputBase & input, TimeIndex k)
{
  assertStateVector_(x);

  Vector3 positionFlex(x.segment(indexes::pos, 3));
  Vector3 velocityFlex(x.segment(indexes::linVel, 3));
  Vector3 accelerationFlex(x.segment(indexes::linAcc, 3));

  Vector3 orientationFlexV(x.segment(indexes::ori, 3));
  Vector3 angularVelocityFlex(x.segment(indexes::angVel, 3));
  Vector3 angularAccelerationFlex(x.segment(indexes::angAcc, 3));

  Quaternion qFlex(computeQuaternion_(orientationFlexV));
  Matrix3 rFlex(qFlex.toRotationMatrix());

  const VectorInput & u = convert_input<VectorInput>(input);
  assertInputVector_(u);

  Vector3 positionControl(u.segment(indexes::pos, 3));
  Vector3 velocityControl(u.segment(indexes::linVel, 3));
  Vector3 accelerationControl(u.segment(indexes::linAcc, 3));

  Vector3 orientationControlV(u.segment(indexes::ori, 3));
  Vector3 angularVelocityControl(u.segment(indexes::angVel, 3));

  Quaternion qControl(computeQuaternion_(orientationControlV));

  Quaternion q = qFlex * qControl;

  Vector3 acceleration(
      (kine::skewSymmetric(angularAccelerationFlex) + tools::square(kine::skewSymmetric(angularVelocityFlex))) * rFlex
          * positionControl
      + 2 * kine::skewSymmetric(angularVelocityFlex) * rFlex * velocityControl + accelerationFlex
      + rFlex * accelerationControl);

  Vector3 angularVelocity(angularVelocityFlex + rFlex * angularVelocityControl);
  Vector v(Vector::Zero(10, 1));

  v.head<4>() = q.coeffs();

  v.segment(4, 3) = acceleration;
  v.tail(3) = angularVelocity;

  sensor_.setState(v, k);

  Vector y(Matrix::Zero(measurementSize_, 1));

  y.head(sensor_.getMeasurementSize()) = sensor_.getMeasurements();

  for(unsigned i = 0; i < contactPositions_.size(); ++i)
  {
    y.segment(sensor_.getMeasurementSize() + i * 3, 3) =
        rFlex * contactPositions_[i] + positionFlex - contactPositions_[i];
  }

  return y;
}

void IMUFixedContactDynamicalSystem::setProcessNoise(NoiseBase * n)
{
  processNoise_ = n;
}

void IMUFixedContactDynamicalSystem::resetProcessNoise()
{
  processNoise_ = 0x0;
}

void IMUFixedContactDynamicalSystem::setMeasurementNoise(NoiseBase * n)
{
  sensor_.setNoise(n);
}

void IMUFixedContactDynamicalSystem::resetMeasurementNoise()
{
  sensor_.resetNoise();
}

void IMUFixedContactDynamicalSystem::setSamplingPeriod(double dt)
{
  dt_ = dt;
}

Index IMUFixedContactDynamicalSystem::getStateSize() const
{
  return stateSize_;
}

Index IMUFixedContactDynamicalSystem::getInputSize() const
{
  return inputSize_;
}

bool IMUFixedContactDynamicalSystem::checkInputvector(const Vector & v)
{
  return (v.rows() == getInputSize() && v.cols() == 1);
}

Index IMUFixedContactDynamicalSystem::getMeasurementSize() const
{
  return measurementSize_;
}

NoiseBase * IMUFixedContactDynamicalSystem::getProcessNoise() const
{
  return processNoise_;
}

NoiseBase * IMUFixedContactDynamicalSystem::getMeasurementNoise() const
{
  return sensor_.getNoise();
}

void IMUFixedContactDynamicalSystem::setContactsNumber(unsigned i)
{
  measurementSize_ = measurementSizeBase_ + 3 * i;
  contactPositions_.resize(i, Vector3::Zero());
}

void IMUFixedContactDynamicalSystem::setContactPosition(unsigned i, const Vector3 & position)
{
  BOOST_ASSERT(i < contactPositions_.size() && "ERROR: The index of contact is out of range.");

  contactPositions_[i] = position;
}
} // namespace flexibilityEstimation
} // namespace stateObservation
