/**
 * \file      imu-dynamical-system.hpp
 * \author    Mehdi Benallegue
 * \date       2013
 * \brief   The file describes the dynamical system defined by an inertial
 *          measurement unit (IMU) fixed on a rigid body.
 *
 * \details
 *
 *
 */

#ifndef IMU_MULTIPLICATIVE_DYNAMICAL_SYSTEM_HPP
#define IMU_MULTIPLICATIVE_DYNAMICAL_SYSTEM_HPP

#include <state-observation/api.h>
#include <state-observation/dynamical-system/dynamical-system-functor-base.hpp>
#include <state-observation/noise/noise-base.hpp>
#include <state-observation/sensors-simulation/accelerometer-gyrometer.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>
#include <state-observation/tools/state-vector-arithmetics.hpp>

namespace stateObservation
{

/**
 * \class  IMUMltpctiveDynamicalSystem
 * \brief  The class is an implementation of the dynamical system defined by
 *         an inertial measurement unit (IMU) fixed on a rigid body. The state
 *         is the position velocity and acceleration and the orientaion and rotation
 *         velocity and acceleration. The sensors are the accelerometer and the gyrometer
 *
 *
 */
class STATE_OBSERVATION_DLLAPI IMUMltpctiveDynamicalSystem : public DynamicalSystemFunctorBase,
                                                             public StateVectorArithmetics
{
public:
  /// The constructor
  IMUMltpctiveDynamicalSystem();

  /// The virtual destructor
  virtual ~IMUMltpctiveDynamicalSystem();

  /// Description of the state dynamics
  virtual Vector stateDynamics(const Vector & x, const InputBase & u, TimeIndex k);

  /// Description of the sensor's dynamics
  virtual Vector measureDynamics(const Vector & x, const InputBase & u, TimeIndex k);

  /// Sets a noise which disturbs the state dynamics
  virtual void setProcessNoise(NoiseBase *);
  /// Removes the process noise
  virtual void resetProcessNoise();
  /// Gets the process noise
  virtual NoiseBase * getProcessNoise() const;

  /// Sets a noise which disturbs the measurements
  virtual void setMeasurementNoise(NoiseBase *);
  /// Removes the measurement noise
  virtual void resetMeasurementNoise();
  /// Gets a pointer on the measurement noise
  virtual NoiseBase * getMeasurementNoise() const;

  /// Set the period of the time discretization
  virtual void setSamplingPeriod(double dt);

  virtual Matrix getAMatrix(const Vector & xh);
  virtual Matrix getCMatrix(const Vector & xp);

  /// Gets the state size
  virtual Index getStateSize() const;
  /// Gets the input size
  virtual Index getInputSize() const;
  /// Gets the measurement size
  virtual Index getMeasurementSize() const;

  void stateSum(const Vector & stateVector, const Vector & tangentVector, Vector & sum);

  void stateDifference(const Vector & stateVector1, const Vector & stateVector2, Vector & difference);

protected:
  /// Gives a boolean answer on whether or not the vector is correctly sized to be an input vector
  virtual bool checkInputvector(const Vector &);

  inline void assertInputVector_(const Vector & v)
  {
    (void)v; // avoid warning
    BOOST_ASSERT(checkInputvector(v) && "ERROR: The input vector has the wrong size");
  }

  static const Index stateSize_ = 19;
  static const Index stateTangentSize_ = 18;
  static const Index inputSize_ = 6;
  static const Index measurementSize_ = 6;
  typedef kine::indexes<kine::quaternion> indexes;
  typedef kine::indexes<kine::rotationVector> indexesTangent;

  struct opt
  {
    /// containers for Jacobians
    Matrix3 jRR, jRv;

    Vector3 deltaR;

    Matrix AJacobian;
    Matrix CJacobian;

    Matrix3 Rt;

    opt(int stateSize, int measurementSize) : AJacobian(stateSize, stateSize), CJacobian(measurementSize, stateSize)
    {
      AJacobian.setZero();
      AJacobian.block<3, 3>(indexesTangent::pos, indexesTangent::pos).setIdentity();
      AJacobian.block<6, 6>(indexesTangent::linVel, indexesTangent::linVel).setIdentity();

      CJacobian.setZero();
    }
  } opt_;

  AccelerometerGyrometer sensor_;

  NoiseBase * processNoise_;

  double dt_;

private:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace stateObservation
#endif // IMU_MULTIPLICATIVE_DYNAMICAL_SYSTEM_HPP
