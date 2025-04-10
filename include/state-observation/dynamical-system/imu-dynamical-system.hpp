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

#ifndef IMU_DYNAMICAL_SYSTEM_HPP
#define IMU_DYNAMICAL_SYSTEM_HPP

#include <state-observation/api.h>
#include <state-observation/dynamical-system/dynamical-system-functor-base.hpp>
#include <state-observation/noise/noise-base.hpp>
#include <state-observation/sensors-simulation/accelerometer-gyrometer.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace stateObservation
{

/**
 * \class  IMUDynamicalSystem
 * \brief  The class is an implementation of the dynamical system defined by
 *         an inertial measurement unit (IMU) fixed on a rigid body. The state
 *         is the position velocity and acceleration and the orientaion and rotation
 *         velocity and acceleration. The sensors are the accelerometer and the gyrometer
 *
 *
 */
class STATE_OBSERVATION_DLLAPI IMUDynamicalSystem : public DynamicalSystemFunctorBase
{
public:
  /// The constructor
  IMUDynamicalSystem(bool withGyroBias = false);

  /// The virtual destructor
  virtual ~IMUDynamicalSystem();

  /// Description of the state dynamics
  virtual Vector stateDynamics(const Vector & x, const std::any & u, TimeIndex k);

  /// Description of the sensor's dynamics
  virtual Vector measureDynamics(const Vector & x, const std::any & u, TimeIndex k);

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

  /// Gets the state size
  virtual Index getStateSize() const;
  /// Gets the input size
  virtual Index getInputSize() const;
  /// Gets the measurement size
  virtual Index getMeasurementSize() const;

  /// @brief Set whether we use Gyro Bias
  void setWithGyroBias(bool);

  void updatestatesize()
  {
    if(withGyroBias_)
    {
      statesize_ = stateSizeBase_ + 3;
    }
    else
    {
      statesize_ = stateSizeBase_;
    }
  }

  struct indexes : public kine::indexes<kine::rotationVector>
  {
    static const Index gyroBias = 18;
  };

protected:
  AccelerometerGyrometer sensor_;

  NoiseBase * processNoise_;

  double dt_;

  Vector3Unaligned orientationVector_;
  QuaternionUnaligned quaternion_;

  Quaternion computeQuaternion_(const Vector3 & x);

  static const Index stateSizeBase_ = 18; /// the state size may be bigger if the bias is considered
  Index statesize_;
  static const Index inputSize_ = 6;
  static const Index measurementSize_ = 6;

  typedef Vector6 inputType;

  bool withGyroBias_;

  /// the factor that approximate the "one" to avoid drifting of unobservable values
  static constexpr double one_ = 0.9999;

private:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace stateObservation
#endif // IMU-DYNAMICAL-SYSTEM_HPP
