/**
 * \file      dynamics-functor-base.hpp
 * \author    Mehdi Benallegue
 * \date       2013
 * \brief      Gives the base class to be derived in order to define any
 *             dynamics system.
 *
 * \details
 *
 *
 */

#ifndef STATEOBSERVERDYNAMICALSYSTEMFUNCTORBASE_H
#define STATEOBSERVERDYNAMICALSYSTEMFUNCTORBASE_H

#include <state-observation/api.h>
#include <state-observation/tools/definitions.hpp>

namespace stateObservation
{

/**
 * \class  DynamicalSystemFunctorBase
 * \brief
 *        This is the base class of any functor that describes the dynamics
 *        of the state and the measurement.
 *        This class is to be derived in order to be given
 *        to the Extended Kalman Filter.
 *
 */

class STATE_OBSERVATION_DLLAPI DynamicalSystemFunctorBase
{
public:
  DynamicalSystemFunctorBase();
  virtual ~DynamicalSystemFunctorBase();

  /// The function to overload to describe the dynamics of the state
  virtual Vector stateDynamics(const Vector & x, const InputBase & u, TimeIndex k) = 0;

  /// The function to overload to describe the dynamics of the sensor (measurements)
  virtual Vector measureDynamics(const Vector & x, const InputBase & u, TimeIndex k) = 0;

  /// The method to overload if the functor needs to be reset when the
  /// Exteded Kalman filter is reset itself
  virtual void reset() {}

  /// gets the state size
  virtual Index getStateSize() const = 0;
  /// gets the measurements size
  virtual Index getMeasurementSize() const = 0;

  /// Gives a boolean answer on whether or not the vector is correctly sized to be a state vector
  virtual bool checkStateVector(const Vector &);

protected:
  inline void assertStateVector_(const Vector & v)
  {
    (void)v; // avoid warning
    BOOST_ASSERT(checkStateVector(v) && "ERROR: The state vector has the wrong size");
  }

private:
};

} // namespace stateObservation

#endif // STATEOBSERVERDYNAMICSYSTEMFUNCTORBASE_H
