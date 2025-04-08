/**
 * \file      tilt-estimator.hpp
 * \author    Arnaud Demont, Mehdi Benallegue
 * \date       2018
 * \brief      Version of the Tilt Estimator that implements all the necessary functions to perform the estimation for
 * humanoid robots.
 *
 * \details
 *
 *
 */

#ifndef DelayedMeasurementIterHPP
#define DelayedMeasurementIterHPP

#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace stateObservation
{

struct Iteration
{
  Iteration(const Vector & initState, double dt) : dt_(dt), initState_(initState) {}

  /// Default constructor
  Iteration() = delete;

  /// Default destructor
  virtual ~Iteration(){};

  virtual Vector & runIteration_() = 0;

  inline double getSamplingTime()
  {
    return dt_;
  }

  /// Iteration's sampling time
  double dt_;
  // state at time k-1
  Vector initState_;
  // updated state (at the end of the iteration)
  Vector finalState_;
  /// Container for the measurements.
  Vector y_;
};

struct IterationComplementaryFilter : public Iteration
{
  IterationComplementaryFilter(const Vector & initState, double dt) : Iteration(initState, dt) {}

  /// Default constructor
  IterationComplementaryFilter() = delete;

  /// Default destructor
  virtual ~IterationComplementaryFilter(){};

  virtual Vector & runIteration_() override = 0;

  virtual Vector computeStateDerivatives_() = 0;
  /// @brief integrates the given dx into the given state.
  /// @param dx_hat The state increment to integrate
  virtual void integrateState_(const Vector & dx_hat) = 0;
};

} // namespace stateObservation

#endif // DelayedMeasurementIterHPP
