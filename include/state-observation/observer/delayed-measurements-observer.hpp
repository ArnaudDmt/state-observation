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

#ifndef DelayedMeasurementObserverHPP
#define DelayedMeasurementObserverHPP

#include <boost/circular_buffer.hpp>
#include <state-observation/api.h>
#include <state-observation/observer/observer-base.hpp>

namespace stateObservation
{

struct Iteration
{
  Iteration(const Vector & initState, double dt) : dt_(dt), initState_(initState) {}

  virtual Vector runIteration_() = 0;
  virtual void startNewIteration_() = 0;

  /// Iteration's sampling time
  double dt_;

  // state at time k-1
  ObserverBase::StateVector initState_;
  // updated state (at the end of the iteration)
  ObserverBase::StateVector finalState_;
};

/**
 * \class  DelayedMeasurementObserver
 * \brief
 *
 */
template<typename IterationT>
class STATE_OBSERVATION_DLLAPI DelayedMeasurementObserver : public ObserverBase
{
public:
  /// The constructor
  ///  \li n : size of the state vector
  ///  \li m : size of the measurements vector
  ///  \li dt  : timestep between each iteration
  ///  \li p : size of the input vector
  DelayedMeasurementObserver(Index n, Index m, double dt, Index p = 0);

  /// The constructor
  ///  \li n : size of the state vector
  ///  \li m : size of the measurements vector
  ///  \li p : size of the input vector
  ///  \li dt  : timestep between each iteration
  ///  \li bufferCapacity  : capacity of the iteration buffer
  DelayedMeasurementObserver(Index n, Index m, double dt, unsigned long bufferCapacity, Index p = 0);

  inline IterationT & getCurrentIter()
  {
    return *bufferedIters_.front();
  }
  /// @brief initializes the state vector.
  /// @param x The initial state vector
  virtual void initEstimator(const Vector & x);

  /// @brief sets the measurement
  /// @param y the measurement vector
  /// @param k the time index
  void setMeasurement(const Vector & y, TimeIndex k) override;

  /// set the sampling time of the measurements
  void setBufferCapacity(unsigned long bufferCapacity)
  {
    bufferedIters_.set_capacity(bufferCapacity);
  }

  /// set the sampling time of the measurements
  void setSamplingTime(const double dt)
  {
    dt_ = dt;
  }
  double getSamplingTime()
  {
    return dt_;
  }

protected:
  virtual StateVector oneStepEstimation_() = 0;

  inline const boost::circular_buffer<std::unique_ptr<IterationT>> & getIterationsBuffer() const
  {
    return bufferedIters_;
  }

protected:
  /// Sampling time
  double dt_;
  /// The state estimation of the observer (only one state is recorded)
  Vector x_;
  /// Container for the measurements.
  Vector y_;
  /// Container for the inputs.
  Vector u_;

  TimeIndex k_est_ = 0; // time index of the last estimation
  TimeIndex k_data_ = 0; // time index of the current measurements

  boost::circular_buffer<std::unique_ptr<IterationT>> bufferedIters_;
};

} // namespace stateObservation

#include <state-observation/observer/delayed-measurements-observer.hxx>

#endif // DelayedMeasurementObserverHPP
