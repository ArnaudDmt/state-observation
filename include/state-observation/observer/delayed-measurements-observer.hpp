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
  ObserverBase::StateVector initState_;
  // updated state (at the end of the iteration)
  ObserverBase::StateVector finalState_;
  /// Container for the measurements.
  Vector y_;
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
  DelayedMeasurementObserver(double dt, Index n, Index m, Index p = 0);

  /// The constructor
  ///  \li n : size of the state vector
  ///  \li m : size of the measurements vector
  ///  \li p : size of the input vector
  ///  \li dt  : timestep between each iteration
  ///  \li bufferCapacity  : capacity of the iteration buffer
  DelayedMeasurementObserver(double dt, Index n, Index m, unsigned long bufferCapacity, Index p = 0);

  /// Default constructor
  DelayedMeasurementObserver() = delete;

  /// Destructor
  virtual ~DelayedMeasurementObserver(){};

  inline IterationT & getCurrentIter()
  {
    return bufferedIters_.front();
  }
  /// @brief initializes the state vector.
  /// @param x The initial state vector
  virtual void initEstimator(const Vector & x);

  /// @brief Set the value of the state vector at time index k.
  ///
  /// @details This replaces the current state estimate. If k < current time then the measurements and the inputs
  /// are also cleared. Otherwise only past measurements and inputs are removed.
  ///
  /// @param x_k
  /// @param k
  virtual void setState(const ObserverBase::StateVector & x_k, TimeIndex k) override;

  /// @brief getestimated State
  /// @param k The time index of the expected state value
  /// @return ObserverBase::StateVector
  virtual ObserverBase::StateVector getEstimatedState(TimeIndex k) override;

  /// @brief Get the Current Estimated State
  /// @return ObserverBase::StateVector
  virtual ObserverBase::StateVector getCurrentEstimatedState() const;

  /// @brief sets the measurement
  /// @param y the measurement vector
  /// @param k the time index
  void setMeasurement(const Vector & y, TimeIndex k) override;

  /// Get the measurement of the time index k
  Vector getMeasurement(TimeIndex k) const;

  /// Get the time index of the last given measurement
  virtual TimeIndex getMeasurementTime() const;

  /// Remove all the given values of the measurements
  virtual void clearMeasurements() override;

  /// Set the value of the input vector at time index k. The
  /// inputs have to be inserted in chronological order without gaps.
  /// If there is no input in the system (p==0), this instruction has no effect
  virtual void setInput(const ObserverBase::InputVector & u_k, TimeIndex k) override;

  /// Remove all the given values of the inputs
  /// If there is no input, this instruction has no effect
  virtual void clearInputs() override;

  /// @brief Modify the value of the state vector at the current time.
  ///
  /// @param x_k The new state value
  ///
  /// This method should NOT be used for first initialization
  /// Use setState() instead.
  ///
  /// Calling this function will not affect the measurements nor the input vectors. It will only replace the current
  /// state/estimate with a new one
  void setCurrentState(const ObserverBase::StateVector & x_k);

  /// @brief  Removes the state estimation
  /// @details inherited from ObserverBase
  virtual void clearStates() override;

  virtual void startNewIteration_() = 0;

  /// set the sampling time of the measurements
  inline virtual void setBufferCapacity(unsigned long bufferCapacity)
  {
    bufferedIters_.set_capacity(bufferCapacity);
  }

  /// set the sampling time of the measurements
  inline void setSamplingTime(const double dt)
  {
    dt_ = dt;
  }
  inline double getSamplingTime()
  {
    return dt_;
  }

  /// Get the value of the time index of the current state estimation
  virtual TimeIndex getCurrentTime() const;

protected:
  virtual StateVector oneStepEstimation_() = 0;

  inline const boost::circular_buffer<std::unique_ptr<IterationT>> & getIterationsBuffer() const
  {
    return bufferedIters_;
  }

  bool stateIsSet() const;

protected:
  /// Sampling time
  double dt_;
  /// The state estimation of the observer (only one state is recorded)
  IndexedVector x_;
  /// Container for the measurements.
  IndexedVectorArray y_;
  /// Container for the inputs.
  IndexedVectorArray u_;

  TimeIndex k_est_ = 0; // time index of the last estimation
  TimeIndex k_data_ = 0; // time index of the current measurements

  boost::circular_buffer<IterationT> bufferedIters_;
};

} // namespace stateObservation

#include <state-observation/observer/delayed-measurements-observer.hxx>

#endif // DelayedMeasurementObserverHPP
