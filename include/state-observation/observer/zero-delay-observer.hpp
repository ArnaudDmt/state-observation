/**
 * \file      zero-delay-observer.hpp
 * \author    Mehdi Benallegue
 * \date       2012
 * \brief      Defines the base class of online zero delay observers.
 *             Zero delay observers are the classical state observers where
 *             input and state values at instant k and the measurement value
 *             at instant k+1 are enough to provide the estimation of the state
 *             at instant k+1.
 *
 * \details
 *
 *
 */

#ifndef ZERODELAYOBSERVER_H
#define ZERODELAYOBSERVER_H

#include <deque>

#include <state-observation/api.h>
#include <state-observation/observer/observer-base.hpp>

namespace stateObservation
{
/**
 * \class  ZeroDelayObserver
 * \brief  Defines the base class of online zero delay observers.
 *         Zero delay observers are the classical state observers where
 *         input and state values at instant k and the measurement value
 *         at instant k+1 are enough to provide the estimation of the state
 *         at instant k+1.
 *         This class mostly defines the data structures for storing the
 *         vectors, it describes the set routines and the observation
 *         loop mechanism. It requires to be derviated to implement the
 *         new oneStepEstimation_() method
 *

 *
 * \details
 *
 */
class STATE_OBSERVATION_DLLAPI ZeroDelayObserver : public ObserverBase
{
public:
  /// The constructor
  ///  \li n : size of the state vector
  ///  \li m : size of the measurements vector
  ///  \li p : size of the input vector
  ZeroDelayObserver(Index n, Index m, const std::shared_ptr<IndexedInputArrayInterface> input) : ObserverBase(n, m)
  {
    if(input)
    {
      u_ = input;
    }
    else
    {
      u_ = std::make_shared<IndexedInputArrayT<>>();
    }
  }

  /// Default constructor (default values for n,m are zero)
  ZeroDelayObserver(const std::shared_ptr<IndexedInputArrayInterface> input)
  {
    if(input == nullptr)
    {
      u_ = std::make_shared<IndexedInputArrayT<>>();
    }
    else
    {
      u_ = input;
    }
  }

  /// Destructor
  virtual ~ZeroDelayObserver(){};

  /// @brief Set the value of the state vector at time index k.
  ///
  /// @details This replaces the current state estimate. If k < current time then the measurements and the inputs
  /// are also cleared. Otherwise only past measurements and inputs are removed.
  ///
  /// @param x_k
  /// @param k
  virtual void setState(const ObserverBase::StateVector & x_k, TimeIndex k);

  /// @brief Modify the value of the state vector at the current time.
  ///
  /// @param x_k The new state value
  ///
  /// This method should NOT be used for first initialization
  /// Use setState() instead.
  ///
  /// Calling this function will not affect the measurements nor the input vectors. It will only replace the current
  /// state/estimate with a new one
  virtual void setCurrentState(const ObserverBase::StateVector & x_k);

  /// @brief  Removes the state estimation
  /// @details inherited from ObserverBase
  virtual void clearStates();

  /// @brief Returns if the state is set or not. The state being set is mandatory to start the estimator
  ///
  /// @return true is set
  /// @return false is not set
  bool stateIsSet() const;

  /// Set the value of the measurements vector at time index k. The
  /// measurements have to be inserted in chronological order without gaps.
  virtual void setMeasurement(const ObserverBase::MeasureVector & y_k, TimeIndex k);

  /// @brief Sets the measurement value at the next time index
  ///
  /// @param y_k Value of the next measurement
  virtual void pushMeasurement(const ObserverBase::MeasureVector & y_k);

  /// Remove all the given values of the measurements
  virtual void clearMeasurements();

  /// Set the value of the input vector at time index k. The
  /// inputs have to be inserted in chronological order without gaps.
  virtual void setInput(const InputBase & u_k, TimeIndex k);

  /// @brief Set the input value at the next time indext
  ///
  /// @param u_k Value of the next input
  virtual void pushInput(const InputBase & u_k);

  /// Remove all the given values of the inputs
  /// If there is no input, this instruction has no effect
  virtual void clearInputs();

  /// @brief  Remove all the given values of the inputs and measurements
  ///
  virtual void clearInputsAndMeasurements();

  /// @brief run the observer until the measurement vector is depleted.
  ///
  /// @details the observer has to be able to reconstruct all the state values from the current time to the last
  /// measurement time. That means all the input values reauired have to be provided before.
  ///
  /// @return the current time
  virtual TimeIndex estimateState();

  /// @brief getestimated State
  ///
  /// @param k The time index of the expected state value
  /// @return ObserverBase::StateVector
  ///
  /// @details If k is equal to the current time k_0, this will give the value of the last state/estimate.
  ///
  /// If k is larger than the current time k_0, this will run the observer loop and get the state estimation of the
  /// state at instant k.
  ///
  /// In order to estimate the state k, two conditions have to be met:
  /// \li the time index k must be superior or equal to the current time k_0,
  ///     the estimator does *not* record past values of the state and cannot observe
  ///     past states.
  /// \li the observer has to be able to reconstruct all the state
  ///     values from k_0 to k. That means all the measurements or input
  ///     values reauired have to be provided before.
  ///
  /// That means generally (for most zero delay observers) that when
  /// current time is k_0 (we know an estimation of x_{k_0}) and we want
  /// to reconstruct the state at time k>k_0 we need to have the values of
  /// y_{k_0+1} to y_{k} and u_{k_0} to u_{k-1} (or u_{k} depending on the measure dynamics)
  ///
  /// This method sets the current time to k
  virtual const ObserverBase::StateVector & getEstimatedState(TimeIndex k);

  /// @brief Get the Current Estimated State
  /// @return ObserverBase::StateVector
  virtual const ObserverBase::StateVector & getCurrentEstimatedState() const;

  /// Get the value of the time index of the current state estimation
  virtual TimeIndex getCurrentTime() const;

  /// Get the value of the input of the time index k
  const InputBase & getInput(TimeIndex k) const;

  /// Get the number of available inputs
  virtual TimeSize getInputsNumber() const;

  /// Get the time index of the last given input
  virtual TimeIndex getInputTime() const;

  /// Get the measurement of the time index k
  const Vector & getMeasurement(TimeIndex k) const;

  /// Get the time index of the last given measurement
  virtual TimeIndex getMeasurementTime() const;

  /// Gets the number of regitered measurements
  virtual TimeSize getMeasurementsNumber() const;

  /// changes the size of the state vector: resets the stored state vector
  virtual void setStateSize(Index n);

  /// changes the size of the measurement vector: reset the stored measurement vectors
  virtual void setMeasureSize(Index m);

protected:
  /// This method describes one loop of the observer (from k_0 to k_0+1)
  /// it has to be implemented in derived classes.
  virtual StateVector oneStepEstimation_() = 0;

  /// while the measurements and iputs are put in lists

  /// The state estimation of the observer (only one state is recorded)
  IndexedVector x_;

  /// Container for the measurements.
  IndexedVectorArray y_;

  /// Container for the inputs.
  std::shared_ptr<IndexedInputArrayInterface> u_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace stateObservation

#endif // ZERODELAYOBSERVER
