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
#include <queue>
#include <state-observation/api.h>
#include <state-observation/observer/observer-base.hpp>

namespace stateObservation
{
struct AsynchronousMeasurement
{
  AsynchronousMeasurement(TimeIndex k) : k_(k) {}
  inline TimeIndex getTime() const
  {
    return k_;
  }

protected:
  TimeIndex k_;
};

/**
 * \class  DelayedMeasurementObserver
 * \brief
 *
 */
class STATE_OBSERVATION_DLLAPI DelayedMeasurementObserver : public ObserverBase
{

public:
  /// The constructor
  ///  \li n : size of the state vector
  ///  \li m : size of the measurements vector
  ///  \li dt  : timestep between each iteration
  ///  \li bufferCapacity  : capacity of the iteration buffer. Given in seconds, as the buffer's duration.
  DelayedMeasurementObserver(double dt, Index n, Index m, unsigned long bufferCapacity);

  /// Default constructor
  DelayedMeasurementObserver() = delete;

  /// Destructor
  virtual ~DelayedMeasurementObserver(){};

  // inline const IndexedVector & getPastState(size_t nbIters)
  // {
  //   return xBuffer_.at(nbIters);
  // }

  /// @brief Get the Current Estimated State
  /// @return ObserverBase::StateVector
  const ObserverBase::StateVector & getCurrentEstimatedState();

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
  virtual const ObserverBase::StateVector & getEstimatedState(TimeIndex k) override;

  /// @brief sets the measurement
  /// @param y the measurement vector
  /// @param k the time index
  void setMeasurement(const Vector & y, TimeIndex k) override;

  void pushInput(const std::any & u_k);

  void setAsyncMeasurement(const AsynchronousMeasurement & asyncMeas);

  /// Get the measurement of the time index k
  Vector getMeasurement(TimeIndex k) const;

  /// Get the time index of the last given measurement
  virtual TimeIndex getMeasurementTime() const;

  /// Remove all the given values of the measurements
  virtual void clearMeasurements() override;

  /// Set the value of the input vector at time index k. The
  /// inputs have to be inserted in chronological order without gaps.
  virtual void setInput(const std::any & u_k, TimeIndex k) override;

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

  /// set the sampling time of the measurements
  inline virtual void setStateCapacity(unsigned long stateCapacity)
  {
    xBuffer_.set_capacity(stateCapacity);
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
  typedef boost::circular_buffer<IndexedVector>::iterator StateIterator;

  struct GreaterIndexAM
  {
    bool operator()(const AsynchronousMeasurement & lhs, const AsynchronousMeasurement & rhs) const
    {
      return lhs.getTime() > rhs.getTime();
    }
  };

  /// @brief Runs one loop of the estimator.
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  virtual StateVector oneStepEstimation_(StateIterator it) = 0;
  virtual void startNewIteration_() = 0;

  inline const boost::circular_buffer<IndexedVector> & getStateVectorBuffer() const
  {
    return xBuffer_;
  }

  bool stateIsSet() const;

protected:
  /// Sampling time
  double dt_;
  /// The state estimation of the observer (only one state is recorded)
  boost::circular_buffer<IndexedVector> xBuffer_;
  /// Container for the measurements.
  IndexedVectorArray y_;

  std::priority_queue<AsynchronousMeasurement, std::vector<AsynchronousMeasurement>, GreaterIndexAM> y_asynchronous_;

  /// Container for the inputs.
  IndexedAnyArray u_;

  // TimeIndex k_est_ = 0; // time index of the last estimation
  // TimeIndex k_meas_ = 0; // time index of the current measurements
  // TimeIndex k_input_ = 0; // time index of the current measurements
};

} // namespace stateObservation

#endif // DelayedMeasurementObserverHPP
