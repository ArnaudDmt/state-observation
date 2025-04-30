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
#include <map>
#include <set>
#include <state-observation/api.h>
#include <state-observation/observer/observer-base.hpp>

namespace stateObservation
{

class AsynchronousDataBase
{
protected:
  AsynchronousDataBase() {}

public:
  virtual ~AsynchronousDataBase() {}
  virtual void merge(const AsynchronousDataBase & input2) = 0;
};

class AsynchronousDataMapBase
{
protected:
  AsynchronousDataMapBase() {}

public:
  virtual ~AsynchronousDataMapBase() {}
  virtual bool checkIndex(TimeIndex k) = 0;
  virtual void pushValue(const AsynchronousDataBase & v, TimeIndex k) = 0;
  virtual bool empty() const = 0;
  virtual void erase(TimeIndex k) = 0;
  virtual void clear() = 0;
  virtual AsynchronousDataBase & getElement(TimeIndex k) = 0;
  virtual TimeIndex getFirstIndex() = 0;
};

template<typename DataType>
struct AsynchronousDataMapT : public AsynchronousDataMapBase
{
public:
  inline bool checkIndex(TimeIndex k) override;
  inline void pushValue(const AsynchronousDataBase & v, TimeIndex k) override;
  inline void clear() override;
  inline void erase(TimeIndex k) override;
  inline bool empty() const override;
  inline AsynchronousDataBase & getElement(TimeIndex k) override;
  inline TimeIndex getFirstIndex() override;

protected:
  std::map<TimeIndex, DataType> v_;
};

template<typename DataType>
inline DataType & convert_async_data(AsynchronousDataBase & u);

template<typename DataType>
inline const DataType & convert_async_data(const AsynchronousDataBase & u);

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
  DelayedMeasurementObserver(double dt,
                             Index n,
                             Index m,
                             unsigned long bufferCapacity,
                             const std::shared_ptr<IndexedInputArrayInterface> input = nullptr,
                             const std::shared_ptr<AsynchronousDataMapBase> async_input = nullptr,
                             const std::shared_ptr<AsynchronousDataMapBase> async_meas = nullptr);

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

  void pushAsyncMeasurement(const AsynchronousDataBase & asyncMeas, TimeIndex k);

  /// Get the measurement of the time index k
  Vector getMeasurement(TimeIndex k) const;

  /// Get the time index of the last given measurement
  virtual TimeIndex getMeasurementTime() const;

  /// Remove all the given values of the measurements
  virtual void clearMeasurements() override;

  /// Remove all the given values of the measurements
  virtual void clearDelayedMeasurements();

  // void pushInput(const InputBase & u_k);

  void pushAsyncInput(const AsynchronousDataBase & asyncMeas, TimeIndex k);

  /// Set the value of the input vector at time index k. The
  /// inputs have to be inserted in chronological order without gaps.
  virtual void setInput(const InputBase & u_k, TimeIndex k) override;

  /// Remove all the given values of the inputs
  /// If there is no input, this instruction has no effect
  virtual void clearInputs() override;

  /// Remove all the given values of the delayed inputs
  /// If there is no input, this instruction has no effect
  virtual void clearDelayedInputs();

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

  TimeIndex getAsynchronousFirstIndex();

  /// Get the value of the time index of the current state estimation
  virtual TimeIndex getCurrentTime() const;

protected:
  typedef boost::circular_buffer<IndexedVector>::iterator StateIterator;

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
  /// Container for the inputs.
  std::shared_ptr<IndexedInputArrayInterface> u_;

  // indicates if new asynchronous data has been added and the impacted iterations must be run again
  bool new_async_data_;
  /// Container for the asynchronous measurements.
  std::shared_ptr<AsynchronousDataMapBase> y_asynchronous_;
  /// Container for the asynchronous inputs.
  std::shared_ptr<AsynchronousDataMapBase> u_asynchronous_;
};

} // namespace stateObservation

#include <state-observation/observer/delayed-measurements-observer.hxx>

#endif // DelayedMeasurementObserverHPP
