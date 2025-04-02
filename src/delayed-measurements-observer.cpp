#pragma once

#include "state-observation/tools/definitions.hpp"
#include <state-observation/observer/delayed-measurements-observer.hpp>

namespace stateObservation
{
template<typename IterationT>
DelayedMeasurementObserver<IterationT>::DelayedMeasurementObserver(double dt, Index n, Index m, Index p)
: ObserverBase(n, m, p)
{
  setSamplingTime(dt);
}

template<typename IterationT>
DelayedMeasurementObserver<IterationT>::DelayedMeasurementObserver(double dt,
                                                                   Index n,
                                                                   Index m,
                                                                   unsigned long bufferCapacity,
                                                                   Index p)
: ObserverBase(n, m, p)
{
  bufferedIters_.set_capacity(bufferCapacity);
  setSamplingTime(dt);
}

template<typename IterationT>
void DelayedMeasurementObserver<IterationT>::initEstimator(const Vector & x)
{
  setState(x, 0);
}

template<typename IterationT>
ObserverBase::StateVector DelayedMeasurementObserver<IterationT>::getEstimatedState(TimeIndex k)
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");

  TimeIndex k0 = getCurrentTime();

  BOOST_ASSERT(k0 <= k && "ERROR: The observer cannot estimate previous states");

  for(TimeIndex i = k0; i < k; ++i)
  {
    oneStepEstimation_();
    if(y_.getFirstIndex() < k) y_.popFront();

    if(p_ > 0)
      if(u_.getFirstIndex() < k) u_.popFront();
  }

  return x_();
}

template<typename IterationT>
ObserverBase::StateVector DelayedMeasurementObserver<IterationT>::getCurrentEstimatedState() const
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");
  return x_();
}

template<typename IterationT>
Vector DelayedMeasurementObserver<IterationT>::getMeasurement(TimeIndex k) const
{
  return y_[k];
}

template<typename IterationT>
TimeIndex DelayedMeasurementObserver<IterationT>::getMeasurementTime() const
{
  BOOST_ASSERT(y_.size() > 0 && "ERROR: There is no measurements registered (past measurements are erased)");
  return y_.getLastIndex();
}

template<typename IterationT>
TimeIndex DelayedMeasurementObserver<IterationT>::getCurrentTime() const
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");
  return x_.getTime();
}

template<typename IterationT>
bool DelayedMeasurementObserver<IterationT>::stateIsSet() const
{
  return x_.isSet();
}

template<typename IterationT>
void DelayedMeasurementObserver<IterationT>::setMeasurement(const ObserverBase::MeasureVector & y_k, TimeIndex k)
{

  BOOST_ASSERT(checkMeasureVector(y_k) && "The size of the measure vector is incorrect");
  if(y_.size() > 0)
    BOOST_ASSERT((y_.getNextIndex() == k || y_.checkIndex(k)) && "ERROR: The time is set incorrectly for \
                                the measurements (order or gap)");
  else
    BOOST_ASSERT((!x_.isSet() || x_.getTime() == k - 1) && "ERROR: The time is set incorrectly for the measurements \
                                (must be [current_time+1])");

  startNewIteration_();

  getCurrentIter().y_ = y_k;
  y_.setValue(y_k, k);
}

template<typename IterationT>
void DelayedMeasurementObserver<IterationT>::setState(const ObserverBase::StateVector & x_k, TimeIndex k)
{
  BOOST_ASSERT(checkStateVector(x_k) && "The size of the state vector is incorrect");

  x_.set(x_k, k);

  if(k < getCurrentTime())
  {
    y_.clear();
    u_.clear();
  }
  else
  {
    while(y_.size() > 0 && y_.getFirstIndex() <= k)
    {
      y_.popFront();
    }

    if(p_ > 0)
      while(u_.size() > 0 && u_.getFirstIndex() < k)
      {
        u_.popFront();
      }
  }
}

template<typename IterationT>
void DelayedMeasurementObserver<IterationT>::setCurrentState(const ObserverBase::StateVector & x_k)
{
  BOOST_ASSERT(x_.isSet() && "The state vector has not been set");
  BOOST_ASSERT(checkStateVector(x_k) && "The size of the state vector is incorrect");

  x_() = x_k;
}

template<typename IterationT>
void DelayedMeasurementObserver<IterationT>::clearStates()
{
  x_.reset();
}

template<typename IterationT>
void DelayedMeasurementObserver<IterationT>::clearMeasurements()
{
  y_.reset();
}

template<typename IterationT>
void DelayedMeasurementObserver<IterationT>::setInput(const ObserverBase::InputVector & u_k, TimeIndex k)
{
  if(p_ > 0)
  {
    BOOST_ASSERT(checkInputVector(u_k) && "The size of the input vector is incorrect");

    if(u_.size() > 0)
      BOOST_ASSERT((u_.getNextIndex() == k || u_.checkIndex(k)) && "ERROR: The time is set incorrectly \
                                for the inputs (order or gap)");
    else
    {
      BOOST_ASSERT((!x_.isSet() || x_.getTime() == k || x_.getTime() == k - 1)
                   && "ERROR: The time is set incorrectly for the \
                          inputs (must be [current_time] or [current_time+1])");
    }

    u_.setValue(u_k, k);
  }
}

template<typename IterationT>
void DelayedMeasurementObserver<IterationT>::clearInputs()
{
  u_.reset();
}

} // namespace stateObservation
