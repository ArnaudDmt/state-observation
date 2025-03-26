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
void DelayedMeasurementObserver<IterationT>::initEstimator(const Vector & x)
{
  setState(x, 0);
}

template<typename IterationT>
void DelayedMeasurementObserver<IterationT>::setCurrentState(const ObserverBase::StateVector & x_k)
{
  BOOST_ASSERT(x_.isSet() && "The state vector has not been set");
  BOOST_ASSERT(checkStateVector(x_k) && "The size of the state vector is incorrect");

  x_() = x_k;
}

template<typename IterationT>
ObserverBase::StateVector DelayedMeasurementObserver<IterationT>::getCurrentEstimatedState() const
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");
  return x_();
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

} // namespace stateObservation
