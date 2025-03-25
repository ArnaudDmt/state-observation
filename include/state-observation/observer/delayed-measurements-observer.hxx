#pragma once

#include "state-observation/tools/definitions.hpp"
#include <state-observation/observer/delayed-measurements-observer.hpp>

namespace stateObservation
{
template<typename IterationT>
DelayedMeasurementObserver<IterationT>::DelayedMeasurementObserver(Index n, Index m, double dt, Index p)
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
void DelayedMeasurementObserver<IterationT>::setMeasurement(const ObserverBase::MeasureVector & y_k, TimeIndex k)
{
  getCurrentIter().startNewIteration();

  ObserverBase::setMeasurement(y_k, k);
  getCurrentIter().y_ = y_k;

  bufferedIters_.push_front(Iteration(dt_));
}

} // namespace stateObservation
