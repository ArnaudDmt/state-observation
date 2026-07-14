#include <state-observation/observer/zero-delay-observer.hpp>

namespace stateObservation
{

void ZeroDelayObserver::setState(const ObserverBase::StateVector & x_k, TimeIndex k)
{
  BOOST_ASSERT(checkStateVector(x_k) && "The size of the state vector is incorrect");
  BOOST_ASSERT(u_ && "The input has not been initialized yet.");

  x_.set(x_k, k);

  if(k < getCurrentTime())
  {
    y_.clear();
    u_->clear();
  }
  else
  {
    while(y_.size() > 0 && y_.getFirstIndex() <= k)
    {
      y_.popFront();
    }

    while(u_->size() > 0 && u_->getFirstIndex() < k)
    {
      u_->popFront();
    }
  }
} // namespace stateObservation

void ZeroDelayObserver::setCurrentState(const ObserverBase::StateVector & x_k)
{
  BOOST_ASSERT(x_.isSet() && "The state vector has not been set");
  BOOST_ASSERT(u_ && "The input has not been initialized yet.");
  BOOST_ASSERT(checkStateVector(x_k) && "The size of the state vector is incorrect");

  x_() = x_k;
}

void ZeroDelayObserver::clearStates()
{
  x_.reset();
}

bool ZeroDelayObserver::stateIsSet() const
{
  return x_.isSet();
}

void ZeroDelayObserver::setMeasurement(const ObserverBase::MeasureVector & y_k, TimeIndex k)
{
  BOOST_ASSERT(checkMeasureVector(y_k) && "The size of the measure vector is incorrect");
  if(y_.size() > 0)
    BOOST_ASSERT((y_.getNextIndex() == k || y_.checkIndex(k)) && "ERROR: The time is set incorrectly for \
                                the measurements (order or gap)");
  else
    BOOST_ASSERT((!x_.isSet() || x_.getTime() == k - 1) && "ERROR: The time is set incorrectly for the measurements \
                                (must be [current_time+1])");

  y_.setValue(y_k, k);
}

void ZeroDelayObserver::pushMeasurement(const ObserverBase::MeasureVector & y_k)
{

  BOOST_ASSERT(checkMeasureVector(y_k) && "The size of the measure vector is incorrect");
  if(y_.size() > 0)
  {
    y_.pushBack(y_k);
  }
  else
  {
    BOOST_ASSERT(x_.isSet()
                 && "Unable to initialize measurement without time index, the state vector has not been set.");
    /// we need the measurement of the next state to correct for the prediction
    y_.setValue(y_k, x_.getTime() + 1);
  }
}

void ZeroDelayObserver::clearMeasurements()
{
  y_.reset();
}

void ZeroDelayObserver::setInput(const InputBase & u_k, TimeIndex k)
{
  BOOST_ASSERT(u_ && "The input has not been initialized yet.");
  if(u_->size() > 0)
    BOOST_ASSERT((u_->getNextIndex() == k || u_->checkIndex(k)) && "ERROR: The time is set incorrectly \
                                for the inputs (order or gap)");
  else
  {
    BOOST_ASSERT((!x_.isSet() || x_.getTime() == k || x_.getTime() == k - 1)
                 && "ERROR: The time is set incorrectly for the \
                          inputs (must be [current_time] or [current_time+1])");
  }

  u_->setValue(u_k, k);
}

void ZeroDelayObserver::pushInput(const InputBase & u_k)
{
  BOOST_ASSERT(u_ && "The input has not been initialized yet.");
  if(u_->size() > 0)
  {
    u_->pushBack(u_k);
  }
  else
  {
    BOOST_ASSERT(x_.isSet() && "Unable to initialize input without time index, the state vector has not been set.");
    /// we need the input at the time of the state vector to predict the next one
    u_->setValue(u_k, x_.getTime());
  }
}

void ZeroDelayObserver::clearInputs()
{
  BOOST_ASSERT(u_ && "The input has not been initialized yet.");
  u_->reset();
}

void ZeroDelayObserver::clearInputsAndMeasurements()
{
  BOOST_ASSERT(u_ && "The input has not been initialized yet.");
  u_->reset();
  y_.reset();
}

TimeIndex ZeroDelayObserver::estimateState()
{
  if(getMeasurementsNumber() > 0)
  {
    getEstimatedState(getMeasurementTime());
  }
  return getCurrentTime();
}

const ObserverBase::StateVector & ZeroDelayObserver::getEstimatedState(TimeIndex k)
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");
  BOOST_ASSERT(u_ && "The input has not been initialized yet.");

  TimeIndex k0 = getCurrentTime();

  BOOST_ASSERT(k0 <= k && "ERROR: The observer cannot estimate previous states");

  for(TimeIndex i = k0; i < k; ++i)
  {
    oneStepEstimation_();
    if(y_.size() > 0 && y_.getFirstIndex() < k) y_.popFront();

    if(u_->size() > 0 && u_->getFirstIndex() < k) u_->popFront();
  }

  return x_();
}

const ObserverBase::StateVector & ZeroDelayObserver::getCurrentEstimatedState() const
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");
  return x_();
}

ObserverBase::StateVector & ZeroDelayObserver::getCurrentEstimatedState()
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");
  return x_();
}

TimeIndex ZeroDelayObserver::getCurrentTime() const
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");
  return x_.getTime();
}

const InputBase & ZeroDelayObserver::getInput(TimeIndex k) const
{
  BOOST_ASSERT(u_ && "The input has not been initialized yet.");
  return (*u_)[k];
}

InputBase & ZeroDelayObserver::getInput(TimeIndex k)
{
  BOOST_ASSERT(u_ && "The input has not been initialized yet.");
  return (*u_)[k];
}

TimeSize ZeroDelayObserver::getInputsNumber() const
{
  BOOST_ASSERT(u_ && "The input has not been initialized yet.");
  return u_->size();
}

TimeIndex ZeroDelayObserver::getInputTime() const
{
  BOOST_ASSERT(u_ && "The input has not been initialized yet.");
  BOOST_ASSERT(y_.size() > 0 && "ERROR: There is no measurements registered (past measurements are erased)");
  return u_->getLastIndex();
}

const Vector & ZeroDelayObserver::getMeasurement(TimeIndex k) const
{
  return y_[k];
}

TimeIndex ZeroDelayObserver::getMeasurementTime() const
{
  BOOST_ASSERT(y_.size() > 0 && "ERROR: There is no measurements registered (past measurements are erased)");
  return y_.getLastIndex();
}

TimeSize ZeroDelayObserver::getMeasurementsNumber() const
{
  return TimeSize(y_.size());
}

void ZeroDelayObserver::setStateSize(Index n)
{
  if(n != n_)
  {
    ObserverBase::setStateSize(n);
    clearStates();
  }
}

void ZeroDelayObserver::setMeasureSize(Index m)
{
  if(m != m_)
  {
    ObserverBase::setMeasureSize(m);
    clearMeasurements();
  }
}

} // namespace stateObservation
