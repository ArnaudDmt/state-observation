#include <state-observation/observer/delayed-measurements-observer.hpp>

namespace stateObservation
{
DelayedMeasurementObserver::DelayedMeasurementObserver(double dt,
                                                       Index n,
                                                       Index m,
                                                       unsigned long bufferCapacity,
                                                       const std::shared_ptr<IndexedInputArrayInterface> input,
                                                       const std::shared_ptr<AsynchronousDataMapBase> async_input,
                                                       const std::shared_ptr<AsynchronousDataMapBase> async_meas)
: ObserverBase(n, m)
{
  u_ = input;
  u_asynchronous_ = async_input;
  y_asynchronous_ = async_meas;

  setStateCapacity(bufferCapacity / dt);
  setSamplingTime(dt);
}

void DelayedMeasurementObserver::initEstimator(const Vector & x)
{
  xBuffer_.push_front(IndexedVector(x, 0));
}

TimeIndex DelayedMeasurementObserver::getAsynchronousFirstIndex()
{
  TimeIndex k_u_async, k_y_async = std::numeric_limits<long>::max();

  if(u_asynchronous_ && !u_asynchronous_->empty())
  {
    k_u_async = u_asynchronous_->getFirstIndex() + 1;
  }
  if(y_asynchronous_ && !y_asynchronous_->empty())
  {
    k_y_async = y_asynchronous_->getFirstIndex();
  }

  return std::min(k_u_async, k_y_async);
}

const ObserverBase::StateVector & DelayedMeasurementObserver::getEstimatedState(TimeIndex k)
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");

  TimeIndex k0 = getCurrentTime();

  // if there are asynchronous measurements, the impacted previous iterations are re-computed until the current one.
  if(u_asynchronous_ || y_asynchronous_)
  {
    TimeIndex k_asynchronous = getAsynchronousFirstIndex();
    if((k0 - k_asynchronous) < xBuffer_.size())
    {
      StateIterator it_x = xBuffer_.begin() + k0 - k_asynchronous;

      // we remove all the input and measurements from iterations before the oldest asynchronous measurement
      if(u_)
      {
        while(u_->size() > 0 && u_->getFirstIndex() < k_asynchronous)
        {
          u_->popFront();
        }
      }

      while(y_.size() > 0 && y_.getFirstIndex() <= k_asynchronous)
      {
        y_.popFront();
      }

      for(boost::circular_buffer<IndexedVector>::iterator it = it_x; it != xBuffer_.begin(); --it)
      {
        oneStepEstimation_(it);
        if(u_ && u_->size() > 0 && u_->checkIndex(it->getTime() - 1))
        {
          u_->popFront();
        }
        if(y_.size() > 0 && y_.checkIndex(it->getTime()))
        {
          y_.popFront();
        }
        if(!u_asynchronous_->empty() > 0 && u_asynchronous_->checkIndex(it->getTime() - 1))
        {
          u_asynchronous_->erase(it->getTime() - 1);
        }
        if(!y_asynchronous_->empty() > 0 && y_asynchronous_->checkIndex(it->getTime()))
        {
          y_asynchronous_->erase(it->getTime());
        }
      }
    }
  }

  for(TimeIndex i = k0; i < k; ++i)
  {
    xBuffer_.push_front(IndexedVector(xBuffer_.front()(), i + 1));
    oneStepEstimation_(xBuffer_.begin());

    TimeIndex oldest_k_x = xBuffer_.begin()->getTime();

    if(u_ && u_->size() > 0 && u_->getFirstIndex() < oldest_k_x)
    {
      u_->popFront();
    }
    if(y_.size() > 0 && y_.getFirstIndex() <= oldest_k_x)
    {
      y_.popFront();
    }
  }

  return xBuffer_.front()();
}

const ObserverBase::StateVector & DelayedMeasurementObserver::getCurrentEstimatedState()
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");
  return xBuffer_.front()();
}

Vector DelayedMeasurementObserver::getMeasurement(TimeIndex k) const
{
  return y_[k];
}

TimeIndex DelayedMeasurementObserver::getMeasurementTime() const
{
  BOOST_ASSERT(y_.size() > 0 && "ERROR: There is no measurements registered (past measurements are erased)");
  return y_.getLastIndex();
}

TimeIndex DelayedMeasurementObserver::getCurrentTime() const
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");
  return xBuffer_.front().getTime();
}

bool DelayedMeasurementObserver::stateIsSet() const
{
  return xBuffer_.front().isSet();
}

void DelayedMeasurementObserver::pushAsyncMeasurement(const AsynchronousDataBase & asyncMeas, TimeIndex k)
{
  y_asynchronous_->pushValue(asyncMeas, k);
}

void DelayedMeasurementObserver::setMeasurement(const ObserverBase::MeasureVector & y_k, TimeIndex k)
{

  BOOST_ASSERT(checkMeasureVector(y_k) && "The size of the measure vector is incorrect");
  if(y_.size() > 0)
    BOOST_ASSERT((y_.getNextIndex() == k || y_.checkIndex(k)) && "ERROR: The time is set incorrectly for \
                                the measurements (order or gap)");
  else
    BOOST_ASSERT((!xBuffer_.front().isSet() || xBuffer_.front().getTime() == k - 1)
                 && "ERROR: The time is set incorrectly for the measurements \
                                (must be [current_time+1])");

  y_.setValue(y_k, k);
}

void DelayedMeasurementObserver::pushInput(const InputBase & u_k)
{
  BOOST_ASSERT(u_ && "The input vector has not been initialized in the contstructor.");
  if(u_->size() > 0)
  {
    u_->pushBack(u_k);
  }
  else
  {
    BOOST_ASSERT(xBuffer_.size() > 0
                 && "Unable to initialize input without time index, the state vector has not been set.");
    /// we need the input at the time of the state vector to predict the next one
    u_->setValue(u_k, getCurrentTime());
  }
}

void DelayedMeasurementObserver::pushAsyncInput(const AsynchronousDataBase & asyncInput, TimeIndex k)
{
  u_asynchronous_->pushValue(asyncInput, k);
}

void DelayedMeasurementObserver::setState(const ObserverBase::StateVector & x_k, TimeIndex k)
{
  BOOST_ASSERT(checkStateVector(x_k) && "The size of the state vector is incorrect");
  BOOST_ASSERT(k > getCurrentTime() && "Cannot modify a past state.");
  if(xBuffer_.empty())
  {

    xBuffer_.push_front(IndexedVector(x_k, k));
  }

  // xBuffer_.front().set(x_k, k);

  // if(k < getCurrentTime())
  // {
  //   y_.clear();
  //   u_.clear();
  // }
  // else
  // {
  //   while(y_.size() > 0 && y_.getFirstIndex() <= k)
  //   {
  //     y_.popFront();
  //   }

  //   if(p_ > 0)
  //     while(u_.size() > 0 && u_.getFirstIndex() < k)
  //     {
  //       u_.popFront();
  //     }
  // }
}

void DelayedMeasurementObserver::setCurrentState(const ObserverBase::StateVector & x_k)
{
  BOOST_ASSERT(xBuffer_.front().isSet() && "The state vector has not been set");
  BOOST_ASSERT(checkStateVector(x_k) && "The size of the state vector is incorrect");

  xBuffer_.front()() = x_k;
}

void DelayedMeasurementObserver::clearStates()
{
  xBuffer_.front().reset();
}

void DelayedMeasurementObserver::clearMeasurements()
{
  y_.reset();
}

void DelayedMeasurementObserver::setInput(const InputBase & u_k, TimeIndex k)
{
  BOOST_ASSERT(u_ && "The input vector has not been initialized in the contstructor.");
  BOOST_ASSERT((u_->getNextIndex() == k || u_->checkIndex(k)) && "ERROR: The time is set incorrectly \
                                for the inputs (order or gap)");

  u_->setValue(u_k, k);
}

void DelayedMeasurementObserver::clearInputs()
{
  BOOST_ASSERT(u_ && "The input vector has not been initialized in the contstructor.");
  u_->reset();
}

} // namespace stateObservation
