#include <algorithm>
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
  currentIter_ = 0;

  setStateCapacity(bufferCapacity / dt);
  setSamplingTime(dt);
}

void DelayedMeasurementObserver::initEstimator(const Vector & x)
{
  xBuffer_.push_front(IndexedVector(x, 0));
}

TimeIndex DelayedMeasurementObserver::getAsynchronousFirstIndex()
{
  TimeIndex k_u_async = std::numeric_limits<long>::max();
  TimeIndex k_y_async = std::numeric_limits<long>::max();

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

  for(TimeIndex i = currentIter_; i < k; ++i)
  {
    // we store the index of the oldest element before it potentially gets pushed out of the buffer
    TimeIndex removed_k_x = xBuffer_.back().getTime();
    xBuffer_.push_front(IndexedVector(xBuffer_.front()(), i + 1));
    oneStepEstimation_(xBuffer_.begin());

    if(xBuffer_.full())
    {
      if(u_ && u_->size() > 0 && u_->getFirstIndex() < removed_k_x)
      {
        u_->popFront();
      }
      if(y_.size() > 0 && y_.getFirstIndex() <= removed_k_x)
      {
        y_.popFront();
      }
      if(u_asynchronous_ && u_asynchronous_->checkIndex(removed_k_x - 1))
      {
        u_asynchronous_->erase(removed_k_x - 1);
      }
      if(y_asynchronous_ && y_asynchronous_->checkIndex(removed_k_x))
      {
        y_asynchronous_->erase(removed_k_x);
      }
    }
  }
  currentIter_ = getCurrentTime();

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
  if(k > xBuffer_.back().getTime())
  {
    y_asynchronous_->pushValue(asyncMeas, k);
    currentIter_ = std::min(currentIter_, k);
  }
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

void DelayedMeasurementObserver::pushAsyncInput(const AsynchronousDataBase & asyncInput, TimeIndex k)
{
  if(k >= xBuffer_.back().getTime())
  {
    u_asynchronous_->pushValue(asyncInput, k);
    currentIter_ = std::min(currentIter_, k);
  }
}

void DelayedMeasurementObserver::setState(const ObserverBase::StateVector & x_k, TimeIndex k)
{
  BOOST_ASSERT(checkStateVector(x_k) && "The size of the state vector is incorrect");
  BOOST_ASSERT(k > getCurrentTime() && "Cannot modify a past state.");
  if(xBuffer_.empty())
  {

    xBuffer_.push_front(IndexedVector(x_k, k));
  }
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

void DelayedMeasurementObserver::clearDelayedMeasurements()
{
  BOOST_ASSERT(y_asynchronous_ && "The delayed measurements vector has not been initialized in the constructor.");
  y_asynchronous_->clear();
}

void DelayedMeasurementObserver::setInput(const InputBase & u_k, TimeIndex k)
{
  BOOST_ASSERT(u_ && "The input vector has not been initialized in the constructor.");
  BOOST_ASSERT((u_->getNextIndex() == k || u_->checkIndex(k)) && "ERROR: The time is set incorrectly \
                                for the inputs (order or gap)");

  u_->setValue(u_k, k);
}

void DelayedMeasurementObserver::clearInputs()
{
  BOOST_ASSERT(u_ && "The input vector has not been initialized in the constructor.");
  u_->reset();
}

void DelayedMeasurementObserver::clearDelayedInputs()
{
  BOOST_ASSERT(u_asynchronous_ && "The delayed inputs vector has not been initialized in the constructor.");
  u_asynchronous_->clear();
}

} // namespace stateObservation
