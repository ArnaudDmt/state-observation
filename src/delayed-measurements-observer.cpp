#include <state-observation/observer/delayed-measurements-observer.hpp>
#include <state-observation/tools/definitions.hpp>

namespace stateObservation
{
DelayedMeasurementObserver::DelayedMeasurementObserver(double dt,
                                                       Index n,
                                                       Index m,
                                                       unsigned long bufferCapacity,
                                                       Index p)
: ObserverBase(n, m, p)
{
  setStateCapacity(bufferCapacity / dt);
  setSamplingTime(dt);
}

void DelayedMeasurementObserver::initEstimator(const Vector & x)
{
  xBuffer_.push_front(IndexedVector(x, 0));
}

ObserverBase::StateVector DelayedMeasurementObserver::getEstimatedState(TimeIndex k)
{
  BOOST_ASSERT(stateIsSet() && "The state vector has not been set");

  TimeIndex k0 = getCurrentTime();

  // if there are asynchronous measurements, the impacted previous iterations are re-computed until the current one.
  if(!y_asynchronous_.empty())
  {
    TimeIndex k_asynchronousMeas = y_asynchronous_.top().getTime();
    StateIterator it_x = xBuffer_.begin() + k0 - k_asynchronousMeas - 1;

    BOOST_ASSERT_MSG(it_x->getTime() - 1 == y_asynchronous_.top().getTime(), "WESH");

    // we remove all the input and measurements from iterations before the oldest asynchronous measurement
    while(y_.size() > 0 && y_.getFirstIndex() < k_asynchronousMeas)
    {
      y_.popFront();
    }
    if(p_ > 0)
      while(u_.size() > 0 && u_.getFirstIndex() < k_asynchronousMeas)
      {
        u_.popFront();
      }

    for(boost::circular_buffer<IndexedVector>::iterator it = it_x; it != xBuffer_.begin(); --it)
    {
      oneStepEstimation_(it);

      BOOST_ASSERT_MSG((it)->getTime() == (it + 1)->getTime() + 1, "WESH2");

      if(p_ > 0)
      {
        u_.popFront();
      }

      y_.popFront();
    }
  }

  for(TimeIndex i = k0; i < k; ++i)
  {
    xBuffer_.push_front(IndexedVector(xBuffer_.front()(), i + 1));
    oneStepEstimation_(xBuffer_.begin());

    if(p_ > 0)
    {
      u_.popFront();
    }
    y_.popFront();
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

void DelayedMeasurementObserver::setAsyncMeasurement(const AsynchronousMeasurement & asyncMeas)
{
  y_asynchronous_.push(asyncMeas);
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

void DelayedMeasurementObserver::setInput(const ObserverBase::InputVector & u_k, TimeIndex k)
{
  if(p_ > 0)
  {
    BOOST_ASSERT(checkInputVector(u_k) && "The size of the input vector is incorrect");

    if(u_.size() > 0)
      BOOST_ASSERT((u_.getNextIndex() == k || u_.checkIndex(k)) && "ERROR: The time is set incorrectly \
                                for the inputs (order or gap)");
    else
    {
      BOOST_ASSERT((!xBuffer_.front().isSet() || xBuffer_.front().getTime() == k || xBuffer_.front().getTime() == k - 1)
                   && "ERROR: The time is set incorrectly for the \
                          inputs (must be [current_time] or [current_time+1])");
    }

    u_.setValue(u_k, k);
  }
}

void DelayedMeasurementObserver::clearInputs()
{
  u_.reset();
}

} // namespace stateObservation
