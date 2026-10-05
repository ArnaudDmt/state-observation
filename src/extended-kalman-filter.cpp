#include <state-observation/observer/extended-kalman-filter.hpp>

namespace stateObservation
{
ExtendedKalmanFilter::ExtendedKalmanFilter(Index n,
                                           Index m,
                                           bool directInputOutputFeedthrough,
                                           bool directInputStateProcessFeedthrough,
                                           const std::shared_ptr<IndexedInputArrayInterface> input)
: KalmanFilterBase(n, m, input), directInputOutputFeedthrough_(directInputOutputFeedthrough),
  directInputStateProcessFeedthrough_(directInputStateProcessFeedthrough), f_(0x0)
{
#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
  std::cout << std::endl << "ExtendedKalmanFilter Constructor" << std::endl;
#endif // STATEOBSERVATION_VERBOUS_CONSTRUCTOR
}

ExtendedKalmanFilter::ExtendedKalmanFilter(Index n,
                                           Index nt,
                                           Index m,
                                           Index mt,
                                           bool directInputOutputFeedthrough,
                                           bool directInputStateProcessFeedthrough,
                                           const std::shared_ptr<IndexedInputArrayInterface> input)
: KalmanFilterBase(n, nt, m, mt, input), directInputOutputFeedthrough_(directInputOutputFeedthrough),
  directInputStateProcessFeedthrough_(directInputStateProcessFeedthrough), f_(0x0)
{
#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
  std::cout << std::endl << "ExtendedKalmanFilter Constructor" << std::endl;
#endif // STATEOBSERVATION_VERBOUS_CONSTRUCTOR
}

void ExtendedKalmanFilter::setFunctor(DynamicalSystemFunctorBase * f)
{
  f_ = f;
  // f_->reset();
}

DynamicalSystemFunctorBase * ExtendedKalmanFilter::getFunctor(void) const
{
  return f_;
}

void ExtendedKalmanFilter::clearFunctor()
{
  f_ = 0x0;
}

void ExtendedKalmanFilter::setDirectInputOutputFeedthrough(bool b)
{

  directInputOutputFeedthrough_ = b;
}

void ExtendedKalmanFilter::setDirectInputStateFeedthrough(bool b)
{

  directInputStateProcessFeedthrough_ = b;
}

ObserverBase::StateVector ExtendedKalmanFilter::prediction_(TimeIndex k)
{
  if(!xbar_.isSet() || xbar_.getTime() != k)
  {
    BOOST_ASSERT(f_ != 0x0 && "ERROR: The Kalman filter functor is not set");
    if(directInputStateProcessFeedthrough_)
    {
      BOOST_ASSERT(this->u_->size() > 0 && this->u_->checkIndex(k - 1) && "ERROR: The input vector is not set");
      xbar_.set(f_->stateDynamics(this->x_(), (*u_)[k - 1], this->x_.getTime()), k);
    }
    else
    {
      xbar_.set(f_->stateDynamics(this->x_(), InputT<>(), this->x_.getTime()), k);
    }
  }

  return xbar_();
}

ObserverBase::MeasureVector ExtendedKalmanFilter::predictSensor_(TimeIndex k)
{

  if(!this->ybar_.isSet() || this->ybar_.getTime() != k)
  {
    ybar_.set(simulateSensor_(xbar_(), k), k);
  }

  return ybar_();
}

ObserverBase::MeasureVector ExtendedKalmanFilter::simulateSensor_(const ObserverBase::StateVector & x, TimeIndex k)
{
  BOOST_ASSERT(f_ != 0x0 && "ERROR: The Kalman filter functor is not set");

  if(directInputOutputFeedthrough_)
  {
    BOOST_ASSERT(u_->checkIndex(k) && "ERROR: The input feedthrough of the measurements is not set \
(the measurement at time k needs the input at time k which was not given) \
if you don't need the input in the computation of measurement, you \
must set directInputOutputFeedthrough to 'false' in the constructor");
    IndexedInputArrayInterface & uArray = *u_;
    return f_->measureDynamics(x, uArray[k], k);
  }
  else
  {
    return f_->measureDynamics(x, InputT<>(), k);
  }
}

KalmanFilterBase::Amatrix // ExtendedKalmanFilter<n,m,p>::Amatrix does not work
    ExtendedKalmanFilter::getAMatrixFD(const Vector & dx)
{
  TimeIndex k = this->x_.getTime();
  opt.a_.resize(nt_, nt_);
  updateStatePrediction();

  opt.x_ = this->x_();
  opt.dx_.resize(nt_);

  for(Index i = 0; i < nt_; ++i)
  {
    opt.dx_.setZero();
    opt.dx_[i] = dx[i];

    arithm_->stateSum(this->x_(), opt.dx_, opt.x_);

    if(directInputStateProcessFeedthrough_)
      opt.xp_ = f_->stateDynamics(opt.x_, (*u_)[k], k);
    else
      opt.xp_ = f_->stateDynamics(opt.x_, InputT<>(), k);

    arithm_->stateDifference(opt.xp_, xbar_(), opt.dx_);

    opt.dx_ /= dx[i];

    opt.a_.col(i) = opt.dx_;
  }
  return opt.a_;
}

KalmanFilterBase::Cmatrix ExtendedKalmanFilter::getCMatrixFD(const Vector & dx)
{
  TimeIndex k = this->x_.getTime();

  opt.c_.resize(m_, nt_);

  updateStateAndMeasurementPrediction();

  xbar_.set(prediction_(k + 1), k + 1);

  opt.dx_.resize(nt_);

  for(Index i = 0; i < nt_; ++i)
  {
    opt.dx_.setZero();
    opt.dx_[i] = dx[i];

    arithm_->stateSum(xbar_(), opt.dx_, opt.xp_);

    opt.yp_ = simulateSensor_(opt.xp_, k + 1);
    opt.yp_ -= ybar_();
    opt.yp_ /= dx[i];

    opt.c_.col(i) = opt.yp_;
  }

  return opt.c_;
}

void ExtendedKalmanFilter::reset()
{
  KalmanFilterBase::reset();
  if(f_ != 0x0) f_->reset();
}

DynamicalSystemFunctorBase * ExtendedKalmanFilter::functor() const
{
  return f_;
}

} // namespace stateObservation
