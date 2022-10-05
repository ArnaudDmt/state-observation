#include <state-observation/observer/extended-kalman-filter.hpp>
#include <iostream>

namespace stateObservation
{
ExtendedKalmanFilter::ExtendedKalmanFilter(Index n, Index m)
: KalmanFilterBase(n, m, 0), directInputOutputFeedthrough_(false), directInputStateProcessFeedthrough_(false), f_(0x0)

{
#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
  std::cout << std::endl << "ExtendedKalmanFilter Constructor" << std::endl;
#endif // STATEOBSERVATION_VERBOUS_CONSTRUCTOR
}

ExtendedKalmanFilter::ExtendedKalmanFilter(Index n,
                                           Index m,
                                           Index p,
                                           bool directInputOutputFeedthrough,
                                           bool directInputStateProcessFeedthrough)
: KalmanFilterBase(n, m, p), directInputOutputFeedthrough_(directInputOutputFeedthrough),
  directInputStateProcessFeedthrough_(directInputStateProcessFeedthrough), f_(0x0)
{
#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
  std::cout << std::endl << "ExtendedKalmanFilter Constructor" << std::endl;
#endif // STATEOBSERVATION_VERBOUS_CONSTRUCTOR

  if(p == 0)
  {
    directInputOutputFeedthrough_ = false;
    directInputStateProcessFeedthrough_ = false;
  }
}

ExtendedKalmanFilter::ExtendedKalmanFilter(Index n,
                                           Index nt,
                                           Index m,
                                           Index mt,
                                           Index p,
                                           bool directInputOutputFeedthrough,
                                           bool directInputStateProcessFeedthrough)
: KalmanFilterBase(n, nt, m, mt, p), directInputOutputFeedthrough_(directInputOutputFeedthrough),
  directInputStateProcessFeedthrough_(directInputStateProcessFeedthrough), f_(0x0)
{
#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
  std::cout << std::endl << "ExtendedKalmanFilter Constructor" << std::endl;
#endif // STATEOBSERVATION_VERBOUS_CONSTRUCTOR

  if(p == 0)
  {
    directInputOutputFeedthrough_ = false;
    directInputStateProcessFeedthrough_ = false;
  }
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
  if(p_ > 0)
  {
    directInputOutputFeedthrough_ = b;
  }
}

void ExtendedKalmanFilter::setDirectInputStateFeedthrough(bool b)
{
  if(p_ > 0)
  {
    directInputStateProcessFeedthrough_ = b;
  }
}

ObserverBase::StateVector ExtendedKalmanFilter::prediction_(TimeIndex k)
{
  if(!xbar_.isSet() || xbar_.getTime() != k)
  {
    if((p_ > 0) && (directInputStateProcessFeedthrough_))
    {

      BOOST_ASSERT(this->u_.size() > 0 && this->u_.checkIndex(k - 1) && "ERROR: The input vector is not set");
      opt.u_ = this->u_[k - 1];
    }
    else
    {
      opt.u_ = inputVectorZero();
    }
    BOOST_ASSERT(f_ != 0x0 && "ERROR: The Kalman filter functor is not set");
    xbar_.set(f_->stateDynamics(this->x_(), opt.u_, this->x_.getTime()), k);
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

  if(p_ > 0)
  {
    if(directInputOutputFeedthrough_)
    {
      BOOST_ASSERT(u_.checkIndex(k) && "ERROR: The input feedthrough of the measurements is not set \
(the measurement at time k needs the input at time k which was not given) \
if you don't need the input in the computation of measurement, you \
must set directInputOutputFeedthrough to 'false' in the constructor");
    }

    if(u_.checkIndex(k))
    {
      opt.u_ = u_[k];
    }
    else
    {
      opt.u_ = inputVectorZero();
    }
  }

  return f_->measureDynamics(x, opt.u_, k);
}

Matrix displayVectorWithIndex(const Vector & vec1, const Vector & indexes) // to be removed
{
  Matrix C(vec1.rows(), vec1.cols()+indexes.cols());
  C << vec1, indexes;
  return C;
}

KalmanFilterBase::Amatrix // ExtendedKalmanFilter<n,m,p>::Amatrix does not work
    ExtendedKalmanFilter::getAMatrixFD(const Vector & dx)
{
  TimeIndex k = this->x_.getTime();
  opt.a_.resize(nt_, nt_);
  updateStatePrediction();

  opt.x_ = this->x_();
  opt.dx_.resize(nt_);
  std::cout << std::endl << "opt.x_: " << std::endl << opt.x_ << std::endl;
  if(p_ > 0)
  {
    if(directInputStateProcessFeedthrough_)
      opt.u_ = this->u_[k];
    else
      opt.u_ = inputVectorZero();
  }
  const int nbIndexes = 48;
  Eigen::VectorXd indexes(nbIndexes); //to be removed
  //Eigen::VectorXd indexes(57); //to be removed
  for (int ind = 0; ind<indexes.size(); ind++)
  {
    indexes(ind) = ind;
  }
  for(Index i = 0; i < nt_; ++i)
  {
    //std::cout << std::endl << "col: " << std::endl << i << std::endl;
    opt.dx_.setZero();
    opt.dx_[i] = dx[i];
    //std::cout << std::endl << "dx: " << std::endl << displayVectorWithIndex(dx.segment<57>(0), indexes) << std::endl;
    //std::cout << std::endl << "dx: " << std::endl << displayVectorWithIndex(dx.segment<nbIndexes>(0), indexes) << std::endl;
    //std::cout << std::endl << "opt.x_ before sum: " << std::endl << displayVectorWithIndex(opt.x_.segment<nbIndexes>(0), indexes) << std::endl;
    //std::cout << std::endl << "opt.x_ before sum: " << std::endl << displayVectorWithIndex(opt.x_.segment<57>(0), indexes) << std::endl; // we don't display the two empty contacts

    arithm_->stateSum(this->x_(), opt.dx_, opt.x_);
    //std::cout << std::endl << "opt.x_ after sum: " << std::endl << displayVectorWithIndex(opt.x_.segment<nbIndexes>(0), indexes) << std::endl;
    //std::cout << std::endl << "opt.x_ after sum: " << std::endl << displayVectorWithIndex(opt.x_.segment<57>(0), indexes) << std::endl; // we don't display the two empty contacts

    opt.xp_ = f_->stateDynamics(opt.x_, opt.u_, k);
    
    //std::cout << std::endl << "opt.x_ after stateDynamics: " << std::endl << displayVectorWithIndex(opt.xp_.segment<nbIndexes>(0), indexes) << std::endl;
    //std::cout << std::endl << "opt.x_ after stateDynamics: " << std::endl << displayVectorWithIndex(opt.xp_.segment<57>(0), indexes) << std::endl; // we don't display the two empty contacts
    //std::cout << std::endl << "xbar_(): " << std::endl << displayVectorWithIndex(xbar_().segment<nbIndexes>(0), indexes) << std::endl;
    //std::cout << std::endl << "xbar_(): " << std::endl << displayVectorWithIndex(xbar_().segment<57>(0), indexes) << std::endl; // we don't display the two empty contacts
    arithm_->stateDifference(opt.xp_, xbar_(), opt.dx_);
    //std::cout << std::endl << "Difference: " << std::endl << displayVectorWithIndex(opt.dx_.segment<57>(0), indexes) << std::endl; // we don't display the two empty contacts
    //std::cout << std::endl << "Difference: " << std::endl << displayVectorWithIndex(opt.dx_.segment<nbIndexes>(0), indexes) << std::endl;

    opt.dx_ /= dx[i];

    bool stopTest = false;
    for (int j = 0; j < nt_; j++)
    {
      
      if (opt.dx_.coeff(j) > 1e+30 )
      {
        //std::cout << std::endl << "error indexes: " << std::endl << "(" << j << "," << i << ")" << std::endl;
        BOOST_ASSERT(false && "error on A");
        stopTest = true;
      }

    }
    BOOST_ASSERT(!stopTest && "Erreurs sur A");


    opt.a_.col(i) = opt.dx_;
  }
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
  Matrix sumA;
  sumA = Matrix::Zero(nt_, nt_);
  for(Index i = 0; i < nt_; i+=3) // to be deleted
  {
    sumA(i,i) = opt.a_.block<3,3>(i, i).mean();
  }
  std::cout << std::endl << "A compact: " << std::endl << sumA.format(CleanFmt) << std::endl;

  
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
