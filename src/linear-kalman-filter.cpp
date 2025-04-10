#include <state-observation/observer/linear-kalman-filter.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
namespace stateObservation
{

void LinearKalmanFilter::setB(const Bmatrix & B)
{
  BOOST_ASSERT(checkBmatrix(B) && "ERROR: The B matrix size is incorrect");
  b_ = B;
}

void LinearKalmanFilter::clearB()
{
  b_.resize(0, 0);
}

void LinearKalmanFilter::setD(const Dmatrix & D)
{
  BOOST_ASSERT(checkDmatrix(D) && "ERROR: The D matrix size is incorrect");

  d_ = D;
}

void LinearKalmanFilter::clearD()
{
  d_.resize(0, 0);
}

ObserverBase::StateVector LinearKalmanFilter::prediction_(TimeIndex k)
{
  (void)k; // unused

  BOOST_ASSERT(checkAmatrix(a_) && "ERROR: The A matrix is not initialized");
  BOOST_ASSERT(checkBmatrix(b_) && "ERROR: The B matrix is not initialized");
  BOOST_ASSERT(checkCmatrix(c_) && "ERROR: The C matrix is not initialized");

  xbar_.set(a_ * x_(), k);

  if(p_ > 0 && b_ != getBmatrixZero())
  {
    BOOST_ASSERT(u_.checkIndex(k - 1)
                 && "ERROR: The input feedthrough of the state dynamics is not set "
                    "(the state at time k+1 needs the input at time k which was not given) "
                    "if you don't need the input in the computation of state, you "
                    "must set B matrix to zero");

    const Vector & u = std::any_cast<Vector>(u_[k - 1]);
    BOOST_ASSERT(checkInputVector(u) && "The size of the input vector is incorrect.");

    xbar_().noalias() += b_ * u;
  }

  return xbar_();
}

ObserverBase::MeasureVector LinearKalmanFilter::simulateSensor_(const StateVector & x, TimeIndex k)
{

  BOOST_ASSERT(checkCmatrix(c_) && "ERROR: The C matrix is not initialized");

  if(p_ > 0 && checkDmatrix(d_) && d_ != getDmatrixZero())
  {
    BOOST_ASSERT(u_.checkIndex(k)
                 && "ERROR: The input feedthrough of the measurements is not set "
                    "(the measurement at time k needs the input at time k which was not given) "
                    "if you don't need the input in the computation of measurement, you "
                    "must set D matrix to zero");

    const Vector & u = std::any_cast<Vector>(u_[k]);
    BOOST_ASSERT(checkInputVector(u) && "The size of the input vector is incorrect.");

    ybar_.set(c_ * x + d_ * u, k);
  }
  else
  {
    ybar_.set(c_ * x, k);
  }
  return ybar_();
}

LinearKalmanFilter::Bmatrix LinearKalmanFilter::getBmatrixConstant(double c) const
{
  return Bmatrix::Constant(n_, p_, c);
}

LinearKalmanFilter::Bmatrix LinearKalmanFilter::getBmatrixRandom() const
{
  return tools::ProbabilityLawSimulation::getUniformMatrix<Bmatrix>(n_, p_);
}

LinearKalmanFilter::Bmatrix LinearKalmanFilter::getBmatrixZero() const
{
  return Bmatrix::Zero(n_, p_);
}

bool LinearKalmanFilter::checkBmatrix(const Bmatrix & a) const
{
  return (a.rows() == n_ && a.cols() == p_);
}

LinearKalmanFilter::Dmatrix LinearKalmanFilter::getDmatrixConstant(double c) const
{
  return Dmatrix::Constant(m_, p_, c);
}

LinearKalmanFilter::Dmatrix LinearKalmanFilter::getDmatrixRandom() const
{
  return tools::ProbabilityLawSimulation::getUniformMatrix<Dmatrix>(m_, p_);
}

LinearKalmanFilter::Dmatrix LinearKalmanFilter::getDmatrixZero() const
{
  return Dmatrix::Zero(m_, p_);
}

bool LinearKalmanFilter::checkDmatrix(const Dmatrix & a) const
{
  return (a.rows() == m_ && a.cols() == p_);
}

void LinearKalmanFilter::setStateSize(Index n)
{
  if(n != n_)
  {
    KalmanFilterBase::setStateSize(n);
    clearB();
  }
}

void LinearKalmanFilter::setMeasureSize(Index m)
{
  if(m != m_)
  {
    KalmanFilterBase::setMeasureSize(m);
    clearD();
  }
}

void LinearKalmanFilter::setInputSize(Index p)
{
  if(p != p_)
  {
    p_ = p;
    clearB();
    clearD();
  }
}

bool LinearKalmanFilter::checkInputVector(const StateVector & v) const
{
  return (v.rows() == n_ && v.cols() == 1);
}

Index LinearKalmanFilter::getInputSize() const
{
  return p_;
}

} // namespace stateObservation
