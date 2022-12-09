#include <state-observation/observer/kalman-filter-base.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
#include <iostream>

#ifndef NDEBUG
//#define VERBOUS_KALMANFILTER
#endif

#ifdef VERBOUS_KALMANFILTER
#  include <iomanip> // std::setprecision
#  include <iostream>
#endif // VERBOUS_KALMANFILTER

namespace stateObservation
{

KalmanFilterBase::KalmanFilterBase() : nt_(0), arithm_(this) {}

KalmanFilterBase::KalmanFilterBase(Index n, Index m, Index p)
: ZeroDelayObserver(n, m, p), nt_(n), mt_(m), arithm_(this)
{
}

KalmanFilterBase::KalmanFilterBase(Index n, Index nt, Index m, Index mt, Index p)
: ZeroDelayObserver(n, m, p), nt_(nt), mt_(mt), arithm_(this)
{
}

void KalmanFilterBase::setA(const Amatrix & A)
{
  BOOST_ASSERT(checkAmatrix(A) && "ERROR: The A matrix dimensions are wrong");
  a_ = A;
}

Matrix KalmanFilterBase::getA() const
{
  return a_;
}

void KalmanFilterBase::clearA()
{
  a_.resize(0, 0);
}

void KalmanFilterBase::setC(const Cmatrix & C)
{
  BOOST_ASSERT(checkCmatrix(C) && "ERROR: The C matrix dimensions are wrong");
  c_ = C;
}

void KalmanFilterBase::clearC()
{
  c_.resize(0, 0);
}

Matrix KalmanFilterBase::getC() const
{
  return c_;
}

void KalmanFilterBase::setR(const Rmatrix & R)
{
  BOOST_ASSERT(checkRmatrix(R) && "ERROR: The dimensions of the measurement noise covariance matrix R are wrong");
  r_ = R;
}

Matrix KalmanFilterBase::getR() const
{
  return r_;
}

void KalmanFilterBase::clearR()
{
  r_.resize(0, 0);
}

void KalmanFilterBase::setQ(const Qmatrix & Q)
{
  BOOST_ASSERT(checkQmatrix(Q) && "ERROR: The dimensions of the process noise covariance matrix Q are wrong");
  q_ = Q;
}

Matrix KalmanFilterBase::getQ() const
{
  return q_;
}

void KalmanFilterBase::clearQ()
{
  q_.resize(0, 0);
}

void KalmanFilterBase::clearStates()
{
  ZeroDelayObserver::clearStates();
  clearStateCovariance();
}

void KalmanFilterBase::setStateCovariance(const Pmatrix & P)
{
  BOOST_ASSERT(checkPmatrix(P) && "ERROR: The P matrix dimensions are wrong");
  pr_ = P;
}

void KalmanFilterBase::clearStateCovariance()
{
  pr_.resize(0, 0);
}

ObserverBase::StateVector KalmanFilterBase::oneStepEstimation_()
{
  TimeIndex k = this->x_.getTime();

  BOOST_ASSERT(this->y_.size() > 0 && this->y_.checkIndex(k + 1) && "ERROR: The measurement vector is not set");

  BOOST_ASSERT(checkAmatrix(a_) && "ERROR: The Matrix A is not initialized");
  BOOST_ASSERT(checkCmatrix(c_) && "ERROR: The Matrix C is not initialized");
  BOOST_ASSERT(checkQmatrix(q_) && "ERROR: The Matrix Q is not initialized");
  BOOST_ASSERT(checkRmatrix(r_) && "ERROR: The Matrix R is not initialized");
  BOOST_ASSERT(checkPmatrix(pr_) && "ERROR: The Matrix P is not initialized");

  // prediction
  updateStateAndMeasurementPrediction(); // runs also updatePrediction_();
  oc_.pbar.noalias() = q_ + a_ * (pr_ * a_.transpose());

  // innovation Measurements
  arithm_->measurementDifference(this->y_[k + 1], ybar_(), oc_.inoMeas);
  oc_.inoMeasCov.noalias() = r_ + c_ * (oc_.pbar * c_.transpose());

  Index & measurementTangentSize = mt_;
  // inversing innovation measurement covariance matrix
  oc_.inoMeasCovLLT.compute(oc_.inoMeasCov);
  oc_.inoMeasCovInverse.resize(measurementTangentSize, measurementTangentSize);
  oc_.inoMeasCovInverse.setIdentity();
  oc_.inoMeasCovLLT.matrixL().solveInPlace(oc_.inoMeasCovInverse);
  oc_.inoMeasCovLLT.matrixL().transpose().solveInPlace(oc_.inoMeasCovInverse);

  // innovation

  oc_.kGain.noalias() = oc_.pbar * (c_.transpose() * oc_.inoMeasCovInverse);

  if (withInnovation)
  {
    innovation_.noalias() = oc_.kGain * oc_.inoMeas;
  }
  else
  {
    innovation_ = Vector::Zero(xbar_().size());
  }

  // update

  arithm_->stateSum(xbar_(), innovation_, oc_.xhat);

#ifndef VERBOUS_KALMANFILTER
  Eigen::IOFormat CleanFmt(2, 0, " ", "\n", "", "");
  std::cout << "A" << std::endl << a_.format(CleanFmt) << std::endl;
  std::cout << "C" << std::endl << c_.format(CleanFmt) << std::endl;
  std::cout << "P" << std::endl << pr_.format(CleanFmt) << std::endl;
  std::cout << "K" << std::endl << oc_.kGain.format(CleanFmt) << std::endl;
  std::cout << "Xbar" << std::endl << xbar_().transpose().format(CleanFmt) << std::endl;
  std::cout << "inoMeasCov" << std::endl << oc_.inoMeasCov.format(CleanFmt) << std::endl;
  std::cout << "oc_.pbar" << std::endl << (oc_.pbar).format(CleanFmt) << std::endl;
  std::cout << "c_ * (oc_.pbar * c_.transpose())" << std::endl
            << (c_ * (oc_.pbar * c_.transpose())).format(CleanFmt) << std::endl;
  std::cout << "inoMeasCovInverse" << std::endl << oc_.inoMeasCovInverse.format(CleanFmt) << std::endl;
  std::cout << "predictedMeasurement " << std::endl << ybar_().transpose().format(CleanFmt) << std::endl;
  std::cout << "inoMeas" << std::endl << oc_.inoMeas.transpose().format(CleanFmt) << std::endl;
  std::cout << "inovation_" << std::endl << innovation_.transpose().format(CleanFmt) << std::endl;
  std::cout << "Xhat" << std::endl << oc_.xhat.transpose().format(CleanFmt) << std::endl;
#endif // VERBOUS_KALMANFILTER

  this->x_.set(oc_.xhat, k + 1);
  pr_.noalias() = -oc_.kGain * c_;
  pr_.diagonal().array() += 1;
  Matrix IKc = pr_;
  pr_ *= oc_.pbar;
  IKc = IKc * oc_.pbar * IKc.transpose() + oc_.kGain * r_ * oc_.kGain.transpose();
  // simmetrize the pr_ matrix
  pr_ = (pr_ + pr_.transpose()) * 0.5;
  IKc = (IKc + IKc.transpose()) * 0.5;

  Matrix diff = IKc - pr_;
  double max = 0;
  double minRatio = 1000.0;
  int maxRow = 0;
  int maxCol = 0;
  double diffMinRatio = 0.0;
  double covMinRatio = 0.0;
  double covMaxRatio = 0.0;
  int rowMinRatio = 0.0;
  int colMinRatio = 0.0;
  std::cout << std::endl << "Q" << std::endl << (q_).format(CleanFmt) << std::endl;
  std::cout << std::endl << "R" << std::endl << (r_).format(CleanFmt) << std::endl;
  std::cout << std::endl << "Diff" << std::endl << (diff).format(CleanFmt) << std::endl;
  for (int i = 0; i<diff.rows(); i++)
  {
    for (int j = 0; j < diff.cols(); j++)
    {
      if (abs(diff(i,j)) > max)
      {
        max = diff(i,j);
        maxRow = i;
        maxCol = j;
      }
      if (diff(i,j) != 0 && std::min(abs(pr_(i, j)), abs(IKc(i, j))) / abs(diff(i,j)) < minRatio)
      {
        minRatio = std::min(abs(pr_(i, j)), abs(IKc(i, j))) / abs(diff(i,j));
        rowMinRatio = i;
        colMinRatio = j;
        covMinRatio = std::min(abs(pr_(i, j)), abs(IKc(i, j)));
        covMaxRatio = std::max(abs(pr_(i, j)), abs(IKc(i, j)));
        diffMinRatio = abs(diff(i,j));
      }
    }
  }
  /*
  std::cout << std::endl << "Positions x : " << "pr_ : " << pr_(0,0) << "  .  IKc" << IKc(0,0) << "  .  pr_ - IKc" << pr_(0,0) - IKc(0,0) << std::endl;
  std::cout << std::endl << "Max : " << max << "    on index : [" << std::to_string(maxRow) << ", " << std::to_string(maxCol) << "]" << std::endl;
  std::cout << std::endl << "Ratio on this index : " << std::endl << std::min(pr_(maxRow, maxCol), IKc(maxRow, maxCol)) / max << " .    Diff at that index : " << diff(maxRow, maxCol) << " .  Min covariance at that index : " << std::min(pr_(maxRow, maxCol), IKc(maxRow, maxCol)) << " .  Max covariance at that index : " << std::max(pr_(maxRow, maxCol), IKc(maxRow, maxCol)) << std::endl;
  std::cout << std::endl << "Min ratio : " << minRatio << " on index : [" << std::to_string(rowMinRatio) << ", " << std::to_string(colMinRatio) << "].    Diff at that index : " << diffMinRatio << " .  Min covariance at that index : " << covMinRatio << " .  Max covariance at that index : " << covMaxRatio << std::endl;
  */
  return oc_.xhat;
}

KalmanFilterBase::Pmatrix KalmanFilterBase::getStateCovariance() const
{
  return pr_;
}

void KalmanFilterBase::reset()
{
  ZeroDelayObserver::reset();

  clearStateCovariance();
}

KalmanFilterBase::Amatrix KalmanFilterBase::getAmatrixConstant(double c) const
{
  return Amatrix::Constant(nt_, nt_, c);
}

KalmanFilterBase::Amatrix KalmanFilterBase::getAmatrixRandom() const
{
  return tools::ProbabilityLawSimulation::getUniformMatrix<Matrix>(nt_, nt_);
}

KalmanFilterBase::Amatrix KalmanFilterBase::getAmatrixZero() const
{
  return Amatrix::Zero(nt_, nt_);
}

KalmanFilterBase::Amatrix KalmanFilterBase::getAmatrixIdentity() const
{
  return Amatrix::Identity(nt_, nt_);
}

bool KalmanFilterBase::checkAmatrix(const Amatrix & a) const
{
  return (a.rows() == nt_ && a.cols() == nt_);
}

KalmanFilterBase::Cmatrix KalmanFilterBase::getCmatrixConstant(double c) const
{
  return Cmatrix::Constant(mt_, nt_, c);
}

KalmanFilterBase::Cmatrix KalmanFilterBase::getCmatrixRandom() const
{
  return tools::ProbabilityLawSimulation::getUniformMatrix<Cmatrix>(mt_, nt_);
}

KalmanFilterBase::Cmatrix KalmanFilterBase::getCmatrixZero() const
{
  return Cmatrix::Zero(mt_, nt_);
}

bool KalmanFilterBase::checkCmatrix(const Cmatrix & a) const
{
  return (a.rows() == mt_ && a.cols() == nt_);
}

KalmanFilterBase::Qmatrix KalmanFilterBase::getQmatrixConstant(double c) const
{
  return Qmatrix::Constant(nt_, nt_, c);
}

KalmanFilterBase::Qmatrix KalmanFilterBase::getQmatrixRandom() const
{
  return tools::ProbabilityLawSimulation::getUniformMatrix<Qmatrix>(nt_, nt_);
}

KalmanFilterBase::Qmatrix KalmanFilterBase::getQmatrixZero() const
{
  return Qmatrix::Zero(nt_, nt_);
}

KalmanFilterBase::Qmatrix KalmanFilterBase::getQmatrixIdentity() const
{
  return Qmatrix::Identity(nt_, nt_);
}

bool KalmanFilterBase::checkQmatrix(const Qmatrix & a) const
{
  return (a.rows() == nt_ && a.cols() == nt_);
}

KalmanFilterBase::Rmatrix KalmanFilterBase::getRmatrixConstant(double c) const
{
  return Cmatrix::Constant(mt_, mt_, c);
}

KalmanFilterBase::Rmatrix KalmanFilterBase::getRmatrixRandom() const
{
  return tools::ProbabilityLawSimulation::getUniformMatrix<Cmatrix>(mt_, mt_);
}

KalmanFilterBase::Rmatrix KalmanFilterBase::getRmatrixZero() const
{
  return Rmatrix::Zero(mt_, mt_);
}

KalmanFilterBase::Rmatrix KalmanFilterBase::getRmatrixIdentity() const
{
  return Rmatrix::Identity(mt_, mt_);
}

bool KalmanFilterBase::checkRmatrix(const Rmatrix & a) const
{
  return (a.rows() == mt_ && a.cols() == mt_);
}

KalmanFilterBase::Pmatrix KalmanFilterBase::getPmatrixConstant(double c) const
{
  return Pmatrix::Constant(nt_, nt_, c);
}

KalmanFilterBase::Pmatrix KalmanFilterBase::getPmatrixRandom() const
{
  return tools::ProbabilityLawSimulation::getUniformMatrix<Pmatrix>(nt_, nt_);
}

KalmanFilterBase::Pmatrix KalmanFilterBase::getPmatrixZero() const
{
  return Pmatrix::Zero(nt_, nt_);
}

KalmanFilterBase::Pmatrix KalmanFilterBase::getPmatrixIdentity() const
{
  return Pmatrix::Identity(nt_, nt_);
}

bool KalmanFilterBase::checkPmatrix(const Pmatrix & a) const
{
  return (a.rows() == nt_ && a.cols() == nt_);
}

KalmanFilterBase::StateVectorTan KalmanFilterBase::stateTangentVectorConstant(double c) const
{
  return StateVectorTan::Constant(nt_, 1, c);
}

KalmanFilterBase::StateVectorTan KalmanFilterBase::stateTangentVectorRandom() const
{
  return tools::ProbabilityLawSimulation::getUniformMatrix<StateVectorTan>(nt_);
}

KalmanFilterBase::StateVectorTan KalmanFilterBase::stateTangentVectorZero() const
{
  return StateVectorTan::Zero(nt_, 1);
}

bool KalmanFilterBase::checkStateTangentVector(const KalmanFilterBase::StateVectorTan & v) const
{
  return (v.rows() == nt_ && v.cols() == 1);
}

KalmanFilterBase::MeasureVectorTan KalmanFilterBase::measureTangentVectorConstant(double c) const
{
  return MeasureVectorTan::Constant(mt_, 1, c);
}

KalmanFilterBase::MeasureVectorTan KalmanFilterBase::measureTangentVectorRandom() const
{
  return tools::ProbabilityLawSimulation::getUniformMatrix<MeasureVectorTan>(mt_, 1);
}

KalmanFilterBase::MeasureVectorTan KalmanFilterBase::measureTangentVectorZero() const
{
  return MeasureVectorTan::Zero(mt_, 1);
}

bool KalmanFilterBase::checkMeasureTangentVector(const KalmanFilterBase::MeasureVectorTan & v) const
{
  return (v.rows() == mt_ && v.cols() == 1);
}

void KalmanFilterBase::setStateSize(Index n)
{
  setStateSize(n, n);
}

void KalmanFilterBase::setStateSize(Index n, Index nt)
{
  if((n != n_) || (nt_ != nt))
  {
    ZeroDelayObserver::setStateSize(n);

    nt_ = nt;

    clearA();
    clearC();
    clearQ();
    clearStateCovariance();
  }
}

void KalmanFilterBase::setMeasureSize(Index m)
{
  setMeasureSize(m, m);
}

void KalmanFilterBase::setMeasureSize(Index m, Index mt)
{
  if((m != m_) || mt != mt_)
  {
    mt_ = mt;
    ZeroDelayObserver::setMeasureSize(m);
    clearC();
    clearR();
  }
}

Vector KalmanFilterBase::getSimulatedMeasurement(TimeIndex k)
{
  return simulateSensor_(getEstimatedState(k), k);
}

Vector KalmanFilterBase::getInnovation()
{
  return innovation_;
}

Vector KalmanFilterBase::getLastPrediction() const
{
  return xbar_();
}

Vector KalmanFilterBase::getLastPredictedMeasurement() const
{
  return ybar_();
}

Vector KalmanFilterBase::getLastMeasurement() const
{
  return y_.back();
}

Matrix KalmanFilterBase::getLastGain() const
{
  return oc_.kGain;
}

Vector KalmanFilterBase::predictSensor_(TimeIndex k)
{
  return ybar_.set(simulateSensor_(xbar_(), k), k)();
}

void KalmanFilterBase::setStateArithmetics(StateVectorArithmetics * a)
{
  arithm_ = a;
}

} // namespace stateObservation
