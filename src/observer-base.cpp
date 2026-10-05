#include <state-observation/observer/observer-base.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>

namespace stateObservation
{

ObserverBase::ObserverBase()
{
  n_ = m_ = 0;
}

ObserverBase::ObserverBase(Index n, Index m) : n_(n), m_(m) {}

ObserverBase::StateVector ObserverBase::stateVectorConstant(double c) const
{
  return StateVector::Constant(n_, 1, c);
}

ObserverBase::StateVector ObserverBase::stateVectorRandom() const
{
  return tools::ProbabilityLawSimulation::getUniformMatrix<StateVector>(n_);
}

ObserverBase::StateVector ObserverBase::stateVectorZero() const
{
  return StateVector::Zero(n_, 1);
}

bool ObserverBase::checkStateVector(const StateVector & v) const
{
  return (v.rows() == n_ && v.cols() == 1);
}

ObserverBase::MeasureVector ObserverBase::measureVectorConstant(double c) const
{
  return MeasureVector::Constant(m_, 1, c);
}

ObserverBase::MeasureVector ObserverBase::measureVectorRandom() const
{
  return tools::ProbabilityLawSimulation::getUniformMatrix<MeasureVector>(m_);
}

ObserverBase::MeasureVector ObserverBase::measureVectorZero() const
{
  return MeasureVector::Zero(m_, 1);
}

bool ObserverBase::checkMeasureVector(const MeasureVector & v) const
{
  return (v.rows() == m_ && v.cols() == 1);
}

void ObserverBase::clearInputsAndMeasurements()
{
  clearMeasurements();
  clearInputs();
}

void ObserverBase::reset()
{
  clearStates();
  clearMeasurements();
  clearInputs();
}

void ObserverBase::setStateSize(Index n)
{
  n_ = n;
}

Index ObserverBase::getStateSize() const
{
  return n_;
}

void ObserverBase::setMeasureSize(Index m)
{
  m_ = m;
}

Index ObserverBase::getMeasureSize() const
{
  return m_;
}

} // namespace stateObservation
