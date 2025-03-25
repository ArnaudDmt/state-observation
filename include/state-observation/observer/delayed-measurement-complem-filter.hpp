#include <state-observation/observer/delayed-measurements-observer.hpp>

namespace stateObservation
{
struct IterationComplementaryFilter : public Iteration
{
  IterationComplementaryFilter(const Vector & initState, double dt) : Iteration(initState, dt) {}

  inline virtual Vector runIteration_() override
  {
    Vector dx_hat = computeStateDerivatives_();
    integrateState_(dx_hat);

    return finalState_;
  }
  virtual void startNewIteration_() override = 0;

  virtual Vector computeStateDerivatives_() = 0;
  /// @brief integrates the given dx into the given state.
  /// @param dx_hat The state increment to integrate
  virtual void integrateState_(Vector & dx_hat) = 0;
};

class STATE_OBSERVATION_DLLAPI DelayedMeasurementComplemFilter
: public DelayedMeasurementObserver<IterationComplementaryFilter>
{
public:
  /// The constructor
  ///  \li n : size of the state vector
  ///  \li m : size of the measurements vector
  ///  \li dt  : timestep between each iteration
  ///  \li p : size of the input vector
  DelayedMeasurementComplemFilter(Index n, Index m, double dt, Index p = 0)
  : DelayedMeasurementObserver<IterationComplementaryFilter>(n, m, dt, p)
  {
  }
  /// The constructor
  ///  \li n : size of the state vector
  ///  \li m : size of the measurements vector
  ///  \li p : size of the input vector
  ///  \li dt  : timestep between each iteration
  ///  \li bufferCapacity  : capacity of the iteration buffer
  DelayedMeasurementComplemFilter(Index n, Index m, double dt, unsigned long bufferCapacity, Index p = 0)
  : DelayedMeasurementObserver<IterationComplementaryFilter>(n, m, dt, bufferCapacity, p)
  {
  }

protected:
  virtual StateVector oneStepEstimation_() = 0;
};
} // namespace stateObservation