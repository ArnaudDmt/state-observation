#ifndef DelayedMeasurementComplemFilterHPP
#define DelayedMeasurementComplemFilterHPP

#include <state-observation/observer/delayed-measurements-observer.hpp>

namespace stateObservation
{

class STATE_OBSERVATION_DLLAPI DelayedMeasurementComplemFilter : public DelayedMeasurementObserver
{
public:
  /// The constructor
  ///  \li n : size of the state vector
  ///  \li m : size of the measurements vector
  ///  \li p : size of the input vector
  ///  \li dt  : timestep between each iteration
  ///  \li bufferCapacity  : capacity of the iteration buffer
  DelayedMeasurementComplemFilter(double dt,
                                  Index n,
                                  Index dx_size,
                                  Index m,
                                  unsigned long bufferCapacity,
                                  const std::shared_ptr<IndexedInputArrayInterface> input = nullptr,
                                  const std::shared_ptr<AsynchronousDataMapBase> async_input = nullptr,
                                  const std::shared_ptr<AsynchronousDataMapBase> async_meas = nullptr)
  : DelayedMeasurementObserver(dt, n, m, bufferCapacity, input, async_input, async_meas)
  {
    dx_hat_.resize(dx_size);
  }

  DelayedMeasurementComplemFilter() = delete;

  /// Destructor
  virtual ~DelayedMeasurementComplemFilter(){};

protected:
  /// @brief Runs one loop of the estimator.
  /// @details Calls \ref computeStateDerivatives_ then \ref integrateState_
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  virtual StateVector oneStepEstimation_(StateIterator it) override = 0;

  /// @brief Computes the dynamics of the state at the desired iteration.
  /// @details Computes x^{dot}_{k-1}
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  virtual StateVector & computeStateDynamics_(StateIterator it) = 0;

  /// @brief Integrates the computed state dynamics
  /// @details Computes x_{k} = x_{k-1} + x^{dot}_{k-1} * dt
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  virtual void integrateState_(StateIterator it) = 0;

  Vector dx_hat_;
};
} // namespace stateObservation

#endif // DelayedMeasurementComplemFilterHPP