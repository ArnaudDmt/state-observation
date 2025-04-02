#ifndef DelayedMeasurementComplemFilterHPP
#define DelayedMeasurementComplemFilterHPP

#include <state-observation/observer/delayed-measurements-observer.hpp>

namespace stateObservation
{

template<typename IterationT>
class STATE_OBSERVATION_DLLAPI DelayedMeasurementComplemFilter : public DelayedMeasurementObserver<IterationT>
{
public:
  /// The constructor
  ///  \li n : size of the state vector
  ///  \li m : size of the measurements vector
  ///  \li dt  : timestep between each iteration
  ///  \li p : size of the input vector
  DelayedMeasurementComplemFilter(double dt, Index n, Index m, Index p = 0)
  : DelayedMeasurementObserver<IterationT>(dt, n, m, p)
  {
  }
  /// The constructor
  ///  \li n : size of the state vector
  ///  \li m : size of the measurements vector
  ///  \li p : size of the input vector
  ///  \li dt  : timestep between each iteration
  ///  \li bufferCapacity  : capacity of the iteration buffer
  DelayedMeasurementComplemFilter(double dt, Index n, Index m, unsigned long bufferCapacity, Index p = 0)
  : DelayedMeasurementObserver<IterationT>(dt, n, m, bufferCapacity, p)
  {
  }

  DelayedMeasurementComplemFilter() = delete;

  /// Destructor
  virtual ~DelayedMeasurementComplemFilter(){};

protected:
  virtual Vector oneStepEstimation_() = 0;
};
} // namespace stateObservation

#endif // DelayedMeasurementComplemFilterHPP