/**
 * \file      viking.hpp
 * \author    Arnaud Demont, Mehdi Benallegue, Abdelaziz Benallegue
 * \date       2025
 *
 * \details
 *
 *
 */

#ifndef VikingHPP
#define VikingHPP

#include <state-observation/observer/delayed-measurements-complem-filter.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace stateObservation
{

struct AsynchronousInputViking : public AsynchronousDataBase
{
public:
  AsynchronousInputViking() {}
  AsynchronousInputViking(const Vector3 & pos, const Matrix3 & ori, double gain)
  {
    pos_ori_measurements_.push_back(PosOriMeas_Gain(pos, ori, gain));
  }
  ~AsynchronousInputViking() {}
  inline void merge(const AsynchronousDataBase & input2) override
  {
    const AsynchronousInputViking & async_input2 = static_cast<const AsynchronousInputViking &>(input2);
    pos_ori_measurements_.insert(pos_ori_measurements_.end(), async_input2.pos_ori_measurements_.begin(),
                                 async_input2.pos_ori_measurements_.end());
  }

  typedef std::tuple<Vector3, Matrix3, double> PosOriMeas_Gain;
  // position and orientation measurement, with associated gain

  std::vector<PosOriMeas_Gain> pos_ori_measurements_;
};

/**
 * \class  Viking
 * \brief
 *
 */

class STATE_OBSERVATION_DLLAPI Viking : public DelayedMeasurementComplemFilter
{
public:
  /// The constructor
  ///  \li alpha : parameter related to the convergence of the linear velocity
  ///              of the IMU expressed in the control frame
  ///  \li beta  : parameter related to the fast convergence of the tilt
  ///  \li rho  : parameter related to the orthogonality
  ///  \li dt  : timestep between each iteration
  ///  \li dt  : capacity of the iteration buffer
  Viking(double dt, double alpha, double beta, double rho, unsigned long bufferCapacity);

  /// @brief Destroy the Kinetics Observer
  ///
  virtual ~Viking();

  /// @brief initializes the state vector.
  /// @param x1 The initial local linear velocity of the IMU.
  /// @param x2_p The initial value of the intermediate estimate of the IMU's tilt.
  /// @param x2 The initial tilt of the IMU.
  void initEstimator(const Vector3 & x1 = Vector3::Zero(),
                     const Vector3 & x2_prime = Vector3::UnitZ(),
                     const Vector3 & pos = Vector3::Zero(),
                     const Vector4 & R = Vector4(0, 0, 0, 1));

  using DelayedMeasurementObserver::initEstimator;

  /// @brief sets the measurement
  /// @param yv_k
  /// @param ya_k.
  /// @param yg_k
  /// @param k
  /// @param resetImuLocVelHat Resets x1hat (the estimate of the local linear velocity of the IMU in the world). Avoid
  /// discontinuities when the computation mode of the anchor point changes
  void setMeasurement(const Vector3 & yv_k,
                      const Vector3 & ya_k,
                      const Vector3 & yg_k,
                      TimeIndex k,
                      bool resetImuLocVelHat = false);

  using DelayedMeasurementObserver::setMeasurement;

  /// @brief adds a delayed global pose measurement to the correction
  /// @param posMeasurement measured position in the world
  /// @param oriMeasurement measured orientation in the world
  /// @param gainDelta weight of the correction
  void addDelayedPosOriMeasurement(const Vector3 & posMeasurement, const Matrix3 & meas, double gain, TimeIndex delay);

  /// @brief adds a global pose measurement to the correction
  /// @param posMeasurement measured position in the world
  /// @param oriMeasurement measured orientation in the world
  /// @param gainDelta weight of the correction
  void addPosOriMeasurement(const Vector3 & posMeasurement, const Matrix3 & meas, double gain);

  /// set the gain of x1_hat variable
  void setAlpha(const double alpha)
  {
    alpha_ = alpha;
  }
  double getAlpha()
  {
    return alpha_;
  }

  /// set the gain of x2prime_hat variable
  void setBeta(const double beta)
  {
    beta_ = beta;
  }
  double getBeta()
  {
    return beta_;
  }

  /// set rho
  void setRho(const double rho)
  {
    rho_ = rho;
  }
  double getRho()
  {
    return rho_;
  }

protected:
  /// @brief Runs one loop of the estimator.
  /// @details Calls \ref computeStateDerivatives_ then \ref integrateState_
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  StateVector oneStepEstimation_(StateIterator it) override;

  /// @brief Computes the dynamics of the state at the desired iteration.
  /// @details Computes x^{dot}_{k-1}
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  StateVector computeStateDynamics_(StateIterator it) override;

  /// @brief Integrates the computed state dynamics
  /// @details Computes x_{k} = x_{k-1} + x^{dot}_{k-1} * dt
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  void integrateState_(StateIterator it, const Vector & dx_hat) override;

  /// @brief Computes the correction terms, used to compute the state dynamics in \ref computeStateDerivatives_
  void computeCorrectionTerms(StateIterator it);
  void startNewIteration_() override;

protected:
  /// The parameters of the estimator
  double alpha_, beta_, rho_;

  Vector3 posCorrection_;
  Vector3 oriCorrection_;
};

} // namespace stateObservation

#endif // VikingHPP
