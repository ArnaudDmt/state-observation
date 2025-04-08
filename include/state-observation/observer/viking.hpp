/**
 * \file      tilt-estimator.hpp
 * \author    Arnaud Demont, Mehdi Benallegue
 * \date       2018
 * \brief      Version of the Tilt Estimator that implements all the necessary functions to perform the estimation for
 * humanoid robots.
 *
 * \details
 *
 *
 */

#ifndef VikingHPP
#define VikingHPP

#include <state-observation/observer/delayed-measurements-complem-filter.hpp>
// #include <state-observation/tools/delayed-measurements-observer-iteration.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace stateObservation
{

struct InputViking
{
  // orientation measurement with the associated correction gain
  typedef std::pair<Matrix3, double> OriMeas_Gain;
  // position measurement coming from a contact, with the associated correction gains, for the position and the
  // orientation, respectively.
  typedef std::tuple<Vector6, double, double> ContactPosMeas_Gains;

  // orientation measurements from contacts
  std::vector<OriMeas_Gain> ori_measurements_;
  // position measurements from contacts = posMeasurement (in the world) << imuContactPos
  std::vector<ContactPosMeas_Gains> pos_measurements_from_contact_;
};
// struct IterationViking : public IterationComplementaryFilter
// {
//   IterationViking(const Vector & initState, const kine::Kinematics & initPose, double dt);

//   /// Default constructor
//   IterationViking() = delete;

//   virtual ~IterationViking();

//   Vector computeStateDerivatives_() override;
//   /// @brief integrates the given dx into the given state.
//   /// @param dx_hat The state increment to integrate
//   void integrateState_(const Vector & dx_hat) override;

//   Vector & runIteration_() override;

//   /// @brief adds the correction from a direct measurement of the IMU's frame orientation.
//   /// @param meas measured orientation of the IMU's frame in the world
//   /// @param gain weight of the correction
//   void addOrientationMeasurement(const Matrix3 & meas, double gain);

//   /// @brief adds the correction from a contact position measurement
//   /// @param posMeasurement measured position of the contact in the world
//   /// @param imuContactPos position of the contact in the imu's frame.
//   /// @param gainDelta weight of the position correction
//   /// @param gainSigma weight of the orientation correction
//   void addContactPosMeasurement(const Vector3 & posMeasurement,
//                                 const Vector3 & imuContactPos,
//                                 double gainDelta,
//                                 double gainSigma);

// public:
//   /// The parameters of the estimator
//   double alpha_, beta_, rho_;
//   /// Estimated pose of the IMU at the beginning of the iteration
//   kine::Kinematics initPose_;
//   /// Estimated pose of the IMU at the end of the iteration
//   kine::Kinematics finalPose_;

//   // correction of the orientation passed as a local angular velocity
//   Vector3 sigma_ = Vector3::Zero();
//   // correction of the orientation coming from orientation measurements, passed as a local angular velocity.
//   Vector3 oriCorrFromOriMeas_ = Vector3::Zero();
//   // correction of the position coming from the contact positions, passed as a local linear velocity.
//   Vector3 posCorrFromContactPos_ = Vector3::Zero();
//   // correction of the orientation coming from the contact positions, passed as a local angular velocity.
//   Vector3 oriCorrFromContactPos_ = Vector3::Zero();

//   TimeIndex k_contacts_ = 0; // time index of the contact measurements
// };

/**
 * \class  Viking
 * \brief  Version of the Tilt Estimator for humanoid robots.
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

  /// @brief adds the correction from a direct measurement of the IMU's frame orientation.
  /// @param meas measured orientation of the IMU's frame in the world
  /// @param gain weight of the correction
  void addOrientationMeasurement(const Matrix3 & meas, double gain);

  /// @brief adds the correction from a contact position measurement
  /// @param posMeasurement measured position of the contact in the world
  /// @param imuContactPos position of the contact in the imu's frame.
  /// @param gainDelta weight of the position correction
  /// @param gainSigma weight of the orientation correction
  void addContactPosMeasurement(const Vector3 & posMeasurement,
                                const Vector3 & imuContactPos,
                                double gainDelta,
                                double gainSigma);

  /// @brief replays a previous iteration with an additional orientation measurement.
  /// @param delay delay between the iteration receiving the measurement and the current one.
  /// @param meas measured orientation of the IMU's frame in the world
  /// @param gain weight of the correction
  StateVector replayIterationWithDelayedOri(unsigned long delay, const Matrix3 & meas, double gain);

  /// @brief replays a previous iteration with an additional orientation measurement and applies the obtained correction
  /// to the current state.
  /// @param delay delay between the iteration receiving the measurement and the current one.
  /// @param meas measured orientation of the IMU's frame in the world
  /// @param gain weight of the correction
  StateVector replayIterationsWithDelayedOri(unsigned long delay, const Matrix3 & meas, double gain);

  // Vector3 getVirtualLocalVelocityMeasurement()
  // {
  //   return x1_;
  // }

  /// set the sampling time of the measurements
  // void setBufferCapacity(unsigned long bufferCapacity) override
  // {
  //   withDelayedOri_ = true;
  //   bufferedIters_.set_capacity(bufferCapacity);
  // }

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

  // inline const boost::circular_buffer & getIterationsBuffer() const
  // {
  //   return bufferedIters_;
  // }

  // returns the correction term applied on the estimated orientation

  // inline const stateObservation::Vector3 & getOriCorrection()
  // {
  //   return getCurrentIter().sigma_;
  // }
  // // correction of the position coming from the contact positions, passed as a local linear velocity.
  // inline const stateObservation::Vector3 & getPosCorrectionFromContactPos()
  // {
  //   return getCurrentIter().posCorrFromContactPos_;
  // }
  // // correction of the orientation coming from the contact positions, passed as a local angular velocity.
  // inline const stateObservation::Vector3 & geOriCorrectionFromContactPos()
  // {
  //   return getCurrentIter().oriCorrFromContactPos_;
  // }
  // // correction of the orientation coming from direct orientation measurements, passed as a local angular velocity.
  // inline const stateObservation::Vector3 & getOriCorrFromOriMeas()
  // {
  //   return getCurrentIter().oriCorrFromOriMeas_;
  // }

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
