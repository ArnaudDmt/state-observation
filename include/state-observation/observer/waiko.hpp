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

#ifndef WaikoHPP
#define WaikoHPP

#include "state-observation/observer/tilt-estimator-humanoid.hpp"

#include <boost/circular_buffer.hpp>
#include <state-observation/api.h>

namespace stateObservation
{

/**
 * \class  Waiko
 * \brief  Version of the Tilt Estimator for humanoid robots.
 *
 */
class STATE_OBSERVATION_DLLAPI Waiko : public ZeroDelayObserver
{
  typedef kine::Orientation Orientation;

protected:
  struct Iteration
  {
    Iteration(double alpha, double beta, double rho, double dt) : dt_(dt), alpha_(alpha), beta_(beta), rho_(rho) {}

    // add an orientation measurement of the IMU's frame in the world frame to the correction
    void addOrientationMeasurement(const Matrix3 & meas, double gain);

    // add the correction coming from a contact position to the estimation
    void addContactPosMeasurement(const Vector3 & posMeasurement,
                                  const Vector3 & imuContactPos,
                                  double gainDelta,
                                  double gainSigma);

    StateVector replayBufferedIteration();

    Eigen::Matrix<double, 12, 1> computeStateDerivatives();

    /// @brief integrates the given dx into the given state.
    /// @param dx_hat The state increment to integrate
    void integrateState(const Eigen::Matrix<double, 12, 1> & dx_hat);

    void startNewIteration();
    void resetCorrectionTerms();

    inline void saveMeasurement(const ObserverBase::MeasureVector & y_k)
    {
      y_k_ = y_k;
    }

    /// Sampling time
    double dt_;
    /// The parameters of the estimator
    double alpha_, beta_, rho_;
    /// Estimated pose of the IMU at the beginning of the iteration
    kine::Kinematics initPose_;
    // state at time k-1
    ObserverBase::StateVector initState_;
    // updated state (at the end of the iteration)
    ObserverBase::StateVector updatedState_;
    /// Estimated pose of the IMU at the end of the iteration
    kine::Kinematics updatedPose_;
    // measurements at time k
    ObserverBase::MeasureVector y_k_;

    // correction of the orientation passed as a local angular velocity
    Vector3 sigma_ = Vector3::Zero();
    // correction of the orientation coming from orientation measurements, passed as a local angular velocity.
    Vector3 oriCorrFromOriMeas_ = Vector3::Zero();
    // correction of the position coming from the contact positions, passed as a local linear velocity.
    Vector3 posCorrFromContactPos_ = Vector3::Zero();
    // correction of the orientation coming from the contact positions, passed as a local angular velocity.
    Vector3 oriCorrFromContactPos_ = Vector3::Zero();

    TimeIndex k_est_ = 0; // time index of the last estimation
    TimeIndex k_data_ = 0; // time index of the current measurements
    TimeIndex k_contacts_ = 0; // time index of the contact measurements
  };

private:
  Iteration currentIter_;

public:
  /// The constructor
  ///  \li alpha : parameter related to the convergence of the linear velocity
  ///              of the IMU expressed in the control frame
  ///  \li beta  : parameter related to the fast convergence of the tilt
  ///  \li rho  : parameter related to the orthogonality
  ///  \li dt  : timestep between each iteration
  Waiko(double alpha, double beta, double rho, double dt);

  /// The constructor
  ///  \li alpha : parameter related to the convergence of the linear velocity
  ///              of the IMU expressed in the control frame
  ///  \li beta  : parameter related to the fast convergence of the tilt
  ///  \li rho  : parameter related to the orthogonality
  ///  \li dt  : timestep between each iteration
  ///  \li dt  : capacity of the iteration buffer
  Waiko(double alpha, double beta, double rho, double dt, unsigned long bufferCapacity);

  inline Iteration & getCurrentIter()
  {
    return currentIter_;
  }
  /// @brief initializes the state vector.
  /// @param x1 The initial local linear velocity of the IMU.
  /// @param x2_p The initial value of the intermediate estimate of the IMU's tilt.
  /// @param x2 The initial tilt of the IMU.
  void initEstimator(const Vector3 & pos = Vector3::Zero(),
                     const Vector3 & x1 = Vector3::Zero(),
                     const Vector3 & x2_prime = Vector3::UnitZ(),
                     const Vector4 & R = Vector4(0, 0, 0, 1));

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

  void setMeasurement(const ObserverBase::MeasureVector & y_k, TimeIndex k) override;

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

  Vector3 getVirtualLocalVelocityMeasurement()
  {
    return x1_;
  }

  /// set the sampling time of the measurements
  virtual void setBufferCapacity(unsigned long bufferCapacity)
  {
    withDelayedOri_ = true;
    bufferedIters_.set_capacity(bufferCapacity);
  }
  double getWithDelayedOri() const
  {
    return withDelayedOri_;
  }

  /// set the sampling time of the measurements
  virtual void setSamplingTime(const double dt)
  {
    getCurrentIter().dt_ = dt;
  }
  double getSamplingTime()
  {
    return getCurrentIter().dt_;
  }

  /// set the gain of x1_hat variable
  void setAlpha(const double alpha)
  {
    getCurrentIter().alpha_ = alpha;
  }
  double getAlpha()
  {
    return getCurrentIter().alpha_;
  }

  /// set the gain of x2prime_hat variable
  void setBeta(const double beta)
  {
    getCurrentIter().beta_ = beta;
  }
  double getBeta()
  {
    return getCurrentIter().beta_;
  }

  /// set rho
  void setRho(const double rho)
  {
    getCurrentIter().rho_ = rho;
  }
  double getRho()
  {
    return getCurrentIter().rho_;
  }

  inline const boost::circular_buffer<std::unique_ptr<Iteration>> & getIterationsBuffer() const
  {
    return bufferedIters_;
  }

  // returns the correction term applied on the estimated orientation
  inline const stateObservation::Vector3 & getOriCorrection()
  {
    return getCurrentIter().sigma_;
  }
  // correction of the position coming from the contact positions, passed as a local linear velocity.
  inline const stateObservation::Vector3 & getPosCorrectionFromContactPos()
  {
    return getCurrentIter().posCorrFromContactPos_;
  }
  // correction of the orientation coming from the contact positions, passed as a local angular velocity.
  inline const stateObservation::Vector3 & geOriCorrectionFromContactPos()
  {
    return getCurrentIter().oriCorrFromContactPos_;
  }
  // correction of the orientation coming from direct orientation measurements, passed as a local angular velocity.
  inline const stateObservation::Vector3 & getOriCorrFromOriMeas()
  {
    return getCurrentIter().oriCorrFromOriMeas_;
  }

protected:
  /// Orientation estimator loop
  StateVector oneStepEstimation_() override;

protected:
  // indicates if the estimator will be used along a source of delayed orientation measurements.
  bool withDelayedOri_;

  /// variables used for the computation
  Vector3 x1_;

  boost::circular_buffer<std::unique_ptr<Iteration>> bufferedIters_;
};

} // namespace stateObservation

#endif // WaikoHPP
