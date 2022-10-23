
///\file      lipm-dcm-estimator.hpp
///\author    Mehdi Benallegue
///\date      2020
///\brief     Filtering of divergent component of motion (DCM) and estimation of a bias betweeen the DCM
///           and the corresponding zero moment point for a linearized inverted
///           pendulum model
///
///\detail
///
///
#ifndef LIPMDCMBIASESTIMATOR_HPP
#define LIPMDCMBIASESTIMATOR_HPP

#include <state-observation/api.h>
#include <state-observation/observer/linear-kalman-filter.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace stateObservation
{

/// \class LipmDcmEstimator
/// \brief Filtering of divergent component of motion (DCM) and estimation of a bias betweeen the DCM
///        and the corresponding zero moment point for a linearized inverted pendulum model.
///
/// \details
/// A humanoid robot can be modeled as an inverted pendulum. The dynamics can be
/// linearized to obtain a dynamics with a convergent and a divergent component of motion (DCN).
/// The dynamics of the DCM depends on the Zero Moment Point.
/// The DCM can be measured using the CoM and its velocity, but the CoM position can be biased.
/// For instance if the measurement of the CoM is biased, or in presence of an external force
/// the dynamics of the DCM is biased \f$\dot{\hat{\xi}} & =\omega_{0}(\hat{\xi}-\kappa z+\detla-b)\f$ where
/// \f$\hat{\xi}$\f$ is the measured dcm, and \f$\gamma$\f$ and \f$\kappa$\f$ are known CoM offset and ZMP coefficient,
/// and \f$b$\f$ is the unknown bias. Also the measurement of the DCM can be noisy (since it uses CoM velocity
/// estimation).
///
/// This estimator uses Kalman Filtering to estimate this bias and give a delay-free filtering of the DCM.
/// The theorerical details are available at \ref lipm_dcm_details
///
/// Inputs and outputs:
///  - Inputs are (every itneration)
///     - measurements of ZMP
///     - biased measurements DCM
///     - The orientation of the robot
///  - Outputs
///      - Estimation of the bias (the bias is considered almost constant in the local frame of the robot)
///      - Filtered DCM
///
/// Tuning:
/// This estimator has a few parameters
///  - Initial uncertainties
///     @param initBiasUncertainty tunes how fast the bias converges initially. Too High values will make the bias
///     overshoot.
///     @param initDcmUncertainty tunes how good is the initial guess. Changing it will reduce the initial filtering
///     on the DCM until the steady convergence
///  - Measurement uncertainties
///     @param dcmMeasureErrorStd tunes how filtered is the DCM signal. Lower values give less DCM filtering
///     @param zmpMeasureErrorStd tunes how much the model can be trusted. If the measurements are noisy or if the
///     LIPM model does not well represent the dynamics then higher values are recommended. Lower values increase DCM
///     filtering
///  - Other parameters
///     @param biasDriftPerSecondStd tunes how fast the bias changes value during the motion. Higher values will make
///     the bias move more during the mosion
///     @param biaslLimit clamps the values of Drift between x and -x on X axis and between y and -y on Y axis.
///
///  Note that besides biaslLimit all the parameters don't change behavior with scaling : if we multiply all of
///  them by the same positive factor the behavior would be the same. So the ratios between these parameters is the
///  actual parameter

class STATE_OBSERVATION_DLLAPI LipmDcmEstimator
{
private:
  constexpr static double defaultDt_ = 0.005;
  constexpr static double defaultOmega_ = tools::sqrt(cst::gravityConstant);

public:
  /// default expected drift of the bias every second
  constexpr static double defaultBiasDriftSecond = 0.002;

  /// default error in the estimation of the sensors
  constexpr static double defaultZmpErrorStd = 0.005;
  constexpr static double defaultDcmErrorStd = 0.01;

  /// default uncertainty in the initial values of DCM and Bias
  constexpr static double defaultDCMUncertainty = 0.01;
  constexpr static double defaultBiasUncertainty = 0.01;

  /// default value for Bias limit (negative means limitless)
  constexpr static double defaultBiasLimit = -1;

  /// @brief Construct a new Lipm Dcm Estimator object
  /// @details Use this if no DCM measurements are available or when a good guess of its unbiased position is
  /// available
  ///
  /// @param dt                     the sampling time in seconds
  /// @param omega_0                the natural frequency of the DCM (rad/s)
  /// @param biasDriftPerSecondStd  the standard deviation of the bias drift (m/s)
  /// @param initZMP                the initial value of the DCM
  /// @param initDCM                the initial value of the DCM
  /// @param initBias               the initial value of the bias
  /// @param dcmMeasureErrorStd     the standard deviation of the dcm estimation error, NOT including the bias (m)
  /// @param zmpMeasureErrorStd     the standard deviaiton of the zmp estimation error (m)
  /// @param biasLimit              the X and Y (expressed in local frame) largest accepted absolute values of the bias
  ///                               (zero means no limit)
  /// @param initDcmUncertainty     the uncertainty in the DCM initial value in meters
  /// @param initBiasUncertainty    the uncertainty in the bias initial value in meters
  LipmDcmEstimator(double dt = defaultDt_,
                   double omega_0 = defaultOmega_,
                   double biasDriftPerSecondStd = defaultBiasDriftSecond,
                   double dcmMeasureErrorStd = defaultDcmErrorStd,
                   double zmpMeasureErrorStd = defaultZmpErrorStd,
                   const Vector2 & biasLimit = Vector2::Constant(defaultBiasLimit),
                   const Vector2 & initZMP = Vector2::Zero(),
                   const Vector2 & initDcm = Vector2::Zero(),
                   const Vector2 & initBias = Vector2::Zero(),
                   const Vector2 & initDcmUncertainty = Vector2::Constant(defaultDCMUncertainty),
                   const Vector2 & initBiasUncertainty = Vector2::Constant(defaultBiasUncertainty));

  /// @brief Resets the estimator with first measurements
  /// @details Use this when initializing with an available DCM (biased / or not) measurement
  ///
  /// @param measuredDcm            the the measured position of the DCM in the world frame
  /// @param measuredZMP            the the measured position of the ZMP in the world frame
  /// @param yaw                    the initial yaw angle in the form of a rotation matrix
  /// @param measurementIsWithBias  sets if yes or no the first measurement is biased
  /// @param biasLimit              the X and Y (expressed in local frame) largest accepted absolute values of the bias
  ///                               (zero means no limit)
  /// @param initBias               the initial value of the drift
  /// @param initBiasuncertainty    the uncertainty in the bias initial value in meters
  void resetWithMeasurements(const Vector2 & measuredDcm,
                             const Vector2 & measuredZMP,
                             const Matrix2 & yaw = Matrix2::Identity(),
                             bool measurementIsWithBias = true,
                             const Vector2 & initBias = Vector2::Constant(0),
                             const Vector2 & initBiasuncertainty = Vector2::Constant(defaultBiasUncertainty));

  /// @brief Resets the estimator with first measurements
  /// @details Use this when initializing with an available DCM (biased / or not) measurement
  ///
  /// @param measuredDcm            the the measured position of the DCM in the world frame
  /// @param measuredZMP            the the measured position of the ZMP in the world frame
  /// @param yaw                    the initial yaw angle
  /// @param measurementIsWithBias  sets if yes or no the first measurement is biased
  /// @param biasLimit              the X and Y (expressed in local frame) largest accepted absolute values of the bias
  ///                               (zero means no limit)
  /// @param initBias               the initial value of the drift
  /// @param initBiasuncertainty    the uncertainty in the bias initial value in meters
  inline void resetWithMeasurements(const Vector2 & measuredDcm,
                                    const Vector2 & measuredZMP,
                                    double yaw = 0,
                                    bool measurementIsWithBias = true,
                                    const Vector2 & initBias = Vector2::Constant(0),
                                    const Vector2 & initBiasuncertainty = Vector2::Constant(defaultBiasUncertainty))
  {
    resetWithMeasurements(measuredDcm, measuredZMP, Rotation2D(yaw).toRotationMatrix(), measurementIsWithBias, initBias,
                          initBiasuncertainty);
  }

  /// @brief Resets the estimator with first measurements
  /// @details Use this when initializing with an available DCM (biased / or not) measurement
  ///
  /// @param measuredDcm            the the measured position of the DCM in the world frame
  /// @param measuredZMP            the the measured position of the ZMP in the world frame
  /// @param rotation                the 3d orientation from which the initial yaw angle will be extracted using the
  /// angle agnostic approach. This orientation is from local to global. i.e. bias_global == orientation * bias*local
  /// @param measurementIsWithBias  sets if yes or no the first measurement is biased
  /// @param biasLimit              the X and Y (expressed in local frame) largest accepted absolute values of the bias
  ///                               (zero means no limit)
  /// @param initBias               the initial value of the drift
  /// @param initBiasuncertainty    the uncertainty in the bias initial value in meters
  inline void resetWithMeasurements(const Vector2 & measuredDcm,
                                    const Vector2 & measuredZMP,
                                    const Matrix3 & rotation = Matrix3::Identity(),
                                    bool measurementIsWithBias = true,
                                    const Vector2 & initBias = Vector2::Constant(0),
                                    const Vector2 & initBiasuncertainty = Vector2::Constant(defaultBiasUncertainty))
  {
    resetWithMeasurements(measuredDcm, measuredZMP, kine::rotationMatrixToYawAxisAgnostic(rotation),
                          measurementIsWithBias, initBias, initBiasuncertainty);
  }

  ///@brief Destroy the Lipm Dcm Bias Estimator object
  ~LipmDcmEstimator();

  ///@brief Set the Lipm Natural Frequency
  ///
  ///@param omega_0  is the sampling time in seconds
  void setLipmNaturalFrequency(double omega_0);

  /// @brief Get the Lipm Natural Frequency
  ///
  /// @return double
  inline double getLipmNaturalFrequency() const
  {
    return omega0_;
  }

  /// @brief Set an offset on the CoM dynamics
  /// @details This is an offset that adds to the CoM and the bias in definin the DCM dynamics, it has the same effect
  /// on the dynamics as manually modifying the estimated bias, but it does not need to interfer with the estimation,
  /// especially when this offset is obtained from a different independent source (e.g. external force sensor).
  ///
  /// @param offset in meters
  void setUnbiasedCoMOffset(const Vector2 & gamma);

  /// @brief Get the ubiased offset on the CoM dynamics
  /// @details gets the value set by setUnbiasedCoMOffset
  ///
  /// @return Vector2
  Vector2 getUnbiasedCoMOffset() const
  {
    return gamma_;
  }

  /// @brief Set the ZMP oefficient
  /// @param kappa is the zmp coefficient (no unit)
  void setZMPCoef(double kappa);

  /// @brief get the ZMP Coefficient
  /// @return double
  double getZMPCoef() const
  {
    return kappa_;
  }

  ///@brief Set the Sampling Time
  ///
  ///@param dt sampling time
  void setSamplingTime(double dt);

  ///@brief Set the Bias object from a guess
  ///
  ///@param bias guess
  void setBias(const Vector2 & bias);

  ///@copydoc setBias(const Vector2 & bias)
  ///
  ///@param the uncertainty you have in this guess in meters
  void setBias(const Vector2 & bias, const Vector2 & uncertainty);

  /// @brief Set the Bias Drift Per Second
  ///
  /// @param driftPerSecond the standard deviation of the drift (m/s)
  void setBiasDriftPerSecond(double driftPerSecond);

  /// @brief Set the Bias Limit
  ///
  /// @param biasLimit the X and Y (expressed in local frame) largest accepted
  ///                   absolute values of the bias (negative means there is no limit)
  void setBiasLimit(const Vector2 & biasLimit);

  /// @brief set the unbiased DCM position from a guess
  ///
  /// @param dcm guess
  void setUnbiasedDCM(const Vector2 & dcm);

  /// @copydoc setUnbiasedDCM(double dcm)
  ///
  /// @param uncertainty the uncertainty in this guess
  void setUnbiasedDCM(const Vector2 & dcm, const Vector2 & uncertainty);

  /// @brief Set the Zmp Measurement Error Stamdard devbiation
  ///
  void setZmpMeasureErrorStd(double);

  /// @brief Set the Dcm Measurement Error Standard
  ///
  void setDcmMeasureErrorStd(double);

  /// @brief Set the Inputs of the estimator.
  /// @details The yaw will be extracted from the orientation using the axis agnostic
  /// approach.
  ///
  /// @param dcm         measurement of the DCM in the world frame
  /// @param zmp         mesaurement of the ZMP in the world frame
  /// @param orientation the 3d orientation from which the yaw will be extracted. This orientation is from local to
  /// global. i.e. bias_global == orientation * bias*local
  /// @param CoMOffset_gamma is the unbiased CoM offset in the dynamics of the DCM (see setUnbiasedCoMOffset)
  /// @param ZMPCoef_kappa is the coefficient multiplied by the ZMP in the DCM dynamics (see setZMPCoef)
  inline void setInputs(const Vector2 & dcm,
                        const Vector2 & zmp,
                        const Matrix3 & orientation,
                        const Vector2 & CoMOffset_gamma = Vector2::Zero(),
                        const double ZMPCoef_kappa = 1)
  {
    setInputs(dcm, zmp, kine::rotationMatrixToYawAxisAgnostic(orientation), CoMOffset_gamma, ZMPCoef_kappa);
  }

  /// @brief Set the Inputs of the estimator.
  ///
  /// @param dcm measurement of the DCM in the world frame
  /// @param zmp mesaurement of the ZMP in the world frame
  /// @param yaw is the yaw angle to be used. This orientation is from local to global. i.e. bias_global == R *
  /// bias*local
  /// @param CoMOffset_gamma is the unbiased CoM offset in the dynamics of the DCM (see setUnbiasedCoMOffset)
  /// @param ZMPCoef_kappa is the coefficient multiplied by the ZMP in the DCM dynamics (see setZMPCoef)
  inline void setInputs(const Vector2 & dcm,
                        const Vector2 & zmp,
                        double yaw,
                        const Vector2 & CoMOffset_gamma = Vector2::Zero(),
                        const double ZMPCoef_kappa = 1)
  {
    setInputs(dcm, zmp, Rotation2D(yaw).toRotationMatrix(), CoMOffset_gamma, ZMPCoef_kappa);
  }

  /// @brief Set the Inputs of the estimator.
  ///
  /// @param dcm  measurement of the DCM in the world frame
  /// @param zmp  mesaurement of the ZMP in the world frame
  /// @param R    the 2x2 Matrix'representing the yaw angle i.e. bias_global == R * bias*local
  /// @param CoMOffset_gamma is the unbiased CoM offset in the dynamics of the DCM (see setUnbiasedCoMOffset)
  /// @param ZMPCoef_kappa is the coefficient multiplied by the ZMP in the DCM dynamics (see setZMPCoef)
  void setInputs(const Vector2 & dcm,
                 const Vector2 & zmp,
                 const Matrix2 & R = Matrix2::Identity(),
                 const Vector2 & CoMOffset_gamma = Vector2::Zero(),
                 const double ZMPCoef_kappa = 1);

  /// @brief Runs the estimation. Needs to be called every timestep
  ///
  /// @return Vector2
  void update();

  /// @brief Get the Unbiased DCM filtered by the estimator
  ///
  /// @detailt This is the recommended output to take
  /// @return double
  Vector2 getUnbiasedDCM() const;

  /// @brief Get the estimated Bias
  ///
  /// @return double
  Vector2 getBias() const;

  /// @brief Get the estimated Bias expressed in the local frame of the robot
  ///
  /// @return double
  inline Vector2 getLocalBias() const
  {
    return previousOrientation_.transpose() * getBias();
  }

  /// @brief Get the Kalman Filter object
  /// This can be used to run specific Advanced Kalman filter related funcions
  /// @return LinearKalmanFilter&
  inline LinearKalmanFilter & getFilter()
  {
    return filter_;
  }

  /// @copydoc getFilter()
  /// const version
  inline const LinearKalmanFilter & getFilter() const
  {
    return filter_;
  }

protected:
  typedef Eigen::Matrix<double, 4, 2> Matrix42;
  typedef Eigen::Matrix<double, 2, 4> Matrix24;

  /// @brief set Matrices: A, B, Q
  void updateMatricesABQ_();

  double omega0_;
  Vector2 gamma_ = Vector2::Zero();
  double kappa_ = 1;
  double dt_;
  double biasDriftStd_;
  double zmpErrorStd_;

  bool needUpdateMatrices_ = true;

  Vector2 previousZmp_;

  Vector2 biasLimit_;

  LinearKalmanFilter filter_;
  Matrix4 A_;
  Matrix4 B_;
  /// this needs to be transposed
  Matrix24 C_;
  /// measurement noise
  Matrix2 R_;
  /// process noise
  Matrix4 Q_;

  Matrix2 previousOrientation_;

  /// @brief builds a diagonal out of the square valued of the Vec2
  inline static Matrix2 Vec2ToSqDiag_(const Vector2 & v)
  {
    return Vector2(v.array().square()).asDiagonal();
  }

  /// @brief builds a constant 2x2 diagonal from a double
  inline static Matrix2 dblToDiag_(const double & d)
  {
    return Vector2::Constant(d).asDiagonal();
  }

  /// @brief builds a constant 2x2 diagonal from a square of a double
  inline static Matrix2 dblToSqDiag_(const double & d)
  {
    return dblToDiag_(d * d);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace stateObservation

#endif /// LIPMDCMBIASESTIMATOR_HPP
