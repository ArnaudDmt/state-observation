/**
 * \file      waiko-humanoid.hpp
 * \author    Arnaud Demont, Mehdi Benallegue, Abdelaziz Benallegue
 * \date       2025
 *
 * \details Implementation of waiko for humanoid robots (with contact
 * orientations)
 *
 *
 */

#ifndef WaikoHumanoidHPP
#define WaikoHumanoidHPP

#include "state-observation/observer/zero-delay-observer.hpp"
#include <optional>
#include <state-observation/observer/delayed-measurements-complem-filter.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace stateObservation
{

/**
 * \class  WaikoHumanoid
 * \brief
 *
 */

class STATE_OBSERVATION_DLLAPI WaikoHumanoid : public ZeroDelayObserver
{

public:
  struct InputWaiko : public InputBase
  {
    struct ContactPosInput
    {
      Vector3 pos_meas_from_contacts_ = Vector3::Zero();
      std::vector<Vector3> pos_meas_;
      std::vector<Vector3> jacobians_;
      std::vector<double> lambdas_;
    };

    InputWaiko(double dt, const Vector3 & yv, const Vector3 & ya, const Vector3 & yg)
    : dt_(dt), yv_(yv), ya_(ya), yg_(yg)
    {
    }
    // sampling time
    double dt_;

    // local linear velocity measurement
    Vector3 yv_;
    // accelerometer measurement
    Vector3 ya_;
    // gyrometer measurement
    Vector3 yg_;
    // IMU position measurements coming from contacts
    std::optional<ContactPosInput> contact_pos_input_;
    // IMU position measurements
    std::vector<Vector3> pos_inputs_;
    // IMU orientation measurements measurements
    std::vector<Matrix3> ori_inputs_;
  };

  inline static constexpr Index sizeX1 = 3;
  inline static constexpr Index sizeX2 = 3;
  inline static constexpr Index sizeOri = 4;
  inline static constexpr Index sizePos = 3;

  inline static constexpr Index sizeX1Tangent = 3;
  inline static constexpr Index sizeX2Tangent = 3;
  inline static constexpr Index sizeOriTangent = 3;
  inline static constexpr Index sizePosTangent = 3;

  inline static constexpr Index x1Index = 0;
  inline static constexpr Index x2Index = sizeX1;
  inline static constexpr Index posIndex = x2Index + sizeX2;

  inline static constexpr Index x1IndexTangent = 0;
  inline static constexpr Index x2IndexTangent = sizeX1Tangent;
  inline static constexpr Index oriIndexTangent = x2IndexTangent + sizeX2Tangent;
  inline static constexpr Index posIndexTangent = oriIndexTangent + sizeOriTangent;

public:
  /// The constructor
  ///  \li dt  : timestep between each iteration
  ///  \li alpha : parameter related to the convergence of the linear velocity
  ///              of the IMU expressed in the control frame
  ///  \li beta  : parameter related to the fast convergence of the tilt
  ///  \li gamma  : parameter related to the orthogonality
  ///  \li rho  : parameter related to the correction of the position by the
  ///  position measurement
  ///  \li mu  : parameter related to the correction of the orientation by the orientation measurement
  ///  \li psi : gain of the yaw correction from contact positions.
  WaikoHumanoid(double alpha, double beta, double gamma, double rho, double mu, double psi);

  /// @brief Destroys the observer
  ///
  virtual ~WaikoHumanoid();

  /// @brief initializes the state vector.
  /// @param x1 The initial local linear velocity of the IMU.
  /// @param x2_p The initial value of the intermediate estimate of the IMU's
  /// tilt.
  /// @param x2 The initial tilt of the IMU.
  void initEstimator(const Vector3 & x1, const Vector3 & x2, const Vector4 & ori, const Vector3 & pos);

  /// @brief sets the input
  /// @details version that computes yv from the kinematics of the anchor frame
  /// in the IMU frame
  /// @param dt sampling time
  /// @param imuAnchorPos position of the anchor point in the IMU
  /// @param imuAnchorPos linear velocity of the anchor point in the IMU
  /// @param ya_k accelerometer measurement
  /// @param yg_k gryometer measurement
  /// @param k time index
  /// @param resetImuLocVelHat Resets x1hat (the estimate of the local linear
  /// velocity of the IMU in the world). Avoid discontinuities when the
  /// computation mode of the anchor point changes
  void setInput(double dt,
                const Vector3 & imuAnchorPos,
                const Vector3 & imuAnchorLinVel,
                const Vector3 & ya_k,
                const Vector3 & yg_k,
                TimeIndex k,
                bool resetImuLocVelHat = false);

  /// @brief sets the input
  /// @param dt sampling time
  /// @param yv_k local linear velocity measurement
  /// @param ya_k accelerometer measurement
  /// @param yg_k gryometer measurement
  /// @param k time index
  /// @param resetImuLocVelHat Resets x1hat (the estimate of the local linear
  /// velocity of the IMU in the world). Avoid discontinuities when the
  /// computation mode of the anchor point changes
  void setInput(double dt,
                const Vector3 & yv_k,
                const Vector3 & ya_k,
                const Vector3 & yg_k,
                TimeIndex k,
                bool resetImuLocVelHat = false);

  /// @brief set position, orientation, or pose inputs
  void addPosInput(const Vector3 & poseInput, TimeIndex k);
  void addOriInput(const Matrix3 & oriInput, TimeIndex k);
  void addPoseInput(const Matrix3 & oriInput, const Vector3 & posInput, TimeIndex k);
  /// @brief adds a position measurement coming from contacts
  /// @details this measurement is also used to compute a correction term for
  /// the orientation
  /// @param refPose reference position of the contact
  /// @param imuContactPos position of the contact in the imu frame.
  /// @param lambda contribution gain of the contact. the lambdas of all
  /// contacts must sum up to 1.
  /// @param k time index
  void addContactPosInput(const Vector3 & refPose, const Vector3 & imuContactPos, double lambda, TimeIndex k);

  using ZeroDelayObserver::setInput;

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

  /// set lambda
  void setGamma(const double gamma)
  {
    gamma_ = gamma;
  }
  double getGamma()
  {
    return gamma_;
  }

  /// set mu
  void setMu(const double mu)
  {
    mu_ = mu;
  }
  double getMu()
  {
    return mu_;
  }

  /// set psi
  void setPsi(const double psi)
  {
    psi_ = psi;
  }
  double getPsi()
  {
    return psi_;
  }

  void setWithOriCorrectFromContactPos(bool withCorrection)
  {
    withOriCorrectFromContactPos_ = withCorrection;
  }
  bool getWithOriCorrectFromContactPos()
  {
    return withOriCorrectFromContactPos_;
  }

  const Eigen::VectorBlock<ObserverBase::StateVector, sizeX1> getEstimatedLocLinVel()
  {
    return x_().segment<sizeX1>(x1Index);
  }
  const Eigen::VectorBlock<ObserverBase::StateVector, sizeX2> getEstimatedTilt()
  {
    return x_().segment<sizeX2>(x2Index);
  }
  Matrix3 getEstimatedOrientation() const
  {
    return state_ori_.toMatrix3();
  }
  const Eigen::VectorBlock<ObserverBase::StateVector, sizePos> getEstimatedLocPosition()
  {
    return x_().segment<sizePos>(posIndex);
  }

  // correction of the position coming from the contact positions, passed as a
  // local linear velocity.
  inline const stateObservation::Vector3 & getPosCorrectionFromContactPos()
  {
    return posCorrFromContactPos_;
  }
  // correction of the orientation coming from the contact positions, passed as
  // a local angular velocity.
  inline const stateObservation::Vector3 & getOriCorrectionFromContactPos()
  {
    return oriCorrFromContactPos_;
  }
  // correction of the orientation coming from direct orientation measurements,
  // passed as a local angular velocity.
  inline const stateObservation::Vector3 & getOriCorrFromOriMeas()
  {
    return oriCorrFromOriMeas_;
  }

protected:
  /// @brief Runs one loop of the estimator.
  /// @details Calls \ref computeStateDynamics_ then \ref integrateState_
  /// @param it Iterator that points to the updated state. Points to x_{k} =
  /// f(x_{k-1}, u_{k-1})
  StateVector oneStepEstimation_() override;

  /// @brief Computes the dynamics of the state at the desired iteration.
  /// @details Computes x^{dot}_{k-1}
  /// @param it Iterator that points to the updated state. Points to x_{k} =
  /// f(x_{k-1}, u_{k-1})
  StateVector & computeStateDynamics_();

  /// @brief Integrates the computed state dynamics
  /// @details Computes x_{k} = x_{k-1} + x^{dot}_{k-1} * dt
  /// @param it Iterator that points to the updated state. Points to x_{k} =
  /// f(x_{k-1}, u_{k-1})
  void integrateState_();

  /// @brief Add the correction terms coming from the input to the computed
  /// state dynamics
  void addCorrectionTerms();
  void startNewIteration_();

protected:
  /// The parameters of the estimator
  ///  \li alpha : parameter related to the convergence of the linear velocity
  ///              of the IMU expressed in the control frame
  ///  \li beta  : parameter related to the fast convergence of the tilt
  ///  \li gamma_  : parameter related to the orthogonality
  ///  \li rho  : parameter related to the correction of the position by the
  ///  position measurement
  ///  \li mu  : parameter related to the correction of the  orientation by the orientation measurement
  ///  \li psi : gain of the yaw correction from contact positions.
  double alpha_, beta_, gamma_, rho_, mu_, psi_;
  Vector dx_hat_;
  kine::Orientation state_ori_;

  // correction of the orientation coming from the contact orientations, passed
  // as a local angular velocity.
  Vector3 oriCorrFromOriMeas_ = Vector3::Zero();
  // correction of the position coming from the contact positions, passed as a
  // local linear velocity.
  Vector3 posCorrFromContactPos_ = Vector3::Zero();
  // correction of the orientation coming from the contact positions, passed as
  // a local angular velocity.
  Vector3 oriCorrFromContactPos_ = Vector3::Zero();

  bool withOriCorrectFromContactPos_ = false;
};

} // namespace stateObservation

#endif // WaikoHumanoidHPP
