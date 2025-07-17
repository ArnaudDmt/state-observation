/**
 * \file      waiko-humanoid.hpp
 * \author    Arnaud Demont, Mehdi Benallegue, Abdelaziz Benallegue
 * \date       2025
 *
 * \details Implementation of waiko for humanoid robots (with contact orientations)
 *
 *
 */

#ifndef WaikoHumanoidHPP
#define WaikoHumanoidHPP

#include "state-observation/observer/zero-delay-observer.hpp"
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
    /// @brief Input coming from a contact: IMU pose with the associated correction gains
    struct ContactInput
    {
      /// @param pos position measurement.
      /// @param ori orientation measurement.
      /// @param mu gain associated with the linear velocity correction from the pose measurement.
      /// @param lambda gain associated with the yaw correction from the orientation measurement.
      /// @param tau gain associated with the tilt correction from the orientation measurement.
      /// @param eta gain associated with the position correction from the pose measurement.
      ContactInput(const Matrix3 & ori,
                   const Vector3 & pos,
                   const Vector3 & imuContactPos,
                   const Vector3 & worldContactPos,
                   double mu,
                   double lambda,
                   double gamma)
      : ori_(ori), pos_(pos), imuContactPos_(imuContactPos), worldContactPos_(worldContactPos), mu_(mu),
        lambda_(lambda), gamma_(gamma)
      {
      }
      Matrix3 ori_;
      Vector3 pos_;
      Vector3 imuContactPos_;
      Vector3 worldContactPos_;
      double mu_;
      double lambda_;
      double gamma_;
      // double eta_;
    };
    InputWaiko(const Vector3 & yv, const Vector3 & ya, const Vector3 & yg) : yv_(yv), ya_(ya), yg_(yg) {}

    // local linear velocity measurement
    Vector3 yv_;
    // accelerometer measurement
    Vector3 ya_;
    // gyrometer measurement
    Vector3 yg_;
    // IMU position measurements from contacts
    std::vector<ContactInput> contact_inputs_;
  };

  inline static constexpr Index sizeX1 = 3;
  inline static constexpr Index sizeX2 = 3;
  inline static constexpr Index sizeGyroBias = 3;
  inline static constexpr Index sizeOri = 4;
  inline static constexpr Index sizePos = 3;

  inline static constexpr Index sizeX1Tangent = 3;
  inline static constexpr Index sizeX2Tangent = 3;
  inline static constexpr Index sizeGyroBiasTangent = 3;
  inline static constexpr Index sizeOriTangent = 3;
  inline static constexpr Index sizePosTangent = 3;

  inline static constexpr Index x1Index = 0;
  inline static constexpr Index x2Index = sizeX1;
  inline static constexpr Index gyroBiasIndex = x2Index + sizeX2;
  inline static constexpr Index oriIndex = gyroBiasIndex + sizeGyroBias;
  inline static constexpr Index posIndex = oriIndex + sizeOri;

  inline static constexpr Index x1IndexTangent = 0;
  inline static constexpr Index x2IndexTangent = sizeX1Tangent;
  inline static constexpr Index gyroBiasIndexTangent = x2IndexTangent + sizeX2Tangent;
  inline static constexpr Index oriIndexTangent = gyroBiasIndexTangent + sizeGyroBiasTangent;
  inline static constexpr Index posIndexTangent = oriIndexTangent + sizeOriTangent;

public:
  /// The constructor
  ///  \li alpha : parameter related to the convergence of the linear velocity
  ///              of the IMU expressed in the control frame
  ///  \li beta  : parameter related to the fast convergence of the tilt
  ///  \li rho  : parameter related to the orthogonality
  ////  \li gamma  : parameter related to the correction of the bias by the linear velocity measurement.
  ///  \li dt  : timestep between each iteration
  ///  \li bufferCapacity  : capacity of the iteration buffer
  ///  \li withGyroBias  : indicates if the gyrometer bias must be used in the estimation
  WaikoHumanoid(double dt, double alpha, double beta, double rho, bool withGyroBias = true);

  /// @brief Destroys the observer
  ///
  virtual ~WaikoHumanoid();

  /// @brief initializes the state vector.
  /// @param x1 The initial local linear velocity of the IMU.
  /// @param x2_p The initial value of the intermediate estimate of the IMU's tilt.
  /// @param x2 The initial tilt of the IMU.
  void initEstimator(const Vector3 & x1,
                     const Vector3 & x2,
                     const Vector3 & gyro_bias,
                     const Vector4 & ori,
                     const Vector3 & pos);

  /// @brief sets the input
  /// @details version that computes yv from the kinematics of the anchor frame in the IMU frame
  /// @param yv_k
  /// @param ya_k.
  /// @param yg_k
  /// @param k
  /// @param resetImuLocVelHat Resets x1hat (the estimate of the local linear velocity of the IMU in the world). Avoid
  /// discontinuities when the computation mode of the anchor point changes
  void setInput(const Vector3 & imuAnchorPos,
                const Vector3 & imuAnchorLinVel,
                const Vector3 & ya_k,
                const Vector3 & yg_k,
                TimeIndex k,
                bool resetImuLocVelHat = false);

  /// @brief sets the input
  /// @param yv_k
  /// @param ya_k.
  /// @param yg_k
  /// @param k
  /// @param resetImuLocVelHat Resets x1hat (the estimate of the local linear velocity of the IMU in the world). Avoid
  /// discontinuities when the computation mode of the anchor point changes
  void setInput(const Vector3 & yv_k,
                const Vector3 & ya_k,
                const Vector3 & yg_k,
                TimeIndex k,
                bool resetImuLocVelHat = false);

  /// @brief sets the input from a contact
  void addContactInput(const InputWaiko::ContactInput & contactInput, TimeIndex k);

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

  /// set gamma
  void setGamma(const double gamma)
  {
    gamma_ = gamma;
  }
  double getGamma()
  {
    return gamma_;
  }

  /// set the sampling time of the measurements
  void setSamplingTime(double dt)
  {
    dt_ = dt;
  }
  double getSamplingTime() const
  {
    return dt_;
  }

  /// set the sampling time of the measurements
  void setWithGyroBias(bool withGyroBias)
  {
    withGyroBias_ = withGyroBias;
  }

  const Eigen::VectorBlock<ObserverBase::StateVector, sizeX1> getEstimatedLocLinVel()
  {
    return x_().segment<sizeX1>(x1Index);
  }
  Eigen::VectorBlock<ObserverBase::StateVector, sizeX2> getEstimatedTilt()
  {
    return x_().segment<sizeX2>(x2Index);
  }
  Eigen::VectorBlock<ObserverBase::StateVector, sizeGyroBias> getEstimatedGyroBias()
  {
    return x_().segment<sizeGyroBias>(gyroBiasIndex);
  }
  Eigen::VectorBlock<ObserverBase::StateVector, sizeOri> getEstimatedOrientation()
  {
    return x_().segment<sizeOri>(oriIndex);
  }
  Eigen::VectorBlock<ObserverBase::StateVector, sizePos> getEstimatedLocPosition()
  {
    return x_().segment<sizePos>(posIndex);
  }

  // correction of the position coming from the contact positions, passed as a local linear velocity.
  inline const stateObservation::Vector3 & getPosCorrectionFromContactPos()
  {
    return posCorrFromContactPos_;
  }
  // correction of the orientation coming from the contact positions, passed as a local angular velocity.
  inline const stateObservation::Vector3 & geOriCorrectionFromContactPos()
  {
    return oriCorrFromContactPos_;
  }
  // correction of the orientation coming from direct orientation measurements, passed as a local angular velocity.
  inline const stateObservation::Vector3 & getOriCorrFromOriMeas()
  {
    return oriCorrFromOriMeas_;
  }

protected:
  /// @brief Runs one loop of the estimator.
  /// @details Calls \ref computeStateDynamics_ then \ref integrateState_
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  StateVector oneStepEstimation_() override;

  /// @brief Computes the dynamics of the state at the desired iteration.
  /// @details Computes x^{dot}_{k-1}
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  StateVector & computeStateDynamics_();

  /// @brief Integrates the computed state dynamics
  /// @details Computes x_{k} = x_{k-1} + x^{dot}_{k-1} * dt
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  void integrateState_();

  /// @brief Add the correction terms coming from the input to the computed state dynamics
  void addCorrectionTerms();
  void startNewIteration_();

protected:
  /// The parameters of the estimator
  ///  \li alpha : parameter related to the convergence of the linear velocity
  ///              of the IMU expressed in the control frame
  ///  \li beta  : parameter related to the fast convergence of the tilt
  ///  \li rho  : parameter related to the orthogonality
  ///  \li gamma  : parameter related to the correction of the bias by the linear velocity measurement.
  double alpha_, beta_, rho_, gamma_;
  bool withGyroBias_;
  Vector dx_hat_;
  kine::Orientation state_ori_;
  // kine::LocalKinematics state_kine_;
  //  sampling time
  double dt_;

  // correction of the orientation coming from the contact orientations, passed as a local angular velocity.
  Vector3 oriCorrFromOriMeas_ = Vector3::Zero();
  // correction of the position coming from the contact positions, passed as a local linear velocity.
  Vector3 posCorrFromContactPos_ = Vector3::Zero();
  // correction of the orientation coming from the contact positions, passed as a local angular velocity.
  Vector3 oriCorrFromContactPos_ = Vector3::Zero();
};

} // namespace stateObservation

#endif // WaikoHumanoidHPP
