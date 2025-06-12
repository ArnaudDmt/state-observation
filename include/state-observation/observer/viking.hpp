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

class VikingInput : public InputBase
{
public:
  VikingInput(const Vector3 & yv, const Vector3 & ya, const Vector3 & yg) : yv_k(yv), ya_k(ya), yg_k(yg) {}
  // local linear velocity measurement
  Vector3 yv_k;
  // accelerometer measurement
  Vector3 ya_k;
  // gyrometer measurement
  Vector3 yg_k;
  // timestep
  double dt_;
};

struct AsynchronousInputViking : public AsynchronousDataBase
{
public:
  AsynchronousInputViking() {}
  /// @brief constructor with the position and orientation measurements, and the associated correction gains.
  /// @param pos position measurement.
  /// @param ori orientation measurement.
  /// @param mu gain associated with the linear velocity correction from the pose measurement.
  /// @param lambda gain associated with the yaw correction from the orientation measurement.
  /// @param tau gain associated with the tilt correction from the orientation measurement.
  /// @param eta gain associated with the position correction from the pose measurement.
  AsynchronousInputViking(const Vector3 & pos, const Matrix3 & ori, double mu, double lambda, double tau, double eta)
  {
    pos_ori_measurements_.push_back(PosOriMeas_Gain(pos, ori, mu, lambda, tau, eta));
  }
  /// @brief constructor with an orientation measurement, and the associated correction gains.
  /// @param ori orientation measurement.
  /// @param lambda gain associated with the yaw correction from the orientation measurement.
  /// @param tau gain associated with the tilt correction from the orientation measurement.
  AsynchronousInputViking(const Matrix3 & ori, double lambda, double tau)
  {
    ori_measurements_.push_back(OriMeas_Gain(ori, lambda, tau));
  }

  ~AsynchronousInputViking() {}
  inline void merge(const AsynchronousDataBase & input2) override
  {
    const AsynchronousInputViking & async_input2 = static_cast<const AsynchronousInputViking &>(input2);
    pos_ori_measurements_.insert(pos_ori_measurements_.end(), async_input2.pos_ori_measurements_.begin(),
                                 async_input2.pos_ori_measurements_.end());
    ori_measurements_.insert(ori_measurements_.end(), async_input2.ori_measurements_.begin(),
                             async_input2.ori_measurements_.end());
  }

  // position and orientation measurement, with associated gains mu, lambda, tau and eta.
  typedef std::tuple<Vector3, Matrix3, double, double, double, double> PosOriMeas_Gain;

  // orientation measurement, with associated gains lambda and tau.
  typedef std::tuple<Matrix3, double, double> OriMeas_Gain;

  std::vector<PosOriMeas_Gain> pos_ori_measurements_;
  std::vector<OriMeas_Gain> ori_measurements_;
};

/**
 * \class  Viking
 * \brief
 *
 */

class STATE_OBSERVATION_DLLAPI Viking : public DelayedMeasurementComplemFilter
{
public:
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
  ///  \li gamma  : parameter related to the orthogonality
  ///  \li rho  : parameter related to the correction of the bias by the linear velocity measurement.
  ///  \li dt  : timestep between each iteration
  ///  \li bufferCapacity  : capacity of the iteration buffer
  ///  \li withGyroBias  : indicates if the gyrometer bias must be used in the estimation
  Viking(double dt,
         double alpha,
         double beta,
         double gamma,
         double rho,
         unsigned long bufferCapacity,
         bool withGyroBias);

  /// @brief Destroy the Kinetics Observer
  ///
  virtual ~Viking();

  /// @brief initializes the state vector.
  /// @param x1 The initial local linear velocity of the IMU.
  /// @param x2_p The initial value of the intermediate estimate of the IMU's tilt.
  /// @param x2 The initial tilt of the IMU.
  void initEstimator(const Vector3 & x1,
                     const Vector3 & x2,
                     const Vector3 & gyro_bias,
                     const Vector4 & ori,
                     const Vector3 & pos);

  using DelayedMeasurementObserver::initEstimator;

  /// @brief sets the measurement
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

  using DelayedMeasurementObserver::setInput;

  /// @brief adds a delayed global pose measurement to the correction
  /// @param posMeasurement measured position in the world
  /// @param oriMeasurement measured orientation in the world
  /// @param mu gain associated with the linear velocity correction from the pose measurement.
  /// @param lambda gain associated with the yaw correction from the orientation measurement.
  /// @param tau gain associated with the tilt correction from the orientation measurement.
  /// @param eta gain associated with the position correction from the pose measurement.
  /// @param delay number of iterations ellapsed between the measurements acquisition and the current iteration.
  void addDelayedPosOriMeasurement(const Vector3 & posMeasurement,
                                   const Matrix3 & oriMeasurement,
                                   double mu,
                                   double lambda,
                                   double tau,
                                   double eta,
                                   double delay);

  /// @brief adds a delayed global orientation measurement to the correction
  /// @param oriMeasurement measured orientation in the world
  /// @param lambda gain associated with the yaw correction from the orientation measurement.
  /// @param tau gain associated with the tilt correction from the orientation measurement.
  /// @param delay number of iterations ellapsed between the measurements acquisition and the current iteration.
  void addDelayedOriMeasurement(const Matrix3 & meas, double lambda, double tau, double delay);

  /// @brief adds a global pose measurement to the correction
  /// @param posMeasurement measured position in the world
  /// @param oriMeasurement measured orientation in the world
  /// @param mu gain associated with the linear velocity correction from the pose measurement.
  /// @param lambda gain associated with the yaw correction from the orientation measurement.
  /// @param tau gain associated with the tilt correction from the orientation measurement.
  /// @param eta gain associated with the position correction from the pose measurement.
  void addPosOriMeasurement(const Vector3 & posMeasurement,
                            const Matrix3 & oriMeasurement,
                            double mu,
                            double lambda,
                            double tau,
                            double eta);

  /// @brief adds a global pose measurement to the correction
  /// @param oriMeasurement measured orientation in the world
  /// @param lambda gain associated with the yaw correction from the orientation measurement.
  /// @param tau gain associated with the tilt correction from the orientation measurement.
  void addOriMeasurement(const Matrix3 & oriMeasurement, double lambda, double tau);

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

  /// set gamma
  void setGamma(const double gamma)
  {
    gamma_ = gamma;
  }
  double getGamma()
  {
    return gamma_;
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

  Eigen::VectorBlock<ObserverBase::StateVector, sizeX1> getEstimatedLocLinVel(StateIterator it = {})
  {
    if(it == StateIterator{})
    {
      it = xBuffer_.begin();
    }
    return (*it)().segment<sizeX1>(x1Index);
  }
  Eigen::VectorBlock<ObserverBase::StateVector, sizeX2> getEstimatedTilt(StateIterator it = {})
  {
    if(it == StateIterator{})
    {
      it = xBuffer_.begin();
    }
    return (*it)().segment<sizeX2>(x2Index);
  }
  Eigen::VectorBlock<ObserverBase::StateVector, sizeGyroBias> getEstimatedGyroBias(StateIterator it = {})
  {
    if(it == StateIterator{})
    {
      it = xBuffer_.begin();
    }
    return (*it)().segment<sizeGyroBias>(gyroBiasIndex);
  }
  Eigen::VectorBlock<ObserverBase::StateVector, sizeOri> getEstimatedOrientation(StateIterator it = {})
  {
    if(it == StateIterator{})
    {
      it = xBuffer_.begin();
    }
    return (*it)().segment<sizeOri>(oriIndex);
  }
  Eigen::VectorBlock<ObserverBase::StateVector, sizePos> getEstimatedLocPosition(StateIterator it = {})
  {
    if(it == StateIterator{})
    {
      it = xBuffer_.begin();
    }
    return (*it)().segment<sizePos>(posIndex);
  }

protected:
  /// @brief Runs one loop of the estimator.
  /// @details Calls \ref computeStateDynamics_ then \ref integrateState_
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  StateVector oneStepEstimation_(StateIterator it) override;

  /// @brief Computes the dynamics of the state at the desired iteration.
  /// @details Computes x^{dot}_{k-1}
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  StateVector & computeStateDynamics_(StateIterator it) override;

  /// @brief Integrates the computed state dynamics
  /// @details Computes x_{k} = x_{k-1} + x^{dot}_{k-1} * dt
  /// @param it Iterator that points to the updated state. Points to x_{k} = f(x_{k-1}, u_{k-1})
  void integrateState_(StateIterator it) override;

  /// @brief Add the correction terms coming from the input to the computed state dynamics
  void addCorrectionTerms(StateIterator it);
  void startNewIteration_() override;

protected:
  /// The parameters of the estimator
  ///  \li alpha : parameter related to the convergence of the linear velocity
  ///              of the IMU expressed in the control frame
  ///  \li beta  : parameter related to the fast convergence of the tilt
  ///  \li gamma  : parameter related to the orthogonality
  ///  \li rho  : parameter related to the correction of the bias by the linear velocity measurement.
  double alpha_, beta_, gamma_, rho_;
  kine::LocalKinematics state_kine_;
  bool withGyroBias_;
};

} // namespace stateObservation

#endif // VikingHPP
