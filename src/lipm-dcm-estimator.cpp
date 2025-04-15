#include <state-observation/dynamics-estimators/lipm-dcm-estimator.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

namespace stateObservation
{

constexpr double LipmDcmEstimator::defaultDCMUncertainty;
constexpr double LipmDcmEstimator::defaultBiasUncertainty;

constexpr double LipmDcmEstimator::defaultDt_;
constexpr double LipmDcmEstimator::defaultOmega_;

/// default expected drift of the bias every second
constexpr double LipmDcmEstimator::defaultBiasDriftSecond;

/// default error in the estimation of the sensors
constexpr double LipmDcmEstimator::defaultZmpErrorStd;
constexpr double LipmDcmEstimator::defaultDcmErrorStd;

constexpr double LipmDcmEstimator::defaultBiasLimit;

using namespace tools;

LipmDcmEstimator::LipmDcmEstimator(double dt,
                                   double omega_0,
                                   double biasDriftStd,
                                   double dcmMeasureErrorStd,
                                   double zmpMeasureErrorStd,
                                   const Vector2 & biasLimit,
                                   const Vector2 & initZMP,
                                   const Vector2 & initDcm,
                                   const Vector2 & initBias,
                                   const Vector2 & initDcmUncertainty,
                                   const Vector2 & initBiasUncertainty)
: omega0_(omega_0), dt_(dt), biasDriftStd_(biasDriftStd), zmpErrorStd_(zmpMeasureErrorStd), previousZmp_(initZMP),
  biasLimit_(biasLimit), filter_(4, 2, 4), previousOrientation_(Matrix2::Identity())
{
  A_.setIdentity();
  B_.setZero();
  Q_.setIdentity();
  updateMatricesABQ_();
  C_ << Matrix2::Identity(), Matrix2::Identity();
  R_ = dblToSqDiag_(dcmMeasureErrorStd);
  filter_.setC(C_);
  filter_.setMeasurementCovariance(R_);
  Vector4 x;
  x << initDcm, initBias;
  filter_.setState(x, 0);
  Matrix4 P;
  // clang-format off
  P<< Vec2ToSqDiag_(initDcmUncertainty),  Matrix2::Zero(),
      Matrix2::Zero(),                   Vec2ToSqDiag_(initBiasUncertainty);
  // clang-format on
  filter_.setStateCovariance(P);
}

void LipmDcmEstimator::resetWithMeasurements(const Vector2 & measuredDcm,
                                             const Vector2 & measuredZMP,
                                             const Matrix2 & yaw,
                                             bool measurementIsWithBias,
                                             const Vector2 & initBias,
                                             const Vector2 & initBiasuncertainty)

{
  filter_.reset();
  previousZmp_ = measuredZMP;
  previousOrientation_ = yaw;

  Vector4 x;

  /// initialize the state using the measurement
  if(measurementIsWithBias)
  {
    x << measuredDcm - initBias, initBias;
  }
  else
  {
    x << measuredDcm, initBias;
  }

  filter_.setState(x, 0);
  Matrix4 P;

  if(measurementIsWithBias)
  {
    Matrix2 initBiasCov = Vec2ToSqDiag_(initBiasuncertainty);
    /// The state and the
    // clang-format off
    P<< initBiasCov+R_, -initBiasCov,
       -initBiasCov,     initBiasCov;
    // clang-format on
  }
  else
  {
    // clang-format off
    P<< R_,               Matrix2::Zero(),
        Matrix2::Zero(),  Vec2ToSqDiag_(initBiasuncertainty);
    // clang-format on
  }

  filter_.setStateCovariance(P);
}

LipmDcmEstimator::~LipmDcmEstimator() {}

void LipmDcmEstimator::setLipmNaturalFrequency(double omega_0)
{
  omega0_ = omega_0;
  needUpdateMatrices_ = true;
}

void LipmDcmEstimator::setUnbiasedCoMOffset(const Vector2 & gamma)
{
  if(gamma != gamma_) /// this test is because many expected calls to this function are with the default value
  {
    gamma_ = gamma;
    needUpdateMatrices_ = true;
  }
}

void LipmDcmEstimator::setZMPCoef(double kappa)
{
  if(kappa != kappa_) /// this test is because many expected calls to this function are with the default value
  {
    kappa_ = kappa;
    needUpdateMatrices_ = true;
  }
}

void LipmDcmEstimator::setSamplingTime(double dt)
{
  dt_ = dt;
  needUpdateMatrices_ = true;
}

void LipmDcmEstimator::setBias(const Vector2 & bias)
{
  Vector4 x = filter_.getCurrentEstimatedState();
  /// update the bias
  x.tail<2>() = bias;
  filter_.setCurrentState(x);
}

void LipmDcmEstimator::setBias(const Vector2 & bias, const Vector2 & uncertainty)
{
  setBias(bias);
  Matrix4 P = filter_.getStateCovariance();
  /// resetting the non diagonal parts
  P.topRightCorner<2, 2>().setZero();
  P.bottomLeftCorner<2, 2>().setZero();
  P.bottomRightCorner<2, 2>() = Vec2ToSqDiag_(uncertainty);
  filter_.setStateCovariance(P);
}

void LipmDcmEstimator::setBiasDriftPerSecond(double driftPerSecond)
{
  /// update the corresponding part in the process noise matrix
  Q_.bottomRightCorner<2, 2>() = dblToSqDiag_(driftPerSecond);
  filter_.setProcessCovariance(Q_);
}

void LipmDcmEstimator::setBiasLimit(const Vector2 & biasLimit)
{
  biasLimit_ = biasLimit;
}

void LipmDcmEstimator::setUnbiasedDCM(const Vector2 & dcm)
{
  Vector4 x = filter_.getCurrentEstimatedState();
  /// update the bias
  x.head<2>() = dcm;
  filter_.setCurrentState(x);
}

void LipmDcmEstimator::setUnbiasedDCM(const Vector2 & dcm, const Vector2 & uncertainty)
{
  setUnbiasedDCM(dcm);
  Matrix4 P = filter_.getStateCovariance();
  /// resetting the non diagonal parts
  P.topRightCorner<2, 2>().setZero();
  P.bottomLeftCorner<2, 2>().setZero();
  P.topLeftCorner<2, 2>() = Vec2ToSqDiag_(uncertainty);
  filter_.setStateCovariance(P);
}

void LipmDcmEstimator::setZmpMeasureErrorStd(double std)
{
  zmpErrorStd_ = std;
  needUpdateMatrices_ = true;
}

void LipmDcmEstimator::setDcmMeasureErrorStd(double std)
{
  Matrix2 R;
  R = dblToSqDiag_(std);
}

void LipmDcmEstimator::update()
{
  updateMatricesABQ_();
  filter_.estimateState();
  if(biasLimit_.x() >= 0 || biasLimit_.y() >= 0)
  {
    Vector2 localBias = getLocalBias();
    Vector2 clampedLocalBias;
    if(biasLimit_.x() >= 0)
    {
      clampedLocalBias.x() = tools::clampScalar(localBias.x(), biasLimit_.x());
    }
    if(biasLimit_.y() >= 0)
    {
      clampedLocalBias.y() = tools::clampScalar(localBias.y(), biasLimit_.y());
    }

    setBias(previousOrientation_ * clampedLocalBias);
    setUnbiasedDCM(getUnbiasedDCM() + localBias - clampedLocalBias);
  }
}

void LipmDcmEstimator::setInputs(const Vector2 & dcm,
                                 const Vector2 & zmp,
                                 const Matrix2 & orientation,
                                 const Vector2 & CoMOffset_gamma,
                                 const double ZMPCoef_kappa)
{
  setUnbiasedCoMOffset(CoMOffset_gamma);
  setZMPCoef(ZMPCoef_kappa);
  if(filter_.stateIsSet())
  {
    if(filter_.getMeasurementsNumber() > 1)
    {
      update(); /// update the estimation of the state to synchronize with the measurements
    }

    Vector u;
    u.resize(previousZmp_.size() + gamma_.size());
    Vector2 y;

    y = dcm;

    /// The prediction of the state depends on the previous value of the ZMP
    u << previousZmp_, gamma_;
    previousZmp_ = zmp;

    filter_.pushMeasurement(y);
    filter_.pushInput(u);

    A_.bottomRightCorner<2, 2>() = orientation * previousOrientation_.transpose(); /// set the rotation differenc
    previousOrientation_ = orientation;
    filter_.setA(A_);
  }
  else
  {
    resetWithMeasurements(dcm, zmp, orientation, true);
  }
}

Vector2 LipmDcmEstimator::getUnbiasedDCM() const
{
  return filter_.getCurrentEstimatedState().head<2>();
}

Vector2 LipmDcmEstimator::getBias() const
{
  return filter_.getCurrentEstimatedState().tail<2>();
}

void LipmDcmEstimator::updateMatricesABQ_()
{
  if(needUpdateMatrices_)
  {
    /// We only modify a corner to avoid resetting the orientation
    A_.diagonal().head<2>().setConstant(1 + omega0_ * dt_);

    B_.topLeftCorner<2, 2>().diagonal().setConstant(-omega0_ * dt_ * kappa_);
    B_.topRightCorner<2, 2>().diagonal().setConstant(omega0_ * dt_);

    Q_.diagonal().head<2>().setConstant(square(omega0_ * dt_ * zmpErrorStd_));
    Q_.diagonal().tail<2>().setConstant(square(biasDriftStd_ * dt_));

    /// no need to perform filter_.setA(A_) since it is done in setInputs
    filter_.setB(B_);
    filter_.setProcessCovariance(Q_);
    needUpdateMatrices_ = false;
  }
}

} // namespace stateObservation
