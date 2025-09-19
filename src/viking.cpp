#include "state-observation/tools/definitions.hpp"
#include "state-observation/tools/rigid-body-kinematics.hxx"
#include <cmath>
#include <iostream>
#include <state-observation/observer/viking.hpp>

namespace stateObservation
{

Viking::Viking() : ZeroDelayObserver(12, 21, std::make_shared<IndexedInputArrayT<>>()) {}

Viking::Viking(double alpha, double beta, double gamma, double mu, double rho, double dt)
: Viking(alpha, beta, gamma, mu, rho, 12, 21, dt)
{
}

Viking::Viking(double alpha, double beta, double gamma, double mu, double rho, int n, int m, double dt)
: ZeroDelayObserver(n, m, std::make_shared<IndexedInputArrayT<>>()), alpha_(alpha), beta_(beta), gamma_(gamma), mu_(mu),
  rho_(rho), dt_(dt)
{
}

void Viking::initEstimator(Vector & x)
{
  setState(x, 0);
}

void Viking::initEstimator(const Vector3 & x1,
                           const Vector3 & x2,
                           const Vector3 & b,
                           const Matrix3 & R,
                           const Vector3 & p_l)
{
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(getStateSize());
  x0.segment<3>(0) = x1;
  x0.segment<3>(3) = x2;
  x0.segment<3>(6) = b;
  R_hat_ = R;
  x0.segment<3>(9) = p_l;

  setState(x0, 0);
}

// ===== Measurement =====

void Viking::setMeasurement(const Vector3 & yv_k,
                            const Vector3 & ya_k,
                            const Vector3 & yg_k,
                            const Matrix3 & Ry_k,
                            const Vector3 & p_l_y_k,
                            TimeIndex k)
{
  // Measurement layout: [ yv(3) ; ya(3) ; yg(3) ; vec(Ry)(9 col-major) ; p_l,y(3) ]
  ObserverBase::MeasureVector y_k(21);

  y_k.segment<3>(0) = yv_k;
  y_k.segment<3>(3) = ya_k;
  y_k.segment<3>(6) = yg_k;
  Eigen::Map<const Eigen::Matrix<double, 9, 1>> Ryvec(Ry_k.data()); // column-major flatten
  y_k.segment<9>(9) = Ryvec;
  y_k.segment<3>(18) = p_l_y_k;

  ZeroDelayObserver::setMeasurement(y_k, k);
}

// ===== Core step =====

ObserverBase::StateVector Viking::oneStepEstimation_()
{
  using Vec = Eigen::VectorXd;

  const TimeIndex k = this->x_.getTime();

  BOOST_ASSERT(this->y_.size() > 0 && this->y_.checkIndex(k + 1) && "ERROR: The measurement vector is not set");

  // Unpack measurement at time k+1
  const Vec y = getMeasurement(k + 1);
  const Vector3 yv = y.segment<3>(0);
  const Vector3 ya = y.segment<3>(3);
  const Vector3 yg = y.segment<3>(6);
  Matrix3 Ry;
  {
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>> RyMap(y.segment<9>(9).data());
    Ry = RyMap;
  }
  const Vector3 p_l_y = y.segment<3>(18);

  // Current estimate x_hat(k)
  ObserverBase::StateVector xhat = getCurrentEstimatedState();

  // Unpack state
  x1_hat_ = xhat.segment<3>(0);
  x2_hat_ = xhat.segment<3>(3);
  b_hat_ = xhat.segment<3>(6);
  p_l_hat_ = xhat.segment<3>(9);

  // Precompute
  const Vector3 omega_hat = yg - b_hat_; // yg - b
  double epsilon = 1e-10;
  const double sqrt_term = std::sqrt(alpha_ * alpha_ + 4.0 * cst::gravityConstant * beta_);
  const double A = (-alpha_ + sqrt_term) / ((2.0 - epsilon) * cst::gravityConstant);
  const double C = (cst::gravityConstant * sqrt_term + alpha_) / beta_;
  const double min_gm = std::min(gamma_, mu_);

  // Error terms
  const Matrix3 Rtilde_y = Ry * R_hat_.toMatrix3().transpose();
  const Vector3 RT_ez = R_hat_.toMatrix3().transpose() * Vector3::UnitZ();
  const Vector3 logR = kine::skewSymmetricToRotationVector(Rtilde_y - Rtilde_y.transpose()) / 2.0;

  // Sigma (Eq. for sigma)
  const Vector3 sigma = gamma_ * (RT_ez.cross(x2_hat_)) - mu_ * (RT_ez * Vector3::UnitZ().dot(logR));

  // Delta (Eq. for delta)
  const Vector3 delta = -rho_ * (p_l_y - p_l_hat_);

  // Dynamics (continuous-time)
  Vector3 dx1_hat = x1_hat_.cross(omega_hat) - cst::gravityConstant * x2_hat_ + ya + alpha_ * (yv - x1_hat_);
  Vector3 dx2_hat = x2_hat_.cross(omega_hat) - beta_ * (yv - x1_hat_);

  Vector3 db_hat =
      A * x1_hat_.cross(yv) + x2_hat_.cross(yv) + x1_hat_.cross(Ry.transpose() * Vector3::UnitZ())
      + C * x2_hat_.cross(Ry.transpose() * Vector3::UnitZ())
      - 0.25 * (cst::gravityConstant * min_gm / (gamma_ * gamma_)) * (R_hat_.toMatrix3().transpose() * logR)
      - alpha_ * rho_ * sqrt_term * (p_l_y.cross(p_l_hat_));

  // std::cout << std::endl << "alpha_ * rho_ * c_ab: " << alpha_ * rho_ * c_ab << std::endl;
  // std::cout << std::endl
  //           << "-alpha_ * rho_ * c_ab * (p_l_y.cross(p_l_hat_)): "
  //           << (-alpha_ * rho_ * c_ab * (p_l_y.cross(p_l_hat_))).transpose() << std::endl;
  Vector3 dp_l_hat = x1_hat_ + p_l_hat_.cross(omega_hat) - delta;

  // Forward Euler integration
  x1_hat_ += dx1_hat * dt_;
  x2_hat_ += dx2_hat * dt_;
  b_hat_ += db_hat * dt_;
  R_hat_.integrateRightSide((omega_hat - sigma) * dt_);
  p_l_hat_ += dp_l_hat * dt_;

  // std::cout << std::endl << " p_l_hat_.transpose(): " << p_l_hat_.transpose() << std::endl;
  // std::cout << std::endl << " p_l_y.transpose(): " << p_l_y.transpose() << std::endl;

  // std::cout << std::endl << " x1_hat_.transpose(): " << x1_hat_.transpose() << std::endl;
  // std::cout << std::endl << " x2_hat_.transpose(): " << x2_hat_.transpose() << std::endl;
  // std::cout << std::endl << " b_hat_.transpose(): " << b_hat_.transpose() << std::endl;
  // std::cout << std::endl << " (omega_hat - sigma) * dt_: " << (omega_hat - sigma) * dt_ << std::endl;

  // Pack state back
  xhat.segment<3>(0) = x1_hat_;
  xhat.segment<3>(3) = x2_hat_;
  xhat.segment<3>(6) = b_hat_;
  xhat.segment<3>(9) = p_l_hat_;

  // Commit state at k+1
  setState(xhat, k + 1);

  return xhat;
}

} // namespace stateObservation
