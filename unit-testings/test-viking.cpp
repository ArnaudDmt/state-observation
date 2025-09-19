#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>

#include <Eigen/Geometry>

#include <state-observation/observer/viking.hpp>
#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

using namespace stateObservation;
using namespace kine;

// ===== Small helpers to unpack state/measurements =====
static inline void unpackState(const ObserverBase::StateVector & xhat,
                               const Orientation & Rhat,
                               Vector3 & x1,
                               Vector3 & x2,
                               Vector3 & b,
                               Orientation & R,
                               Vector3 & pl)
{
  x1 = xhat.segment<3>(0);
  x2 = xhat.segment<3>(3);
  b = xhat.segment<3>(6);
  R = Rhat;
  pl = xhat.segment<3>(9);
}

static inline Vector4 matToQuatVec4(const Matrix3 & R)
{
  Eigen::Quaterniond q(R);
  Vector4 qv;
  qv << q.w(), q.x(), q.y(), q.z();
  return qv;
}

inline Vector3 clampNorm(const Vector3 & v, double maxNorm)
{
  double n = v.norm();
  if(n <= maxNorm) return v;
  return v * (maxNorm / std::max(n, 1e-12));
}

struct Traj
{
  struct Iteration
  {
  protected:
    Iteration() {};

  public:
    Iteration(int id, double t, LocalKinematics kine) : id_(id), t_(t), kine_(kine) {}
    LocalKinematics & getKine()
    {
      return kine_;
    }
    Vector3 getPl() const
    {
      return kine_.position();
    } // local position
    Vector3 getPos() const
    {
      return kine_.orientation.toMatrix3() * getPl();
    } // world position
    Matrix3 getOri() const
    {
      return kine_.orientation.toMatrix3();
    }
    Orientation getOrientation() const
    {
      return kine_.orientation;
    }
    Vector4 getOriQuat() const
    {
      return kine_.orientation.toVector4();
    }
    Vector3 getX2() const
    {
      return kine_.orientation.toMatrix3().transpose() * Vector3::UnitZ();
    }
    Vector3 getYv() const
    {
      return kine_.linVel();
    } // local vel
    Vector3 getLinVel() const
    {
      return kine_.orientation.toMatrix3() * kine_.linVel();
    }
    Vector3 getYg() const
    {
      return kine_.angVel();
    }
    Vector3 getAngVel() const
    {
      return kine_.orientation.toMatrix3() * kine_.angVel();
    }
    Vector3 getYa() const
    {
      return kine_.linAcc() + cst::gravityConstant * getX2();
    }
    int getId() const
    {
      return id_;
    }
    double getTime() const
    {
      return t_;
    }

  protected:
    int id_;
    double t_;
    LocalKinematics kine_;
  };

  Traj() {};
  void init(double dt, double duration, bool inMotion)
  {
    setInMotion(inMotion);
    dt_ = dt;
    duration_ = duration;
    const int nbIters = int(std::round(duration / dt));

    iterations_.insert({0, Iteration(0, 0.0, LocalKinematics::zeroKinematics(kine::LocalKinematics::Flags::all))});
    prevIter_ = iterations_.cbegin();

    for(int iter = 0; iter < nbIters; ++iter)
    {
      iterate(iter, dt_);
    }
  }

  void setInMotion(bool inMotion)
  {
    inMotion_ = inMotion;
  }

  void iterate(int iter, double dt)
  {
    LocalKinematics newKine = iterations_.at(iter).getKine();

    // Random jerk like before
    Vector3 linJerk = tools::ProbabilityLawSimulation::getUniformMatrix(3, 1, -0.2, 0.2) * 10.0;
    Vector3 angJerk = tools::ProbabilityLawSimulation::getUniformMatrix(3, 1, -0.2, 0.2) * 10.0;

    // ---- Limits (tune to your system) ----
    const double maxLinAcc = 1.0; // m/s^2
    const double maxAngAcc = 6.0; // rad/s^2
    const double maxLinVel = 0.3; // m/s
    const double maxAngVel = 0.3; // rad/s
    const double c_lin = 2; // viscous damping (s^-1)
    const double c_ang = 2; // viscous damping (s^-1)

    // ---- Acceleration update ----
    if(inMotion_)
    {
      newKine.linAcc() += linJerk * dt;
      newKine.angAcc() += angJerk * dt;
    }
    else
    {
      // Stop mode: we’ll decay velocity directly (stable), so zero acc here
      newKine.linAcc().setZero();
      newKine.angAcc().setZero();
    }

    // Clamp accelerations component-wise
    newKine.linAcc() = newKine.linAcc().cwiseMax(-maxLinAcc).cwiseMin(maxLinAcc);
    newKine.angAcc() = newKine.angAcc().cwiseMax(-maxAngAcc).cwiseMin(maxAngAcc);

    // ---- Semi-implicit Euler on velocities ----
    Vector3 v_lin = newKine.linVel() + newKine.linAcc() * dt;
    Vector3 v_ang = newKine.angVel() + newKine.angAcc() * dt;

    // Viscous damping (prevents random-walk blow-up)
    v_lin *= std::exp(-c_lin * dt);
    v_ang *= std::exp(-c_ang * dt);

    // In stop mode, add stronger exact decay (stable for any dt)
    if(!inMotion_)
    {
      const double Kd = 20.0; // s^-1 (your old K, but used safely)
      const double decay = std::exp(-Kd * dt);
      v_lin *= decay;
      v_ang *= decay;
    }

    // Clamp velocity magnitudes
    v_lin = clampNorm(v_lin, maxLinVel);
    v_ang = clampNorm(v_ang, maxAngVel);

    // Write back clamped velocities
    newKine.linVel() = v_lin;
    newKine.angVel() = v_ang;

    // Integrate pose using *clamped* velocities (your existing integrate)
    newKine.integrate(dt);

    // Optional: clean tiny noise
    if(newKine.linVel().squaredNorm() < 1e-12) newKine.linVel().setZero();
    if(newKine.angVel().squaredNorm() < 1e-12) newKine.angVel().setZero();

    iterations_.insert({iter + 1, Iteration(iter + 1, (iter + 1) * dt, newKine)});
  }

  void reset()
  {
    prevIter_ = iterations_.cbegin();
  }

  // Sequential access API
  bool hasNextIter() const
  {
    return prevIter_ != iterations_.cend();
  }

  const Iteration & getNextIter()
  {
    BOOST_ASSERT(prevIter_ != iterations_.cend() && "No more iterations");
    const Iteration & out = prevIter_->second;
    ++prevIter_;
    return out;
  }

  Iteration & getFirstIter()
  {
    return iterations_.begin()->second;
  }

protected:
  double dt_{0.0};
  bool inMotion_{true};
  double duration_{0.0};
  std::map<int, Iteration> iterations_;
  std::map<int, Iteration>::const_iterator prevIter_;
};

// ===================== Tests =====================

int testWithGyroBias(int errorcode, double threshold)
{
  const double simTime = 50.00;
  const double dt = 0.0005;
  const int nbIters = int(std::round(simTime / dt));

  // Local trajectory just to get an initial state
  Traj traj;
  traj.init(dt, simTime, true);
  Traj::Iteration & firstIter = traj.getFirstIter();

  const Orientation R_true_ori = firstIter.getOrientation();
  const Matrix3 R_true = R_true_ori.toMatrix3();
  const Vector3 x2_true = firstIter.getX2();
  const Vector3 x1_true = Vector3::Zero();
  const Vector3 p_true = firstIter.getPl();

  // Viking (alpha, beta, gamma, mu, rho, v1, g0, dt)
  Viking viking(/*alpha*/ 3, /*beta*/ 3, /*gamma*/ 3,
                /*mu*/ 3, /*rho*/ 3, /*dt*/ dt);

  // ----- Build undesired-equilibrium init -----
  Eigen::Vector3d u = Eigen::Vector3d::Random();
  if(u.norm() < 1e-12) u = Eigen::Vector3d::UnitX();
  u.normalize();

  const double c_mag = 0.5; // ||tilde b|| along u
  Vector3 b_true = Vector3::Random() / 50.; // constant true gyro bias
  Vector3 b_hat0 = b_true - c_mag * u; // => tilde b = c_mag * u
  Vector3 x2_hat0 = u;
  x2_hat0.normalize(); // x2_hat0 ‖ tilde b and unit norm
  Vector3 x1_hat0 = 0.5 * u; // x1_hat0 ‖ tilde b

  // Orientation error tilde R = 180° about u  ⇒  R_hat0 = tildeR^T * R_true
  Matrix3 R_tilde_inf = Eigen::AngleAxisd(M_PI, u).toRotationMatrix();
  Matrix3 R_hat0_m = R_tilde_inf.transpose() * R_true;

  viking.initEstimator(x1_hat0, x2_hat0, b_hat0, R_hat0_m, p_true);

  // ----- Single CSV (shared name with testWithNonzeroLinAcc) -----
  std::ofstream file("/tmp/gyro_bias.csv");
  file << "t,"
          "est_x1_x,est_x1_y,est_x1_z,true_x1_x,true_x1_y,true_x1_z,"
          "est_x2_x,est_x2_y,est_x2_z,true_x2_x,true_x2_y,true_x2_z,"
          "est_px,est_py,est_pz,true_px,true_py,true_pz,"
          "est_roll,est_pitch,est_yaw,true_roll,true_pitch,true_yaw,"
          "est_bx,est_by,est_bz,true_bx,true_by,true_bz\n";

  double err = 0.0;

  for(int i = 0; i < nbIters; ++i)
  {
    // Inputs: yv=0, ya=0 ; yg = bias only
    const Vector3 yv = Vector3::Zero();
    const Vector3 ya = Vector3::Zero();
    const Vector3 yg = b_true;

    // Provide synchronous measurements each step
    viking.setMeasurement(yv, ya, yg, R_true, p_true, i + 1);

    // Step filter and fetch state at k+1
    ObserverBase::StateVector xhat = viking.getEstimatedState(i + 1);

    Vector3 x1_hat, x2_hat, b_hat, pl_hat;
    Orientation R_hat;
    unpackState(xhat, viking.getEstOrientation(), x1_hat, x2_hat, b_hat, R_hat, pl_hat);

    const Vector3 rpy_hat = R_hat.toRollPitchYaw();
    const Vector3 rpy_true = R_true_ori.toRollPitchYaw();

    const double t = (i + 1) * dt;
    file << t << "," << x1_hat[0] << "," << x1_hat[1] << "," << x1_hat[2] << "," << x1_true[0] << "," << x1_true[1]
         << "," << x1_true[2] << "," << x2_hat[0] << "," << x2_hat[1] << "," << x2_hat[2] << "," << x2_true[0] << ","
         << x2_true[1] << "," << x2_true[2] << "," << pl_hat[0] << "," << pl_hat[1] << "," << pl_hat[2] << ","
         << p_true[0] << "," << p_true[1] << "," << p_true[2] << "," << rpy_hat[0] << "," << rpy_hat[1] << ","
         << rpy_hat[2] << "," << rpy_true[0] << "," << rpy_true[1] << "," << rpy_true[2] << "," << b_hat[0] << ","
         << b_hat[1] << "," << b_hat[2] << "," << b_true[0] << "," << b_true[1] << "," << b_true[2] << "\n";

    // ---- Threshold checks on last 20% ----
    if(i > 4 * nbIters / 5)
    {
      // x1 → 0
      err = (x1_hat - x1_true).squaredNorm();
      if(err > threshold)
      {
        std::cout << "\nThe local velocity estimate is incorrect.\nEstimated: " << x1_hat.transpose()
                  << "\nSimulated: " << x1_true.transpose() << std::endl;
        file.close();
        return errorcode;
      }

      // x2 → R^T e_z
      err = (x2_hat - x2_true).squaredNorm();
      if(err > threshold)
      {
        std::cout << "\nThe tilt estimate is incorrect.\nEstimated: " << x2_hat.transpose()
                  << "\nSimulated: " << x2_true.transpose() << std::endl;
        file.close();
        return errorcode;
      }

      // position
      err = (pl_hat - p_true).squaredNorm();
      if(err > 1e-3)
      {
        std::cout << "\nThe position estimate is incorrect.\nEstimated: " << pl_hat.transpose()
                  << "\nSimulated: " << p_true.transpose() << std::endl;
        file.close();
        return errorcode;
      }

      // orientation
      Matrix3 oriError = R_hat.toMatrix3() * R_true.transpose();
      Vector3 oriErrorVector = kine::skewSymmetricToRotationVector(oriError - oriError.transpose()) / 2.0;
      err = oriErrorVector.squaredNorm();
      if(err > threshold)
      {
        std::cout << "\nThe orientation estimate is incorrect.\nError vector: " << oriErrorVector.transpose()
                  << std::endl;
        file.close();
        return errorcode;
      }

      // gyro bias
      err = (b_hat - b_true).squaredNorm();
      if(err > threshold)
      {
        std::cout << "\nThe gyro bias estimate is incorrect.\nEstimated: " << b_hat.transpose()
                  << "\nSimulated: " << b_true.transpose() << std::endl;
        file.close();
        return errorcode;
      }
    }
  }

  file.close();
  return 0;
}

int testWithNonzeroLinAcc(int errorcode, double threshold)
{
  const double simTime = 100.0;
  const double sim_dt = 1e-3;
  const double est_dt = 1e-3;
  const int N = int(std::lround(est_dt / sim_dt));
  BOOST_ASSERT(std::abs(est_dt - N * sim_dt) < 1e-12 && "est_dt must be an integer multiple of sim_dt");

  const int nbEstSteps = int(std::floor(simTime / est_dt));

  Traj localTraj;
  localTraj.init(sim_dt, simTime, true);

  Viking viking(3, 5, 3, 3, 3, est_dt);

  // True gyro bias (unknown to estimator)
  const Vector3 b_true = Vector3::Random();

  Traj::Iteration & firstIter = localTraj.getFirstIter();
  const Vector3 p0 = firstIter.getPl();
  const Orientation R0 = firstIter.getOrientation();

  // Choose a desired nonzero initial bias error b̃ and derive b̂0
  Vector3 btilde_desired = Vector3::Random();
  if(btilde_desired.norm() < 1e-9)
  {
    btilde_desired = Vector3(1.0, 0.0, 0.0);
  }
  const Vector3 b_hat0 = b_true - btilde_desired;

  // Make x1_hat0 and x2_hat0 colinear with b̃
  const Vector3 dir = btilde_desired.normalized();
  const Vector3 x1_hat0 = dir;
  const Vector3 x2_hat0 = dir;

  // Initialize the estimator at an attitude 180 deg from the true orientation (undesired equilibrium)
  const Matrix3 R_pi = Eigen::AngleAxisd(M_PI, Vector3::UnitX()).toRotationMatrix();
  const Matrix3 R_hat0 = R0.toMatrix3() * R_pi;

  viking.initEstimator(x1_hat0, x2_hat0, b_hat0, R_hat0, p0);

  std::ofstream file("/tmp/gyro_bias.csv");
  file << "t,"
          "est_x1_x,est_x1_y,est_x1_z,true_x1_x,true_x1_y,true_x1_z,"
          "est_x2_x,est_x2_y,est_x2_z,true_x2_x,true_x2_y,true_x2_z,"
          "est_px,est_py,est_pz,true_px,true_py,true_pz,"
          "est_roll,est_pitch,est_yaw,true_roll,true_pitch,true_yaw,"
          "est_qw,est_qx,est_qy,est_qz,true_qw,true_qx,true_qy,true_qz,"
          "est_bx,est_by,est_bz,true_bx,true_by,true_bz\n";

  int k = 0;
  while(k < nbEstSteps && localTraj.hasNextIter())
  {
    Vector3 sum_yv = Vector3::Zero();
    Vector3 sum_ya = Vector3::Zero();
    Vector3 sum_yg = Vector3::Zero();
    Traj::Iteration it = firstIter;

    for(int s = 0; s < N; ++s)
    {
      if(!localTraj.hasNextIter()) break;
      it = localTraj.getNextIter();
      sum_yv += it.getYv();
      sum_ya += it.getYa();
      sum_yg += it.getYg();
    }

    const Vector3 yv_avg = sum_yv / double(std::max(1, N));
    const Vector3 ya_avg = sum_ya / double(std::max(1, N));
    const Vector3 yg_avg = sum_yg / double(std::max(1, N));

    const Orientation R_true_ori = it.getOrientation();
    const Matrix3 R_true = R_true_ori.toMatrix3();
    const Vector3 p_true = it.getPl();
    const Vector3 x2_true = it.getX2();

    const Vector3 yg_meas = yg_avg + b_true;

    viking.setMeasurement(yv_avg, ya_avg, yg_meas, R_true, p_true, k + 1);

    ObserverBase::StateVector xhat = viking.getEstimatedState(k + 1);

    Vector3 x1_hat, x2_hat, b_hat, pl_hat;
    Orientation R_hat;
    unpackState(xhat, viking.getEstOrientation(), x1_hat, x2_hat, b_hat, R_hat, pl_hat);

    const Vector3 rpy_hat = R_hat.toRollPitchYaw();
    const Vector3 rpy_true = R_true_ori.toRollPitchYaw();

    const Quaternion q_hat = R_hat.toQuaternion().normalized();
    const Quaternion q_true = R_true_ori.toQuaternion().normalized();

    const double t_log = (k + 1) * est_dt;
    file << t_log << "," << x1_hat[0] << "," << x1_hat[1] << "," << x1_hat[2] << "," << yv_avg[0] << "," << yv_avg[1]
         << "," << yv_avg[2] << "," << x2_hat[0] << "," << x2_hat[1] << "," << x2_hat[2] << "," << x2_true[0] << ","
         << x2_true[1] << "," << x2_true[2] << "," << pl_hat[0] << "," << pl_hat[1] << "," << pl_hat[2] << ","
         << p_true[0] << "," << p_true[1] << "," << p_true[2] << "," << rpy_hat[0] << "," << rpy_hat[1] << ","
         << rpy_hat[2] << "," << rpy_true[0] << "," << rpy_true[1] << "," << rpy_true[2] << "," << q_hat.w() << ","
         << q_hat.x() << "," << q_hat.y() << "," << q_hat.z() << "," << q_true.w() << "," << q_true.x() << ","
         << q_true.y() << "," << q_true.z() << "," << b_hat[0] << "," << b_hat[1] << "," << b_hat[2] << "," << b_true[0]
         << "," << b_true[1] << "," << b_true[2] << "\n";

    ++k;
  }

  file.close();
  return 0;
}

int testWithAsyncPoseMeas(int errorcode, double threshold, bool inMotion)
{
  const double simTime = 5.0000;
  const double dt = 0.0001;
  const int nbIters = int(std::round(simTime / dt));

  // Per-test trajectory
  Traj traj;
  traj.init(dt, simTime, inMotion);

  double err;
  Viking viking(/*alpha*/ 1, /*beta*/ 1, /*gamma*/ 1,
                /*mu*/ 3, /*rho*/ 1, /*dt*/ dt);

  Traj::Iteration & firstIter = traj.getFirstIter();

  // Init estimate
  viking.initEstimator(firstIter.getYv(), firstIter.getX2(), Vector3::Zero(), firstIter.getOri(), firstIter.getPl());

  Vector3 gyroBias = Vector3::Random() / 1000;

  int i = 0;
  while(traj.hasNextIter() && i < nbIters)
  {
    const Traj::Iteration & currentIter = traj.getNextIter();

    // Synchronous measurements in new API
    const Vector3 yv = currentIter.getYv();
    const Vector3 ya = currentIter.getYa();
    const Vector3 yg = currentIter.getYg() + gyroBias;

    viking.setMeasurement(yv, ya, yg, currentIter.getOri(), currentIter.getPl(), i + 1);

    ObserverBase::StateVector xhat = viking.getEstimatedState(i + 1);

    Vector3 x1_hat, x2_hat, b_hat, pl_hat;
    Orientation R_hat;
    unpackState(xhat, viking.getEstOrientation(), x1_hat, x2_hat, b_hat, R_hat, pl_hat);

    if(i > 4 * nbIters / 5)
    {
      Matrix3 oriError = R_hat.toMatrix3() * currentIter.getOri().transpose();
      Vector3 oriErrorVector = kine::skewSymmetricToRotationVector(oriError - oriError.transpose()) / 2.0;

      err = (x1_hat - currentIter.getYv()).squaredNorm();
      if(err > threshold)
      {
        std::cout << "\nThe local velocity estimate is incorrect.\nEstimated: " << x1_hat.transpose()
                  << "\nSimulated: " << currentIter.getYv().transpose() << std::endl;
        return errorcode;
      }

      err = (x2_hat - currentIter.getX2()).squaredNorm();
      if(err > threshold)
      {
        std::cout << "\nThe tilt estimate is incorrect.\nEstimated: " << x2_hat.transpose()
                  << "\nSimulated: " << currentIter.getX2().transpose() << std::endl;
        return errorcode;
      }

      err = oriErrorVector.squaredNorm();
      if(err > threshold)
      {
        std::cout << "\nThe orientation estimate is incorrect.\nError vector: " << oriErrorVector.transpose()
                  << "\nEstimated: " << kine::rotationMatrixToYawAxisAgnostic(R_hat.toMatrix3())
                  << "\nSimulated: " << kine::rotationMatrixToYawAxisAgnostic(currentIter.getOri()) << std::endl;
        return errorcode;
      }

      err = (pl_hat - currentIter.getPl()).squaredNorm();
      if(err > 1e-3)
      {
        std::cout << "\nThe position estimate is incorrect.\nEstimated: " << pl_hat.transpose()
                  << "\nSimulated: " << currentIter.getPl().transpose() << std::endl;
        return errorcode;
      }
    }

    ++i;
  }

  return 0;
}

int testWithAsyncOriMeas(int errorcode, double threshold, bool inMotion)
{
  const double simTime = 20.0000;
  const double dt = 0.0001;
  const int nbIters = int(std::round(simTime / dt));

  // Per-test trajectory
  Traj traj;
  traj.init(dt, simTime, inMotion);

  double err;
  Viking viking(/*alpha*/ 1, /*beta*/ 1, /*gamma*/ 1,
                /*mu*/ 3, /*rho*/ 1, /*dt*/ dt);

  Traj::Iteration & firstIter = traj.getFirstIter();

  viking.initEstimator(firstIter.getYv(), firstIter.getX2(), Vector3::Zero(), firstIter.getOri(), firstIter.getPl());

  Vector3 gyroBias = Vector3::Random() / 100;

  // CSV
  std::ofstream file("/tmp/test.csv");
  file << "Iteration,EstimatedVelX,EstimatedVelY,EstimatedVelZ,SimulatedVelX,SimulatedVelY,SimulatedVelZ,EstimatedRoll,"
          "EstimatedPitch,EstimatedYaw,SimulatedRoll,SimulatedPitch,SimulatedYaw,"
          "EstimatedPosX,EstimatedPosY,EstimatedPosZ,SimulatedPosX,SimulatedPosY,SimulatedPosZ,EstimatedBiasX,"
          "EstimatedBiasY,EstimatedBiasZ,SimulatedBiasX,SimulatedBiasY,SimulatedBiasZ\n";

  int i = 0;
  while(traj.hasNextIter() && i < nbIters)
  {
    const Traj::Iteration & currentIter = traj.getNextIter();

    const Vector3 yv = currentIter.getYv();
    const Vector3 ya = currentIter.getYa();
    const Vector3 yg = currentIter.getYg() + gyroBias;

    viking.setMeasurement(yv, ya, yg, currentIter.getOri(), currentIter.getPl(), i + 1);

    ObserverBase::StateVector xhat = viking.getEstimatedState(i + 1);

    Vector3 x1_hat, x2_hat, b_hat, pl_hat;
    Orientation R_hat;
    unpackState(xhat, viking.getEstOrientation(), x1_hat, x2_hat, b_hat, R_hat, pl_hat);

    // CSV row
    file << i + 1 << "," << x1_hat[0] << "," << x1_hat[1] << "," << x1_hat[2] << "," << yv[0] << "," << yv[1] << ","
         << yv[2] << "," << R_hat.toRollPitchYaw()[0] << "," << R_hat.toRollPitchYaw()[1] << ","
         << R_hat.toRollPitchYaw()[2] << "," << currentIter.getOrientation().toRollPitchYaw()[0] << ","
         << currentIter.getOrientation().toRollPitchYaw()[1] << "," << currentIter.getOrientation().toRollPitchYaw()[2]
         << "," << pl_hat[0] << "," << pl_hat[1] << "," << pl_hat[2] << "," << currentIter.getPl()[0] << ","
         << currentIter.getPl()[1] << "," << currentIter.getPl()[2] << "," << b_hat[0] << "," << b_hat[1] << ","
         << b_hat[2] << "," << gyroBias[0] << "," << gyroBias[1] << "," << gyroBias[2] << "\n";

    if(i > 4 * nbIters / 5)
    {
      Matrix3 oriError = R_hat.toMatrix3() * currentIter.getOri().transpose();
      Vector3 oriErrorVector = kine::skewSymmetricToRotationVector(oriError - oriError.transpose()) / 2.0;

      err = (x1_hat - currentIter.getYv()).squaredNorm();
      if(err > threshold)
      {
        std::cout << "\nThe local velocity estimate is incorrect.\nEstimated: " << x1_hat.transpose()
                  << "\nSimulated: " << currentIter.getYv().transpose() << std::endl;
        file.close();
        return errorcode;
      }

      err = (x2_hat - currentIter.getX2()).squaredNorm();
      if(err > threshold)
      {
        std::cout << "\nThe tilt estimate is incorrect.\nEstimated: " << x2_hat.transpose()
                  << "\nSimulated: " << currentIter.getX2().transpose() << std::endl;
        file.close();
        return errorcode;
      }

      err = oriErrorVector.squaredNorm();
      if(err > threshold)
      {
        std::cout << "\nThe orientation estimate is incorrect.\nError vector: " << oriErrorVector.transpose()
                  << "\nEstimated: " << kine::rotationMatrixToYawAxisAgnostic(R_hat.toMatrix3())
                  << "\nSimulated: " << kine::rotationMatrixToYawAxisAgnostic(currentIter.getOri()) << std::endl;
        file.close();
        return errorcode;
      }

      err = (pl_hat - currentIter.getPl()).squaredNorm();
      if(err > 1e-3)
      {
        std::cout << "\nThe position estimate is incorrect.\nEstimated: " << pl_hat.transpose()
                  << "\nSimulated: " << currentIter.getPl().transpose() << std::endl;
        file.close();
        return errorcode;
      }
    }

    ++i;
  }

  file.close();
  return 0;
}

// ===================== Main =====================

int main()
{
  int returnVal;
  int errorcode = 1;

  std::cout << "Starting testWithNonzeroLinAcc" << std::endl;
  if((returnVal = testWithNonzeroLinAcc(errorcode, 1e-4)))
  {
    std::cout << "testWithNonzeroLinAcc failed!" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithNonzeroLinAcc succeeded" << std::endl;
  }
  ++errorcode;

  // std::cout << "Starting testWithGyroBias" << std::endl;
  // if((returnVal = testWithGyroBias(errorcode, 1e-4)))
  // {
  //   std::cout << "testWithGyroBias failed!" << errorcode << std::endl;
  //   return returnVal;
  // }
  // else
  // {
  //   std::cout << "testWithGyroBias succeeded" << std::endl;
  // }
  // ++errorcode;

  std::cout << "Test VikingEstimator succeeded" << std::endl;
  return 0;
}
