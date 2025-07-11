#include <iostream>
#include <state-observation/observer/waiko-humanoid.hpp>
#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

using namespace stateObservation;
using namespace kine;

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
    }
    Vector3 getPos() const
    {
      return kine_.orientation.toMatrix3() * getPl();
    }
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
    }
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
  void init(double dt, double duration)
  {
    dt_ = dt;
    duration_ = duration;
    int nbIters = int(std::round(duration / dt));

    iterations_.insert({0, Iteration(0, 0.0, LocalKinematics::zeroKinematics(kine::LocalKinematics::Flags::all))});
    prevIter_ = iterations_.begin();

    for(int iter = 0; iter < nbIters; iter++)
    {
      iterate(iter, dt_);
    }
  }

  void iterate(int iter, double dt, bool inMotion = true)
  {
    LocalKinematics newKine = iterations_.at(iter).getKine();
    Vector3 linJerk = tools::ProbabilityLawSimulation::getUniformMatrix(3, 1, -0.2, 0.2) * 10;
    Vector3 angJerk = tools::ProbabilityLawSimulation::getUniformMatrix(3, 1, -0.2, 0.2) * 10;

    if(inMotion)
    {
      newKine.linAcc() += linJerk * dt;
      newKine.angAcc() += angJerk * dt;
    }
    else
    {
      double K = 20;
      newKine.linAcc() = -K * newKine.linVel();
      newKine.angAcc() = -K * newKine.angVel();
    }
    newKine.integrate(dt);

    iterations_.insert({iter + 1, Iteration(iter + 1, (iter + 1) * dt, newKine)});
  }

  void reset()
  {
    prevIter_ = iterations_.begin();
  }

  const Iteration & getIter(double target_t)
  {
    double t1 = prevIter_->second.getTime();
    while(prevIter_->second.getTime() - target_t > std::numeric_limits<double>().epsilon())
    {
      prevIter_--;
    }
    while(prevIter_->second.getTime() - target_t < std::numeric_limits<double>().epsilon())
    {
      t1 = prevIter_->second.getTime();
      prevIter_++;
    }
    double dt1 = target_t - t1;
    double dt2 = prevIter_->second.getTime() - target_t;

    if(dt1 < dt2)
    {
      prevIter_--;
    }

    BOOST_ASSERT(abs(prevIter_->second.getTime() - target_t) <= abs(t1 - target_t) && "MARCHE PAS");

    return prevIter_->second;
  }

  Iteration & getFirstIter()
  {
    return iterations_.begin()->second;
  }

protected:
  double dt_;
  double duration_;
  std::map<int, Iteration> iterations_;
  std::map<int, Iteration>::const_iterator prevIter_;
};

Traj traj;

int testWithoutPosAndOriMeasurement(int errorcode, double threshold)
{
  double simTime = 3.00;
  double dt = 0.005;
  int nbIters = int(std::round(simTime / dt));

  double err;
  WaikoHumanoid waiko(dt, 1, 1, 1, 1, false);

  Traj::Iteration & firstIter = traj.getFirstIter();
  waiko.initEstimator(firstIter.getYv(), firstIter.getX2(), Vector3::Zero(), firstIter.getOriQuat(),
                      firstIter.getPos());

  for(int i = 0; i < nbIters; i++)
  {
    const Traj::Iteration & currentIter = traj.getIter(i * dt);
    waiko.setInput(currentIter.getYv(), currentIter.getYa(), currentIter.getYg(), i);
    waiko.getEstimatedState(i + 1);

    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizeX1> x1_hat = waiko.getEstimatedLocLinVel();
    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizeX2> x2_hat = waiko.getEstimatedTilt();
    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizeOri> q_hat = waiko.getEstimatedOrientation();
    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizePos> pl_hat = waiko.getEstimatedLocPosition();

    Orientation finalOri_hat;
    finalOri_hat.fromVector4(q_hat);

    Matrix3 oriError = finalOri_hat.toMatrix3() * currentIter.getOri().transpose();
    Vector3 oriErrorVector = kine::skewSymmetricToRotationVector(oriError - oriError.transpose()) / 2.0;

    if(i > 4 * nbIters / 5)
    {
      err = (x1_hat - currentIter.getYv()).squaredNorm();
      if(err > threshold)
      {
        std::cout << std::endl << "The local velocity estimate is incorrect." << std::endl;
        std::cout << std::endl << "Estimated: " << x1_hat.transpose() << std::endl;
        std::cout << std::endl << "Simulated: " << currentIter.getYv().transpose() << std::endl;
        return errorcode;
      }

      err = (x2_hat - currentIter.getX2()).squaredNorm();
      if(err > threshold)
      {
        std::cout << std::endl << "The tilt estimate is incorrect." << std::endl;
        std::cout << std::endl << "Estimated: " << x2_hat.transpose() << std::endl;
        std::cout << std::endl << "Simulated: " << currentIter.getX2().transpose() << std::endl;

        return errorcode;
      }

      err = (pl_hat - currentIter.getPl()).squaredNorm();
      if(err > 1e-3)
      {
        std::cout << std::endl << "The position estimate is incorrect." << std::endl;
        std::cout << std::endl << "Estimated: " << pl_hat.transpose() << std::endl;
        std::cout << std::endl << "Simulated: " << currentIter.getPl().transpose() << std::endl;
        return errorcode;
      }

      err = (oriErrorVector).squaredNorm();
      if(err > threshold)
      {
        std::cout << std::endl << "The orientation estimate is incorrect." << std::endl;
        std::cout << std::endl << "Error vector: " << oriErrorVector.transpose() << std::endl;
        return errorcode;
      }
    }
  }

  return 0;
}

int testWithPosAndOriMeasurement(int errorcode, double threshold)
{
  double simTime = 5.00;
  double dt = 0.005;
  int nbIters = int(std::round(simTime / dt));

  double err;
  WaikoHumanoid waiko(dt, 1, 1, 1, 1, false);
  Traj::Iteration & firstIter = traj.getFirstIter();

  waiko.initEstimator(firstIter.getYv(), firstIter.getX2(), Vector3::Zero(), firstIter.getOriQuat(),
                      firstIter.getPos());

  for(int i = 0; i < nbIters; i++)
  {
    const Traj::Iteration & currentIter = traj.getIter(i * dt);

    Vector3 yv = currentIter.getYv() + Vector3::Random() / 1000;
    Vector3 ya = currentIter.getYa() + Vector3::Random() / 1000;
    Vector3 yg = currentIter.getYg() + Vector3::Random() / 1000;

    waiko.setInput(yv, ya, yg, i);
    waiko.addContactInput(InputWaiko::ContactInput(currentIter.getOri(), currentIter.getPos(), 1, 1, 1, 1), i);
    waiko.getEstimatedState(i + 1);

    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizeX1> x1_hat = waiko.getEstimatedLocLinVel();
    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizeX2> x2_hat = waiko.getEstimatedTilt();
    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizeOri> q_hat = waiko.getEstimatedOrientation();
    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizePos> pl_hat = waiko.getEstimatedLocPosition();

    Orientation finalOri_hat;
    finalOri_hat.fromVector4(q_hat);

    Matrix3 oriError = finalOri_hat.toMatrix3() * currentIter.getOri().transpose();
    Vector3 oriErrorVector = kine::skewSymmetricToRotationVector(oriError - oriError.transpose()) / 2.0;

    if(i > 4 * nbIters / 5)
    {
      err = (x1_hat - currentIter.getYv()).squaredNorm();
      if(err > threshold)
      {
        std::cout << std::endl << "The local velocity estimate is incorrect." << std::endl;
        std::cout << std::endl << "Estimated: " << x1_hat.transpose() << std::endl;
        std::cout << std::endl << "Simulated: " << currentIter.getYv().transpose() << std::endl;
        return errorcode;
      }

      err = (x2_hat - currentIter.getX2()).squaredNorm();
      if(err > threshold)
      {
        std::cout << std::endl << "The tilt estimate is incorrect." << std::endl;
        std::cout << std::endl << "Estimated: " << x2_hat.transpose() << std::endl;
        std::cout << std::endl << "Simulated: " << currentIter.getX2().transpose() << std::endl;

        return errorcode;
      }

      err = (pl_hat - currentIter.getPl()).squaredNorm();
      if(err > 1e-3)
      {
        std::cout << std::endl << "The position estimate is incorrect." << std::endl;
        std::cout << std::endl << "Estimated: " << pl_hat.transpose() << std::endl;
        std::cout << std::endl << "Simulated: " << currentIter.getPl().transpose() << std::endl;
        return errorcode;
      }

      err = (oriErrorVector).squaredNorm();
      if(err > threshold)
      {
        std::cout << std::endl << "The orientation estimate is incorrect." << std::endl;
        std::cout << std::endl << "Error vector: " << oriErrorVector.transpose() << std::endl;
        std::cout << std::endl
                  << "Estimated: " << kine::rotationMatrixToYawAxisAgnostic(finalOri_hat.toMatrix3()) << std::endl;
        std::cout << std::endl
                  << "Simulated: " << kine::rotationMatrixToYawAxisAgnostic(currentIter.getOri()) << std::endl;
        return errorcode;
      }
    }
  }

  return 0;
}

int testWithGyroBias(int errorcode, double threshold)
{
  double simTime = 5.00;
  double dt = 0.0001;
  int nbIters = int(std::round(simTime / dt));

  double err;
  WaikoHumanoid waiko(dt, 1, 1, 1, 1, true);

  Traj::Iteration & firstIter = traj.getFirstIter();

  waiko.initEstimator(firstIter.getYv(), firstIter.getX2(), Vector3::Zero(), firstIter.getOriQuat(),
                      firstIter.getPos());

  Vector3 gyroBias = Vector3::Random() / 100;

  for(int i = 0; i < nbIters; i++)
  {
    const Traj::Iteration & currentIter = traj.getIter(i * dt);

    Vector3 yv = currentIter.getYv();
    Vector3 ya = currentIter.getYa();
    Vector3 yg = currentIter.getYg() + gyroBias;

    waiko.setInput(yv, ya, yg, i);

    waiko.addContactInput(InputWaiko::ContactInput(currentIter.getOri(), currentIter.getPos(), 1, 1, 1, 1), i);
    waiko.getEstimatedState(i + 1);

    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizeX1> x1_hat = waiko.getEstimatedLocLinVel();
    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizeX2> x2_hat = waiko.getEstimatedTilt();
    // Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizeGyroBias> b_hat = waiko.getEstimatedGyroBias();
    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizeOri> q_hat = waiko.getEstimatedOrientation();
    Eigen::VectorBlock<ObserverBase::StateVector, WaikoHumanoid::sizePos> pl_hat = waiko.getEstimatedLocPosition();

    Orientation finalOri_hat;
    finalOri_hat.fromVector4(q_hat);

    if(i > 4 * nbIters / 5)
    {
      Matrix3 oriError = finalOri_hat.toMatrix3() * currentIter.getOri().transpose();
      Vector3 oriErrorVector = kine::skewSymmetricToRotationVector(oriError - oriError.transpose()) / 2.0;

      err = (x1_hat - currentIter.getYv()).squaredNorm();
      if(err > threshold)
      {
        std::cout << std::endl << "The local velocity estimate is incorrect." << std::endl;
        std::cout << std::endl << "Estimated: " << x1_hat.transpose() << std::endl;
        std::cout << std::endl << "Simulated: " << currentIter.getYv().transpose() << std::endl;
        return errorcode;
      }

      err = (x2_hat - currentIter.getX2()).squaredNorm();
      if(err > threshold)
      {
        std::cout << std::endl << "The tilt estimate is incorrect." << std::endl;
        std::cout << std::endl << "Estimated: " << x2_hat.transpose() << std::endl;
        std::cout << std::endl << "Simulated: " << currentIter.getX2().transpose() << std::endl;

        return errorcode;
      }

      err = (oriErrorVector).squaredNorm();
      if(err > threshold)
      {
        std::cout << std::endl << "The orientation estimate is incorrect." << std::endl;
        std::cout << std::endl << "Error vector: " << oriErrorVector.transpose() << std::endl;
        std::cout << std::endl
                  << "Estimated: " << kine::rotationMatrixToYawAxisAgnostic(finalOri_hat.toMatrix3()) << std::endl;
        std::cout << std::endl
                  << "Simulated: " << kine::rotationMatrixToYawAxisAgnostic(currentIter.getOri()) << std::endl;
        return errorcode;
      }

      err = (pl_hat - currentIter.getPl()).squaredNorm();
      if(err > 1e-3)
      {
        std::cout << std::endl << "The position estimate is incorrect." << std::endl;
        std::cout << std::endl << "Estimated: " << pl_hat.transpose() << std::endl;
        std::cout << std::endl << "Simulated: " << currentIter.getPl().transpose() << std::endl;
        return errorcode;
      }
    }
  }

  return 0;
}

int main()
{
  int returnVal;
  int errorcode = 1;

  traj.init(0.0001, 20);

  std::cout << "Starting testWithoutPosAndOriMeasurement" << std::endl;
  if((returnVal = testWithoutPosAndOriMeasurement(errorcode, 1e-6)))
  {
    std::cout << "testWithoutPosAndOriMeasurement failed!" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithoutPosAndOriMeasurement succeeded" << std::endl;
  }
  errorcode++;
  traj.reset();

  std::cout << "Starting testWithPosAndOriMeasurement" << std::endl;
  if((returnVal = testWithPosAndOriMeasurement(errorcode, 1e-6)))
  {
    std::cout << "testWithPosAndOriMeasurement failed!" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithPosAndOriMeasurement succeeded" << std::endl;
  }
  errorcode++;
  traj.reset();

  std::cout << "Starting testWithGyroBias" << std::endl;
  if((returnVal = testWithGyroBias(errorcode, 1e-4)))
  {
    std::cout << "testWithGyroBias failed!" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithGyroBias succeeded" << std::endl;
  }
  errorcode++;
  traj.reset();

  std::cout << "Test WaikoHumanoid succeeded" << std::endl;
  return 0;
}
