#include <iostream>
#include <state-observation/observer/viking.hpp>
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

    BOOST_ASSERT(abs(prevIter_->second.getTime() - target_t) < abs(t1 - target_t) && "MARCHE PAS");

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
  Viking viking(dt, 1, 1, 1, 1, 1, false);

  Traj::Iteration & firstIter = traj.getFirstIter();
  viking.initEstimator(firstIter.getYv(), firstIter.getX2(), Vector3::Zero(), firstIter.getOriQuat(),
                       firstIter.getPos());

  for(int i = 0; i < nbIters; i++)
  {
    const Traj::Iteration & currentIter = traj.getIter(i * dt);
    viking.setInput(currentIter.getYv(), currentIter.getYa(), currentIter.getYg(), i);
    viking.getEstimatedState(i + 1);

    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX1> x1_hat = viking.getEstimatedLocLinVel();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX2> x2_hat = viking.getEstimatedTilt();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeOri> q_hat = viking.getEstimatedOrientation();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizePos> pl_hat = viking.getEstimatedLocPosition();

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
  Viking viking(dt, 1, 1, 1, 1, 1, false);
  Traj::Iteration & firstIter = traj.getFirstIter();

  viking.initEstimator(firstIter.getYv(), firstIter.getX2(), Vector3::Zero(), firstIter.getOriQuat(),
                       firstIter.getPos());

  for(int i = 0; i < nbIters; i++)
  {
    const Traj::Iteration & currentIter = traj.getIter(i * dt);

    Vector3 yv = currentIter.getYv() + Vector3::Random() / 1000;
    Vector3 ya = currentIter.getYa() + Vector3::Random() / 1000;
    Vector3 yg = currentIter.getYg() + Vector3::Random() / 1000;

    viking.setInput(yv, ya, yg, i);
    viking.addPosOriMeasurement(currentIter.getPos(), currentIter.getOri(), 1, 1, 1, 1);
    viking.getEstimatedState(i + 1);

    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX1> x1_hat = viking.getEstimatedLocLinVel();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX2> x2_hat = viking.getEstimatedTilt();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeOri> q_hat = viking.getEstimatedOrientation();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizePos> pl_hat = viking.getEstimatedLocPosition();

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
  Viking viking(dt, 1, 1, 1, 1, 1, true);

  Traj::Iteration & firstIter = traj.getFirstIter();

  viking.initEstimator(firstIter.getYv(), firstIter.getX2(), Vector3::Zero(), firstIter.getOriQuat(),
                       firstIter.getPos());

  Vector3 gyroBias = Vector3::Random() / 100;

  for(int i = 0; i < nbIters; i++)
  {
    const Traj::Iteration & currentIter = traj.getIter(i * dt);

    Vector3 yv = currentIter.getYv();
    Vector3 ya = currentIter.getYa();
    Vector3 yg = currentIter.getYg() + gyroBias;

    viking.setInput(yv, ya, yg, i);
    viking.addPosOriMeasurement(currentIter.getPos(), currentIter.getOri(), 1, 1, 1, 1);
    viking.getEstimatedState(i + 1);

    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX1> x1_hat = viking.getEstimatedLocLinVel();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX2> x2_hat = viking.getEstimatedTilt();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeGyroBias> b_hat = viking.getEstimatedGyroBias();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeOri> q_hat = viking.getEstimatedOrientation();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizePos> pl_hat = viking.getEstimatedLocPosition();

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

int testWithAsyncPoseMeas(int errorcode, double threshold)
{
  double simTime = 5.0000;
  double dt = 0.0001;
  int nbIters = int(std::round(simTime / dt));

  double err;
  Viking viking(dt, 1, 1, 1, 3, 1, true);

  Traj::Iteration & firstIter = traj.getFirstIter();

  viking.initEstimator(firstIter.getYv(), firstIter.getX2(), Vector3::Zero(), firstIter.getOriQuat(),
                       firstIter.getPos());

  Vector3 gyroBias = Vector3::Random() / 1000;

  for(int i = 0; i < nbIters; i++)
  {
    const Traj::Iteration & currentIter = traj.getIter(i * dt);

    Vector3 yv = currentIter.getYv();
    Vector3 ya = currentIter.getYa();
    Vector3 yg = currentIter.getYg() + gyroBias;

    viking.setInput(yv, ya, yg, i);

    double delay = 0.0004;
    if(i % 2 == 0 && (i * dt - delay) >= 0.0)
    {
      const Traj::Iteration & delayedIter = traj.getIter(i * dt - delay);

      viking.addDelayedPosOriMeasurement(delayedIter.getPos(), delayedIter.getOri(), 1, 1, 1, 1, delay);
    }

    viking.getEstimatedState(i + 1);

    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX1> x1_hat = viking.getEstimatedLocLinVel();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX2> x2_hat = viking.getEstimatedTilt();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeOri> q_hat = viking.getEstimatedOrientation();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizePos> pl_hat = viking.getEstimatedLocPosition();

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

int testWithAsyncOriMeas(int errorcode, double threshold)
{
  double simTime = 20.0000;
  double dt = 0.0001;
  int nbIters = int(std::round(simTime / dt));

  double err;
  Viking viking(dt, 1, 1, 1, 3, 1, true);

  Traj::Iteration & firstIter = traj.getFirstIter();

  viking.initEstimator(firstIter.getYv(), firstIter.getX2(), Vector3::Zero(), firstIter.getOriQuat(),
                       firstIter.getPos());

  Vector3 gyroBias = Vector3::Random() / 1000;

  // Open the CSV file for writing
  std::ofstream file("/tmp/test.csv");

  // Write the header row
  file << "Iteration,EstimatedVelX,EstimatedVelY,EstimatedVelZ,SimulatedVelX,SimulatedVelY,SimulatedVelZ,EstimatedRoll,"
          "EstimatedPitch,EstimatedYaw,SimulatedRoll,SimulatedPitch,SimulatedYaw,"
          "EstimatedPosX,EstimatedPosY,EstimatedPosZ,SimulatedPosX,SimulatedPosY,SimulatedPosZ,EstimatedBiasX,"
          "EstimatedBiasY,EstimatedBiasZ,SimulatedBiasX,SimulatedBiasY,SimulatedBiasZ\n";

  for(int i = 0; i < nbIters; i++)
  {
    const Traj::Iteration & currentIter = traj.getIter(i * dt);

    Vector3 yv = currentIter.getYv();
    Vector3 ya = currentIter.getYa();
    Vector3 yg = currentIter.getYg() + gyroBias;

    viking.setInput(yv, ya, yg, i);

    double delay = 0.0004;
    if(i % 2 == 0 && (i * dt - delay) >= 0.0)
    {
      const Traj::Iteration & delayedIter = traj.getIter(i * dt - delay);

      viking.addDelayedOriMeasurement(delayedIter.getOri(), 1, 1, delay);
    }

    viking.getEstimatedState(i + 1);

    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX1> x1_hat = viking.getEstimatedLocLinVel();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeX2> x2_hat = viking.getEstimatedTilt();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeGyroBias> b_hat = viking.getEstimatedGyroBias();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizeOri> q_hat = viking.getEstimatedOrientation();
    Eigen::VectorBlock<ObserverBase::StateVector, Viking::sizePos> pl_hat = viking.getEstimatedLocPosition();

    Orientation finalOri_hat;
    finalOri_hat.fromVector4(q_hat);

    // Write the data for this iteration to the CSV
    file << i + 1 << "," << x1_hat[0] << "," << x1_hat[1] << "," << x1_hat[2] << "," << yv[0] << "," << yv[1] << ","
         << yv[2] << "," << finalOri_hat.toRollPitchYaw()[0] << "," << finalOri_hat.toRollPitchYaw()[1] << ","
         << finalOri_hat.toRollPitchYaw()[2] << "," << currentIter.getOrientation().toRollPitchYaw()[0] << ","
         << currentIter.getOrientation().toRollPitchYaw()[1] << "," << currentIter.getOrientation().toRollPitchYaw()[2]
         << "," << pl_hat[0] << "," << pl_hat[1] << "," << pl_hat[2] << "," << currentIter.getPl()[0] << ","
         << currentIter.getPl()[1] << "," << currentIter.getPl()[2] << "," << b_hat[0] << "," << b_hat[1] << ","
         << b_hat[2] << "," << gyroBias[0] << "," << gyroBias[1] << "," << gyroBias[2] << "\n";

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
  file.close();
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

  std::cout << "Starting testWithAsyncPoseMeas" << std::endl;
  if((returnVal = testWithAsyncPoseMeas(errorcode, 1e-6)))
  {
    std::cout << "testWithAsyncPoseMeas failed!" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithAsyncPoseMeas succeeded" << std::endl;
  }
  errorcode++;
  traj.reset();

  std::cout << "Starting testWithAsyncOriMeas" << std::endl;
  if((returnVal = testWithAsyncOriMeas(errorcode, 1e-6)))
  {
    std::cout << "testWithAsyncOriMeas failed!" << errorcode << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithAsyncOriMeas succeeded" << std::endl;
  }

  std::cout << "Test Viking succeeded" << std::endl;
  return 0;
}
