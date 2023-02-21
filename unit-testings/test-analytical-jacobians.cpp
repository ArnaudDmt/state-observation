#include <bitset>
#include <iostream>
#include <state-observation/tools/definitions.hpp>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

using namespace stateObservation::kine;

namespace stateObservation
{

double dt_ = 0.005;

double lin_stiffness_ = (double)rand() / RAND_MAX * 1e5;
double lin_damping_ = (double)rand() / RAND_MAX * 5 * 1e1;
double ang_stiffness_ = (double)rand() / RAND_MAX * 1e5;
double ang_damping_ = (double)rand() / RAND_MAX * 5 * 1e1;

Matrix3 K1_ = lin_stiffness_ * Matrix3::Identity();
Matrix3 K2_ = lin_damping_ * Matrix3::Identity();
Matrix3 K3_ = ang_stiffness_ * Matrix3::Identity();
Matrix3 K4_ = ang_damping_ * Matrix3::Identity();

double lin_stiffness_2_ = (double)rand() / RAND_MAX * 1e5;
double lin_damping_2_ = (double)rand() / RAND_MAX * 5 * 1e1;
double ang_stiffness_2_ = (double)rand() / RAND_MAX * 1e5;
double ang_damping_2_ = (double)rand() / RAND_MAX * 5 * 1e1;

Matrix3 K1_2_ = lin_stiffness_2_ * Matrix3::Identity();
Matrix3 K2_2_ = lin_damping_2_ * Matrix3::Identity();
Matrix3 K3_2_ = ang_stiffness_2_ * Matrix3::Identity();
Matrix3 K4_2_ = ang_damping_2_ * Matrix3::Identity();

Vector3 com_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 com_d_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 com_dd_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 position_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation ori_;
Vector3 linvel_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 angvel_ = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10 * 100;

Vector3 gyroBias1_ = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;
Vector3 gyroBias2_ = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

Vector3 extForces_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 extTorques_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 worldContactPos1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation worldContactOri1_;
Vector3 centroidContactPos1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidContactOri1_;
Vector3 centroidContactLinVel1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidContactAngVel1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 contactForces1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() * 1000;
Vector3 contactTorques1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() * 10;

Vector3 worldContactPos2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation worldContactOri2_;
Vector3 centroidContactPos2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidContactOri2_;
Vector3 centroidContactLinVel2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidContactAngVel2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 contactForces2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() * 100;
Vector3 contactTorques2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() * 10;

Vector3 centroidIMUPos1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidIMUOri1_;
Vector3 centroidIMULinVel1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidIMUAngVel1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidIMULinAcc1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidIMUAngAcc1_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 centroidIMUPos2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidIMUOri2_;
Vector3 centroidIMULinVel2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidIMUAngVel2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidIMULinAcc2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidIMUAngAcc2_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Matrix3 inertiaMatrix_ = tools::ProbabilityLawSimulation::getUniformMatrix<Matrix3>();
Matrix3 inertiaMatrix_d_ = tools::ProbabilityLawSimulation::getGaussianMatrix<Matrix3>();
Vector3 angularMomentum_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 angularMomentum_d_ = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Eigen::IOFormat CleanFmt_(4, 0, ", ", "\n", "[", "]");

Vector dx_;
double error_ = 0;

Kinematics worldContactPose1_;
Kinematics centroidContactPose1_;
Kinematics centroidIMUPose1_;

Kinematics worldContactPose2_;
Kinematics centroidIMUPose2_;
Kinematics centroidContactPose2_;

KineticsObserver ko_1_(1, 1);
KineticsObserver ko_2_(2, 2);
KineticsObserver ko_3_(2, 2);

///////////////////////////////////////////////////////////////////////
/// -------------------Intermediary functions for the tests-------------
///////////////////////////////////////////////////////////////////////

Matrix displayVectorWithIndex(Matrix A) // to be remove
{
  Matrix indexedA(A.rows() + 1, A.cols() + 1);
  indexedA.setZero();
  indexedA.block(1, 1, A.rows(), A.cols()) = A;

  for(int i = 0; i < A.rows(); i++)
  {
    for(int j = 0; j < A.cols(); j++)
    {
      indexedA(0, j + 1) = j;
      indexedA(i + 1, 0) = i;
    }
  }
  return indexedA;
}

///////////////////////////////////////////////////////////////////////
/// -------------------Tests implementation-------------
///////////////////////////////////////////////////////////////////////

int testAccelerationsJacobians(KineticsObserver & ko_,
                               int errcode,
                               double relativeErrorThreshold,
                               double threshold) // 1
{
  /* Finite differences Jacobian */
  Matrix accJacobianFD = Matrix::Zero(6, ko_.getStateTangentSize());

  Vector accBar = Vector6::Zero();
  Vector accBarIncremented = Vector6::Zero();
  Vector accBarDiff = Vector6::Zero();

  Vector x = ko_.getEKF().getCurrentEstimatedState();
  Vector xIncrement = ko_.getEKF().getCurrentEstimatedState();

  ko_.computeLocalAccelerations(x, accBar);

  xIncrement.resize(ko_.getStateTangentSize());

  for(Index i = 0; i < ko_.getStateTangentSize(); ++i)
  {
    xIncrement.setZero();
    xIncrement[i] = dx_[i];

    ko_.stateSum(x, xIncrement, x);

    ko_.computeLocalAccelerations(x, accBarIncremented);

    accBarDiff = accBarIncremented - accBar;

    accBarDiff /= dx_[i];

    accJacobianFD.col(i) = accBarDiff;

    x = ko_.getEKF().getCurrentEstimatedState();
  }

  /* Analytical jacobian */

  LocalKinematics worldCentroidKinematics(x, ko_.flagsStateKine);
  Matrix accJacobianAnalytical = Matrix::Zero(6, ko_.getStateTangentSize());
  Matrix3 I_inv = ko_.getInertiaMatrix()().inverse();

  // Jacobians of the linear acceleration
  accJacobianAnalytical.block<3, ko_.sizeOriTangent>(0, ko_.oriIndexTangent()) =
      -cst::gravityConstant
      * (worldCentroidKinematics.orientation.toMatrix3().transpose() * kine::skewSymmetric(Vector3(0, 0, 1)));
  accJacobianAnalytical.block<3, ko_.sizeForceTangent>(0, ko_.unmodeledForceIndexTangent()) =
      Matrix::Identity(ko_.sizeLinAccTangent, ko_.sizeTorqueTangent) / ko_.getMass();

  // Jacobians of the angular acceleration
  accJacobianAnalytical.block<3, ko_.sizeTorqueTangent>(3, ko_.unmodeledTorqueIndexTangent()) = I_inv;
  accJacobianAnalytical.block<3, ko_.sizeAngVelTangent>(3, ko_.angVelIndexTangent()) =
      I_inv
      * (kine::skewSymmetric(ko_.getInertiaMatrix()() * worldCentroidKinematics.angVel()) - ko_.getInertiaMatrix_d()()
         - kine::skewSymmetric(worldCentroidKinematics.angVel()) * ko_.getInertiaMatrix()()
         + kine::skewSymmetric(ko_.getAngularMomentum()()));

  // Jacobians with respect to the contacts
  for(KineticsObserver::VectorContactConstIterator i = ko_.contacts_.begin(); i != ko_.contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      // Jacobian of the linar acceleration with respect to the contact force
      accJacobianAnalytical.block<3, ko_.sizeForceTangent>(0, ko_.contactForceIndexTangent(i)) =
          (1.0 / ko_.getMass()) * i->centroidContactKine.orientation.toMatrix3();
      // Jacobian of the angular acceleration with respect to the contact force
      accJacobianAnalytical.block<3, ko_.sizeTorqueTangent>(3, ko_.contactForceIndexTangent(i)) =
          (I_inv * kine::skewSymmetric(i->centroidContactKine.position()))
          * (i->centroidContactKine.orientation).toMatrix3();
      // Jacobian of the angular acceleration with respect to the contact torque
      accJacobianAnalytical.block<3, ko_.sizeTorqueTangent>(3, ko_.contactTorqueIndexTangent(i)) =
          I_inv * i->centroidContactKine.orientation.toMatrix3();
    }
  }

  /* Comparison */

  for(int i = 0; i < accJacobianAnalytical.rows(); i++)
  {
    for(int j = 0; j < accJacobianAnalytical.cols(); j++)
    {
      if(abs(accJacobianAnalytical(i, j) - accJacobianFD(i, j))
                 / std::max(abs(accJacobianAnalytical(i, j)), abs(accJacobianFD(i, j))) * 100
             > relativeErrorThreshold
         && abs(accJacobianAnalytical(i, j) - accJacobianFD(i, j)) != 0
         && (abs(accJacobianAnalytical(i, j)) > 1.0e-9 && abs(accJacobianFD(i, j)) > 1.0e-9))
      {
        std::cout << std::endl
                  << "\033[1;31m"
                  << "error indexes: " << std::endl
                  << "(" << i << "," << j << "):  Analytic : " << accJacobianAnalytical(i, j)
                  << "    FD : " << accJacobianFD(i, j) << "    Relative error : "
                  << abs(accJacobianAnalytical(i, j) - accJacobianFD(i, j))
                         / std::max(abs(accJacobianAnalytical(i, j)), abs(accJacobianFD(i, j))) * 100
                  << " % "
                  << "\033[0m\n"
                  << std::endl;
        return errcode;
      }
    }
  }

  error_ = (accJacobianAnalytical - accJacobianFD).squaredNorm();

  std::cout << "Error between the analytical and the finite differences acceleration Jacobians: " << error_
            << std::endl;

  if(error_ > threshold)
  {
    return errcode;
  }
}

int testOrientationsJacobians(KineticsObserver & ko_, int errcode, double relativeErrorThreshold, double threshold) // 2
{
  /* Finite differences Jacobian */
  Matrix rotationJacobianDeltaFD = Matrix::Zero(3, 3);

  Vector currentState = ko_.getEKF().getCurrentEstimatedState();
  Vector accelerations = Vector6::Zero();
  ko_.computeLocalAccelerations(currentState, accelerations);
  LocalKinematics kineTestOri(currentState);

  kineTestOri.linAcc = accelerations.segment<3>(0);
  kineTestOri.angAcc = accelerations.segment<3>(3);

  Orientation oriBar = Orientation::zeroRotation();
  Orientation oriBarIncremented = Orientation::zeroRotation();
  Vector3 oriBarDiff = Vector3::Zero();

  oriBar = kineTestOri.orientation;
  oriBarIncremented = kineTestOri.orientation;

  // Vector3 dt_x_omega(10000, 24265, 589);
  Vector3 dt_x_omega = dt_ * kineTestOri.angVel() + dt_ * dt_ / 2 * kineTestOri.angAcc();

  oriBar.integrateRightSide(dt_x_omega);

  Vector3 xIncrement = Vector3::Zero();

  for(Index i = 0; i < 3; ++i)
  {
    xIncrement.setZero();
    xIncrement[i] = dx_[i];

    Vector3 incremented_dt_x_omega = dt_x_omega + xIncrement;

    oriBarIncremented.integrateRightSide(incremented_dt_x_omega);

    oriBarDiff = oriBar.differentiate(oriBarIncremented);

    oriBarDiff /= dx_[i];

    rotationJacobianDeltaFD.col(i) = oriBarDiff;
    oriBarIncremented = kineTestOri.orientation;
  }

  Matrix rotationJacobianDeltaAnalytical =
      2.0 / dt_x_omega.norm()
      * (((dt_x_omega.norm() - 2.0 * sin(dt_x_omega.norm() / 2.0)) / (2.0 * dt_x_omega.squaredNorm()))
             * kineTestOri.orientation.toMatrix3() * dt_x_omega * dt_x_omega.transpose()
         + sin(dt_x_omega.norm() / 2.0) * kineTestOri.orientation.toMatrix3()
               * kine::rotationVectorToRotationMatrix(dt_x_omega / 2.0));

  for(int i = 0; i < rotationJacobianDeltaAnalytical.rows(); i++)
  {
    for(int j = 0; j < rotationJacobianDeltaAnalytical.cols(); j++)
    {
      if(abs(rotationJacobianDeltaAnalytical(i, j) - rotationJacobianDeltaFD(i, j))
                 / std::max(abs(rotationJacobianDeltaAnalytical(i, j)), abs(rotationJacobianDeltaFD(i, j))) * 100
             > relativeErrorThreshold
         && abs(rotationJacobianDeltaAnalytical(i, j) - rotationJacobianDeltaFD(i, j)) != 0)
      {
        std::cout << std::endl
                  << "\033[1;31m"
                  << "error indexes: " << std::endl
                  << "(" << i << "," << j << "):  Analytic : " << rotationJacobianDeltaAnalytical(i, j)
                  << "    FD : " << rotationJacobianDeltaFD(i, j) << "    Relative error : "
                  << abs(rotationJacobianDeltaAnalytical(i, j) - rotationJacobianDeltaFD(i, j))
                         / std::max(abs(rotationJacobianDeltaAnalytical(i, j)), abs(rotationJacobianDeltaFD(i, j)))
                         * 100
                  << " % "
                  << "\033[0m\n"
                  << std::endl;
        return errcode;
      }
    }
  }

  error_ = (rotationJacobianDeltaAnalytical - rotationJacobianDeltaFD).squaredNorm();

  std::cout << "Error between the analytical and the finite differences Jacobians of the orientation integration wrt "
               "an increment delta: "
            << error_ << std::endl;

  if(error_ > threshold)
  {
    return errcode;
  }
}

int testAnalyticalAJacobianVsFD(KineticsObserver & ko_,
                                int errcode,
                                double relativeErrorThreshold,
                                double threshold) // 3
{
  Matrix A_analytic = ko_.computeAMatrix();

  Matrix A_FD = ko_.getEKF().getAMatrixFD(dx_);

  for(int i = 0; i < A_analytic.rows(); i++)
  {
    for(int j = 0; j < A_analytic.cols(); j++)
    {
      if(abs(A_analytic(i, j) - A_FD(i, j)) / std::max(abs(A_analytic(i, j)), abs(A_FD(i, j))) * 100
             > relativeErrorThreshold
         && abs(A_analytic(i, j) - A_FD(i, j)) != 0 && (abs(A_analytic(i, j)) > 1.0e-9 && abs(A_FD(i, j)) > 1.0e-9))
      {
        std::cout << std::endl
                  << "\033[1;31m"
                  << "error indexes: " << std::endl
                  << "(" << i << "," << j << "):  Analytic : " << A_analytic(i, j) << "    FD : " << A_FD(i, j)
                  << "    Relative error : "
                  << abs(A_analytic(i, j) - A_FD(i, j)) / std::max(abs(A_analytic(i, j)), abs(A_FD(i, j))) * 100
                  << " % "
                  << "\033[0m\n"
                  << std::endl;
        return errcode;
      }
      else
      {
        /*
        std::cout << std::endl
                  << "good indexes: " << std::endl
                  << "(" << i << "," << j << "):  Analytic : " << A_analytic(i, j) << "    FD : " << A_FD(i, j)
                  << "    Relative error : " << abs(A_analytic(i, j) - A_FD(i, j)) / std::max(abs(A_analytic(i,
        j)), abs(A_FD(i, j))) * 100
                  << " % " << std::endl;

                  */
      }
    }
  }

  error_ = (A_analytic - A_FD).squaredNorm();

  std::cout << "Error between the analytical and the finite differences A Jacobian: " << error_ << std::endl;

  if(error_ > threshold)
  {
    return errcode;
  }
}

int testAnalyticalCJacobianVsFD(KineticsObserver & ko_,
                                int errcode,
                                double relativeErrorThreshold,
                                double threshold) // 3
{
  Matrix C_analytic = ko_.computeCMatrix();

  Matrix C_FD = ko_.getEKF().getCMatrixFD(dx_);

  for(int i = 0; i < C_analytic.rows(); i++)
  {
    for(int j = 0; j < C_analytic.cols(); j++)
    {
      if(abs(C_analytic(i, j) - C_FD(i, j)) / std::max(abs(C_analytic(i, j)), abs(C_FD(i, j))) * 100
             > relativeErrorThreshold
         && abs(C_analytic(i, j) - C_FD(i, j)) != 0 && (abs(C_analytic(i, j)) > 1.0e-9 && abs(C_FD(i, j)) > 1.0e-9))
      {
        std::cout << std::endl
                  << "\033[1;31m"
                  << "error indexes: " << std::endl
                  << "(" << i << "," << j << "):  Analytic : " << C_analytic(i, j) << "    FD : " << C_FD(i, j)
                  << "    Relative error : "
                  << abs(C_analytic(i, j) - C_FD(i, j)) / std::max(abs(C_analytic(i, j)), abs(C_FD(i, j))) * 100
                  << " % "
                  << "\033[0m\n"
                  << std::endl;
        return errcode;
      }
      else
      {
        /*
        std::cout << std::endl
                  << "good indexes: " << std::endl
                  << "(" << i << "," << j << "):  C_analytic : " << C_analytic(i, j) << "    FD : " << C_FD(i, j)
                  << "    Relative error : " << abs(C_analytic(i, j) - C_FD(i, j)) / std::max(abs(C_analytic(i,
        j)), abs(C_FD(i, j))) * 100
                  << " % " << std::endl;

                  */
      }
    }
  }

  error_ = (C_analytic - C_FD).squaredNorm();

  std::cout << "Error between the analytical and the finite differences C Jacobian: " << error_ << std::endl;

  if(error_ > threshold)
  {
    return errcode;
  }
}

} // end namespace stateObservation

using namespace stateObservation;

int main()
{
  int returnVal;
  int errorcode = 0;

  Vector stateVector_;

  inertiaMatrix_ = inertiaMatrix_ * inertiaMatrix_.transpose();
  inertiaMatrix_d_ = inertiaMatrix_d_ * inertiaMatrix_d_.transpose();

  ori_.setRandom();

  /*
  /// Kinetics Observer 1 initialization ///

  worldContactOri1_.setRandom();

  worldContactPose1_.position = worldContactPos1_;
  worldContactPose1_.orientation = worldContactOri1_;
  centroidContactOri1_.setRandom();

  centroidContactPose1_.position = centroidContactPos1_;
  centroidContactPose1_.orientation = centroidContactOri1_;
  centroidContactPose1_.linVel = centroidContactLinVel1_;
  centroidContactPose1_.angVel = centroidContactAngVel1_;

  centroidIMUOri1_.setRandom();

  centroidIMUPose1_.position = centroidIMUPos1_;
  centroidIMUPose1_.orientation = centroidIMUOri1_;
  centroidIMUPose1_.linVel = centroidIMULinVel1_;
  centroidIMUPose1_.angVel = centroidIMUAngVel1_;
  centroidIMUPose1_.linAcc = centroidIMULinAcc1_;
  centroidIMUPose1_.angAcc = centroidIMUAngAcc1_;

  ko_1_.setCenterOfMass(com_, com_d_, com_dd_);

  ko_1_.setSamplingTime(dt_);
  ko_1_.setWithUnmodeledWrench(true);
  ko_1_.useRungeKutta(false);
  ko_1_.setWithGyroBias(true);
  ko_1_.setWithUnmodeledWrench(true);

  ko_1_.setAngularMomentum(angularMomentum_, angularMomentum_d_);
  ko_1_.setInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_1_.addContact(worldContactPose1_, 0, K1_, K2_, K3_, K4_);
  ko_1_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose1_, 0);

  ko_1_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose1_, 0);

  stateVector_.resize(position_.size() + 4 + linvel_.size() + angvel_.size() + gyroBias1_.size() + extForces_.size()
                      + extTorques_.size() + worldContactPos1_.size() + worldContactOri1_.toVector4().size()
                      + contactForces1_.size() + contactTorques1_.size());
  stateVector_ << position_, ori_.toVector4(), linvel_, angvel_, gyroBias1_, extForces_, extTorques_, worldContactPos1_,
      worldContactOri1_.toVector4(), contactForces1_, contactTorques1_;

  ko_1_.setInitWorldCentroidStateVector(stateVector_);

  dx_.resize(ko_1_.getStateTangentSize());
  dx_.setZero();
  dx_.setConstant(1e-6);

  dx_.segment<ko_1_.sizeForceTangent>(ko_1_.contactForceIndexTangent(0)).setConstant(1e-5);

  ko_1_.updateMeasurements();

  ko_1_.getEKF().updateStatePrediction();

  /// Kinetics Observer 2 initialization ///

  worldContactOri2_.setRandom();

  worldContactPose2_.position = worldContactPos2_;
  worldContactPose2_.orientation = worldContactOri2_;
  centroidContactOri2_.setRandom();

  centroidContactPose2_.position = centroidContactPos2_;
  centroidContactPose2_.orientation = centroidContactOri2_;
  centroidContactPose2_.linVel = centroidContactLinVel2_;
  centroidContactPose2_.angVel = centroidContactAngVel2_;

  centroidIMUOri2_.setRandom();

  centroidIMUPose2_.position = centroidIMUPos2_;
  centroidIMUPose2_.orientation = centroidIMUOri2_;
  centroidIMUPose2_.linVel = centroidIMULinVel2_;
  centroidIMUPose2_.angVel = centroidIMUAngVel2_;
  centroidIMUPose2_.linAcc = centroidIMULinAcc2_;
  centroidIMUPose2_.angAcc = centroidIMUAngAcc2_;

  ko_2_.setCenterOfMass(com_, com_d_, com_dd_);

  ko_2_.setSamplingTime(dt_);
  ko_2_.setWithUnmodeledWrench(true);
  ko_2_.useRungeKutta(false);
  ko_2_.setWithGyroBias(true);
  ko_2_.setWithUnmodeledWrench(true);

  ko_2_.setAngularMomentum(angularMomentum_, angularMomentum_d_);

  ko_2_.setInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_2_.addContact(worldContactPose1_, 0, K1_, K2_, K3_, K4_);
  ko_2_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose1_, 0);
  ko_2_.addContact(worldContactPose2_, 1, K1_2_, K2_2_, K3_2_, K4_2_);
  ko_2_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose2_, 1);

  ko_2_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose1_, 0);
  ko_2_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose2_, 1);

  stateVector_.resize(position_.size() + 4 + linvel_.size() + angvel_.size() + gyroBias1_.size() + gyroBias2_.size()
                      + extForces_.size() + extTorques_.size() + worldContactPos1_.size()
                      + worldContactOri1_.toVector4().size() + contactForces1_.size() + contactTorques1_.size()
                      + worldContactPos2_.size() + worldContactOri2_.toVector4().size() + contactForces2_.size()
                      + contactTorques2_.size());
  stateVector_ << position_, ori_.toVector4(), linvel_, angvel_, gyroBias1_, gyroBias2_, extForces_, extTorques_,
      worldContactPos1_, worldContactOri1_.toVector4(), contactForces1_, contactTorques1_, worldContactPos2_,
      worldContactOri2_.toVector4(), contactForces2_, contactTorques2_;

  ko_2_.setInitWorldCentroidStateVector(stateVector_);

  dx_.resize(ko_2_.getStateTangentSize());
  dx_.setZero();
  dx_.setConstant(1e-6);

  dx_.segment<ko_2_.sizeForceTangent>(ko_2_.contactForceIndexTangent(0)).setConstant(1e-5);
  dx_.segment<ko_2_.sizeForceTangent>(ko_2_.contactForceIndexTangent(1)).setConstant(1e-5);

  ko_2_.updateMeasurements();

  ko_2_.getEKF().updateStatePrediction();

  std::cout << std::endl << "Tests with 2 contacts and 2 gyrometers: " << std::endl << std::endl;

  std::cout << "Starting testAccelerationsJacobians." << std::endl;
  if((returnVal = testAccelerationsJacobians(ko_2_, ++errorcode, 0.1, 4.95e-11)))
  {
    std::cout << "testAccelerationsJacobians Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testAccelerationsJacobians succeeded" << std::endl;
  }

  std::cout << "Starting testOrientationsJacobians." << std::endl;
  if((returnVal = testOrientationsJacobians(ko_2_, ++errorcode, 0.1, 1.64e-16)))
  {
    std::cout << "testOrientationsJacobians Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testOrientationsJacobians succeeded" << std::endl;
  }

  std::cout << "Starting testAnalyticalAJacobianVsFD." << std::endl;
  if((returnVal = testAnalyticalAJacobianVsFD(ko_2_, ++errorcode, 2.5, 6)))
  {
    std::cout << "testAnalyticalAJacobianVsFD Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testAnalyticalAJacobianVsFD succeeded" << std::endl;
  }

  std::cout << "Starting testAnalyticalCJacobianVsFD." << std::endl;
  if((returnVal = testAnalyticalCJacobianVsFD(ko_2_, ++errorcode, 0.77, 1.18e-10)))
  {
    std::cout << "testAnalyticalCJacobianVsFD Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testAnalyticalCJacobianVsFD succeeded" << std::endl;
  }

  std::cout << "test succeeded" << std::endl;
  */
  /* Kinetics Observer 3 initialization with variables coherent obtained from real simulation */

  std::cout << std::endl
            << "Tests with 2 contacts and 2 gyrometers using simulation values: " << std::endl
            << std::endl;

  Vector3 ori;
  Vector3 tempVec;
  tempVec << 0.0245188, 0.080997, 0.00077336;
  worldContactPose1_.position = tempVec;
  ori << -0.000157517, 0.0100821, -0.00223641;
  worldContactPose1_.orientation.fromVector4(kine::rotationVectorToQuaternion(ori).coeffs());
  tempVec << -0.00259903, 0.0792194, -0.779776;
  centroidContactPose1_.position = tempVec;
  ori << 0.000367755, 0.00842225, -0.0025534;
  centroidContactPose1_.orientation.fromVector4(kine::rotationVectorToQuaternion(ori).coeffs());
  tempVec << 8.45645e-07, 2.50606e-08, 1.31682e-06;
  centroidContactPose1_.linVel = tempVec;
  tempVec << 0, 0, 0;
  centroidContactPose1_.angVel = tempVec;

  LocalKinematics localCentroidIMU1;

  tempVec << -0.0325, 0, 0.856687;
  localCentroidIMU1.position = tempVec;
  ori << 0, 0, 0;
  localCentroidIMU1.orientation.fromVector4(kine::rotationVectorToQuaternion(ori).coeffs());
  tempVec << 0, 0, 0;
  localCentroidIMU1.linVel = tempVec;
  tempVec << 0, 0, 0;
  localCentroidIMU1.angVel = tempVec;
  tempVec << 0, 0, 0;
  localCentroidIMU1.linAcc = tempVec;
  tempVec << 0, 0, 0;
  localCentroidIMU1.angAcc = tempVec;

  centroidIMUPose1_ = localCentroidIMU1;

  tempVec << 0.0258562, -0.0784297, 0.000782872;
  worldContactPose2_.position = tempVec;
  ori << -0.000996467, 0.00915504, 0.00232681;
  worldContactPose2_.orientation.fromVector4(kine::rotationVectorToQuaternion(ori).coeffs());

  tempVec << -0.00120752, -0.0802216, -0.779767;
  centroidContactPose2_.position = tempVec;
  ori << -0.000495544, 0.00749715, 0.00199812;
  centroidContactPose2_.orientation.fromVector4(kine::rotationVectorToQuaternion(ori).coeffs());
  tempVec << 8.45645e-07, 2.50606e-08, 1.31682e-06;
  centroidContactPose2_.linVel = tempVec;
  tempVec << 0, 0, 0;
  centroidContactPose2_.angVel = tempVec;

  LocalKinematics localCentroidIMU2;

  tempVec << 0, 0, 0.747187;
  localCentroidIMU2.position = tempVec;
  ori << 0, 0, 0;
  localCentroidIMU2.orientation.fromVector4(kine::rotationVectorToQuaternion(ori).coeffs());
  tempVec << 0, 0, 0;
  localCentroidIMU2.linVel = tempVec;
  tempVec << 0, 0, 0;
  localCentroidIMU2.angVel = tempVec;
  tempVec << 0, 0, 0;
  localCentroidIMU2.linAcc = tempVec;
  tempVec << 0, 0, 0;
  localCentroidIMU2.angAcc = tempVec;

  centroidIMUPose2_ = localCentroidIMU2;

  com_ << 0.0208054, 0.000453782, 0.780549;
  com_d_ << -8.45645e-07, -2.50606e-08, -1.31682e-06;
  com_dd_ << 1.73329e-06, 6.16717e-08, 3.34942e-06;

  angularMomentum_ << 1.24961e-06, 1.08498e-05, 5.28098e-07;
  angularMomentum_d_ << -2.86686e-06, -2.52373e-05, -1.20837e-06;

  inertiaMatrix_.row(0) << 28.91, -0.000423273, -0.492491;
  inertiaMatrix_.row(1) << -0.000423273, 28.6053, -0.00993046;
  inertiaMatrix_.row(2) << -0.492491, -0.00993046, 0.46347;

  inertiaMatrix_d_.row(0) << -7.87275e-05, 3.46417e-08, 2.62965e-05;
  inertiaMatrix_d_.row(1) << 3.46417e-08, -8.00725e-05, 7.71861e-07;
  inertiaMatrix_d_.row(2) << 2.62965e-05, 7.71861e-07, -1.3468e-06;

  ko_3_.setCenterOfMass(com_, com_d_, com_dd_);
  ko_3_.setMass(38.0549);

  ko_3_.setSamplingTime(dt_);
  ko_3_.setWithUnmodeledWrench(true);
  ko_3_.useRungeKutta(false);
  ko_3_.setWithGyroBias(true);
  ko_3_.setWithUnmodeledWrench(true);

  ko_3_.setAngularMomentum(angularMomentum_, angularMomentum_d_);

  ko_3_.setInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  Vector3 K;
  K << 4e4, 4e4, 4e4;
  Matrix K1 = K.asDiagonal();
  K << 65.76, 65.76, 65.76;
  Matrix K2 = K.asDiagonal();
  K << 727, 727, 727;
  Matrix K3 = K.asDiagonal();
  K << 17, 17, 17;
  Matrix K4 = K.asDiagonal();

  ko_3_.addContact(worldContactPose1_, 0, K1, K2, K3, K4);
  ko_3_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose1_, 0);
  ko_3_.addContact(worldContactPose2_, 1, K1, K2, K3, K4);
  ko_3_.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose2_, 1);

  ko_3_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose1_, 0);
  ko_3_.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose2_, 1);

  stateVector_.resize(ko_3_.sizePos + ko_3_.sizeOri + ko_3_.sizeLinVel + ko_3_.sizeAngVel + ko_3_.sizeGyroBias * 2
                      + ko_3_.sizeForce + ko_3_.sizeTorque // ext wrench
                      + (ko_3_.sizePos + ko_3_.sizeOri + ko_3_.sizeForce + ko_3_.sizeTorque) * 2); // contacts)

  stateVector_ << 0.0322053, 0.00287065, 0.769771, 0.000188336, -0.00256324, 1.63073e-05, 0.999997, 0.0388738,
      0.0134236, -0.0470932, 0.00607427, 0.0210703, -0.00290669, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0245188,
      0.080997, 0.00077336, -7.87579e-05, 0.00504102, -0.0011182, 0.999987, 1.92111, -42.3394, 178.001, -7.64832,
      5.0657, 2.67891, 0.0258562, -0.0784297, 0.000782872, -0.000498232, 0.00457751, 0.0011634, 0.999989, 0.186025,
      -42.1272, 192.886, -7.60783, 5.09597, 2.69835;

  ko_3_.setInitWorldCentroidStateVector(stateVector_);
  std::cout << std::endl << "x : " << std::endl << ko_3_.getEKF().getCurrentEstimatedState() << std::endl;
  dx_.resize(ko_3_.getStateTangentSize());
  dx_.setZero();
  dx_.setConstant(1e-6);

  ko_3_.updateMeasurements();

  ko_3_.getEKF().updateStatePrediction();

  std::cout << std::endl << "Tests with 2 contacts and 2 gyrometers: " << std::endl << std::endl;

  std::cout << std::endl << "x : " << std::endl << ko_3_.getEKF().getCurrentEstimatedState() << std::endl;
  std::cout << std::endl << "xbar : " << std::endl << ko_3_.getEKF().getLastPrediction() << std::endl;

  /*
  std::cout << "Starting testAccelerationsJacobians." << std::endl;
  if((returnVal = testAccelerationsJacobians(ko_3_, ++errorcode, 0.1, 4.95e-11)))
  {
    std::cout << "testAccelerationsJacobians Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testAccelerationsJacobians succeeded" << std::endl;
  }

  std::cout << "Starting testOrientationsJacobians." << std::endl;
  if((returnVal = testOrientationsJacobians(ko_3_, ++errorcode, 0.1, 1.64e-16)))
  {
    std::cout << "testOrientationsJacobians Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testOrientationsJacobians succeeded" << std::endl;
  }
  */
  std::cout << "Starting testAnalyticalAJacobianVsFD." << std::endl;
  if((returnVal = testAnalyticalAJacobianVsFD(ko_3_, ++errorcode, 0.2, 6)))
  {
    std::cout << "testAnalyticalAJacobianVsFD Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testAnalyticalAJacobianVsFD succeeded" << std::endl;
  }

  std::cout << "Starting testAnalyticalCJacobianVsFD." << std::endl;
  if((returnVal = testAnalyticalCJacobianVsFD(ko_3_, ++errorcode, 0.77, 1.18e-10)))
  {
    std::cout << "testAnalyticalCJacobianVsFD Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testAnalyticalCJacobianVsFD succeeded" << std::endl;
  }

  std::cout << "test succeeded" << std::endl;

  return 0;
}