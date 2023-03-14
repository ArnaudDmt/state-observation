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

KineticsObserver ko_1_(1, 1);
KineticsObserver ko_2_(2, 2);

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
      * (kine::skewSymmetric(ko_.getInertiaMatrix()() * worldCentroidKinematics.angVel()) - ko_.getInertiaMatrixDot()()
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

  bool stopIT = false;

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
        stopIT = true;
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

  bool stopIT = false;

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
        stopIT = true;
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

  /* Kinetics Observer 1 initialization */

  worldContactOri1_.setRandom();
  Kinematics worldContactPose1_;
  worldContactPose1_.position = worldContactPos1_;
  worldContactPose1_.orientation = worldContactOri1_;
  centroidContactOri1_.setRandom();
  Kinematics centroidContactPose1_;
  centroidContactPose1_.position = centroidContactPos1_;
  centroidContactPose1_.orientation = centroidContactOri1_;
  centroidContactPose1_.linVel = centroidContactLinVel1_;
  centroidContactPose1_.angVel = centroidContactAngVel1_;

  centroidIMUOri1_.setRandom();
  Kinematics centroidIMUPose1_;
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

  ko_1_.setCoMAngularMomentum(angularMomentum_, angularMomentum_d_);
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

  std::cout << std::endl << "Tests with 1 contact and 1 gyrometer: " << std::endl << std::endl;

  /*
  for(int i = 0; i < stateVector_.size(); i++)
  {
    dx_.segment<ko_.sizePosTangent>(ko_.posIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizePos>(ko_.posIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizeOriTangent>(ko_.oriIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizeOri>(ko_.oriIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizeLinVelTangent>(ko_.linVelIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizeLinVel>(ko_.linVelIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizeAngVelTangent>(ko_.angVelIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizeAngVel>(ko_.angVelIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizeGyroBiasTangent>(ko_.gyroBiasIndexTangent(0))
        .setConstant(stateVector_.segment<ko_.sizeGyroBias>(ko_.gyroBiasIndex(0)).norm() * 1e-8);
    dx_.segment<ko_.sizeForceTangent>(ko_.unmodeledForceIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizeForce>(ko_.unmodeledForceIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizePosTangent>(ko_.unmodeledTorqueIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizeTorque>(ko_.unmodeledTorqueIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizePosTangent>(ko_.contactPosIndexTangent(0))
        .setConstant(stateVector_.segment<ko_.sizePos>(ko_.contactPosIndex(0)).norm() * 1e-8);
    dx_.segment<ko_.sizeOriTangent>(ko_.contactOriIndexTangent(0))
        .setConstant(stateVector_.segment<ko_.sizeOri>(ko_.contactOriIndex(0)).norm() * 1e-8);
    dx_.segment<ko_.sizeForceTangent>(ko_.contactForceIndexTangent(0))
        .setConstant(stateVector_.segment<ko_.sizeForce>(ko_.contactForceIndex(0)).norm() * 1e-8);
    dx_.segment<ko_.sizeTorqueTangent>(ko_.contactTorqueIndexTangent(0))
        .setConstant(stateVector_.segment<ko_.sizeTorque>(ko_.contactTorqueIndex(0)).norm() * 1e-8);
    if(secondContactAndGyro_)
    {
      dx_.segment<ko_.sizeGyroBiasTangent>(ko_.gyroBiasIndexTangent(1))
          .setConstant(stateVector_.segment<ko_.sizeGyroBias>(ko_.gyroBiasIndex(1)).norm() * 1e-8);
      dx_.segment<ko_.sizePosTangent>(ko_.contactPosIndexTangent(0))
          .setConstant(stateVector_.segment<ko_.sizePos>(ko_.contactPosIndex(1)).norm() * 1e-8);
      dx_.segment<ko_.sizeOriTangent>(ko_.contactOriIndexTangent(0))
          .setConstant(stateVector_.segment<ko_.sizeOri>(ko_.contactOriIndex(1)).norm() * 1e-8);
      dx_.segment<ko_.sizeForceTangent>(ko_.contactForceIndexTangent(0))
          .setConstant(stateVector_.segment<ko_.sizeForce>(ko_.contactForceIndex(1)).norm() * 1e-8);
      dx_.segment<ko_.sizeTorqueTangent>(ko_.contactTorqueIndexTangent(0))
          .setConstant(stateVector_.segment<ko_.sizeTorque>(ko_.contactTorqueIndex(1)).norm() * 1e-8);
    }
  }
  */
  std::cout << "Starting testAccelerationsJacobians." << std::endl;
  if((returnVal = testAccelerationsJacobians(ko_1_, ++errorcode, 0.1, 5e-11)))
  {
    std::cout << "testAccelerationsJacobians Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testAccelerationsJacobians succeeded" << std::endl;
  }

  std::cout << "Starting testOrientationsJacobians." << std::endl;
  if((returnVal = testOrientationsJacobians(ko_1_, ++errorcode, 0.1, 1.66e-16)))
  {
    std::cout << "testOrientationsJacobians Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testOrientationsJacobians succeeded" << std::endl;
  }

  std::cout << "Starting testAnalyticalAJacobianVsFD." << std::endl;
  if((returnVal = testAnalyticalAJacobianVsFD(ko_1_, ++errorcode, 2.5, 0.005)))
  {
    std::cout << "testAnalyticalAJacobianVsFD Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testAnalyticalAJacobianVsFD succeeded" << std::endl;
  }

  std::cout << "Starting testAnalyticalCJacobianVsFD." << std::endl;
  if((returnVal = testAnalyticalCJacobianVsFD(ko_1_, ++errorcode, 0.05, 8e-11)))
  {
    std::cout << "testAnalyticalCJacobianVsFD Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testAnalyticalCJacobianVsFD succeeded" << std::endl;
  }

  /* Kinetics Observer 2 initialization */

  worldContactOri2_.setRandom();
  Kinematics worldContactPose2_;
  worldContactPose2_.position = worldContactPos2_;
  worldContactPose2_.orientation = worldContactOri2_;
  centroidContactOri2_.setRandom();
  Kinematics centroidContactPose2_;
  centroidContactPose2_.position = centroidContactPos2_;
  centroidContactPose2_.orientation = centroidContactOri2_;
  centroidContactPose2_.linVel = centroidContactLinVel2_;
  centroidContactPose2_.angVel = centroidContactAngVel2_;

  centroidIMUOri2_.setRandom();
  Kinematics centroidIMUPose2_;
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

  ko_2_.setCoMAngularMomentum(angularMomentum_, angularMomentum_d_);

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

  /*
  for(int i = 0; i < stateVector_.size(); i++)
  {
    dx_.segment<ko_.sizePosTangent>(ko_.posIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizePos>(ko_.posIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizeOriTangent>(ko_.oriIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizeOri>(ko_.oriIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizeLinVelTangent>(ko_.linVelIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizeLinVel>(ko_.linVelIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizeAngVelTangent>(ko_.angVelIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizeAngVel>(ko_.angVelIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizeGyroBiasTangent>(ko_.gyroBiasIndexTangent(0))
        .setConstant(stateVector_.segment<ko_.sizeGyroBias>(ko_.gyroBiasIndex(0)).norm() * 1e-8);
    dx_.segment<ko_.sizeForceTangent>(ko_.unmodeledForceIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizeForce>(ko_.unmodeledForceIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizePosTangent>(ko_.unmodeledTorqueIndexTangent())
        .setConstant(stateVector_.segment<ko_.sizeTorque>(ko_.unmodeledTorqueIndex()).norm() * 1e-8);
    dx_.segment<ko_.sizePosTangent>(ko_.contactPosIndexTangent(0))
        .setConstant(stateVector_.segment<ko_.sizePos>(ko_.contactPosIndex(0)).norm() * 1e-8);
    dx_.segment<ko_.sizeOriTangent>(ko_.contactOriIndexTangent(0))
        .setConstant(stateVector_.segment<ko_.sizeOri>(ko_.contactOriIndex(0)).norm() * 1e-8);
    dx_.segment<ko_.sizeForceTangent>(ko_.contactForceIndexTangent(0))
        .setConstant(stateVector_.segment<ko_.sizeForce>(ko_.contactForceIndex(0)).norm() * 1e-8);
    dx_.segment<ko_.sizeTorqueTangent>(ko_.contactTorqueIndexTangent(0))
        .setConstant(stateVector_.segment<ko_.sizeTorque>(ko_.contactTorqueIndex(0)).norm() * 1e-8);
    if(secondContactAndGyro_)
    {
      dx_.segment<ko_.sizeGyroBiasTangent>(ko_.gyroBiasIndexTangent(1))
          .setConstant(stateVector_.segment<ko_.sizeGyroBias>(ko_.gyroBiasIndex(1)).norm() * 1e-8);
      dx_.segment<ko_.sizePosTangent>(ko_.contactPosIndexTangent(0))
          .setConstant(stateVector_.segment<ko_.sizePos>(ko_.contactPosIndex(1)).norm() * 1e-8);
      dx_.segment<ko_.sizeOriTangent>(ko_.contactOriIndexTangent(0))
          .setConstant(stateVector_.segment<ko_.sizeOri>(ko_.contactOriIndex(1)).norm() * 1e-8);
      dx_.segment<ko_.sizeForceTangent>(ko_.contactForceIndexTangent(0))
          .setConstant(stateVector_.segment<ko_.sizeForce>(ko_.contactForceIndex(1)).norm() * 1e-8);
      dx_.segment<ko_.sizeTorqueTangent>(ko_.contactTorqueIndexTangent(0))
          .setConstant(stateVector_.segment<ko_.sizeTorque>(ko_.contactTorqueIndex(1)).norm() * 1e-8);
    }
  }
  */

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
  return 0;
}