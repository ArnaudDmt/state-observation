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

Matrix3 K1 = lin_stiffness_ * Matrix3::Identity();
Matrix3 K2 = lin_damping_ * Matrix3::Identity();
Matrix3 K3 = ang_stiffness_ * Matrix3::Identity();
Matrix3 K4 = ang_damping_ * Matrix3::Identity();

Vector3 com = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 com_d = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 com_dd = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 position = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation ori(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10 * 100;

Vector3 gyroBias1 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;
Vector3 gyroBias2 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() / 10;

Vector3 extForces = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 extTorques = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Vector3 worldContactPos1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation worldContactOri1(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 centroidContactPos1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidContactOri1(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 centroidContactLinVel1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidContactAngVel1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 contactForces1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() * 10000;
Vector3 contactTorques1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() * 10;

Vector3 worldContactPos2 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation worldContactOri2(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 centroidContactPos2 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
kine::Orientation centroidContactOri2(Vector3(tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10));
Vector3 centroidContactLinVel2 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 centroidContactAngVel2 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 contactForces2 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 contactTorques2 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Matrix3 inertiaMatrix_ = tools::ProbabilityLawSimulation::getUniformMatrix<Matrix3>();
Matrix3 inertiaMatrix_d_ = tools::ProbabilityLawSimulation::getGaussianMatrix<Matrix3>();
Vector3 angularMomentum = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;
Vector3 angularMomentum_d = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>() / 10;

Eigen::IOFormat CleanFmt_(4, 0, ", ", "\n", "[", "]");

Vector dx_;
double error_ = 0;

bool secondContactAndGyro_ = false;
KineticsObserver ko_(1, 1);

///////////////////////////////////////////////////////////////////////
/// -------------------Intermediary functions for the tests-------------
///////////////////////////////////////////////////////////////////////

Matrix displayVectorWithIndex(Matrix A) // to be remove
{
  Matrix indexedA(A.rows() + 1, A.cols() + 1);
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

int testAccelerationsJacobians(int errcode, double relativeErrorThreshold, double threshold) // 1
{
  /* Finite differences Jacobian */
  Matrix accJacobianFD = Matrix::Zero(6, ko_.getStateTangentSize());

  Vector accBar = Vector6::Zero();
  Vector accBarIncremented = Vector6::Zero();
  Vector accBarDiff = Vector6::Zero();

  Vector x = ko_.getEKF().getCurrentEstimatedState();
  Vector xIncrement = ko_.getEKF().getCurrentEstimatedState();

  ko_.computeLocalAccelerations(x, accBar);

  std::cout << "accBar: " << std::endl << accBar << std::endl;

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

  std::cout << std::endl
            << "Analytical : " << std::endl
            << displayVectorWithIndex(accJacobianAnalytical).format(CleanFmt_) << std::endl;
  std::cout << std::endl
            << "FD : " << std::endl
            << displayVectorWithIndex(accJacobianFD).format(CleanFmt_) << std::endl;

  /* Comparison */

  for(int i = 0; i < accJacobianAnalytical.rows(); i++)
  {
    for(int j = 0; j < accJacobianAnalytical.cols(); j++)
    {
      if(abs(accJacobianAnalytical(i, j) - accJacobianFD(i, j))
                 / std::max(abs(accJacobianAnalytical(i, j)), abs(accJacobianFD(i, j))) * 100
             > relativeErrorThreshold
         && abs(accJacobianAnalytical(i, j) - accJacobianFD(i, j)) != 0)
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

int testOrientationsJacobians(int errcode, double relativeErrorThreshold, double threshold) // 2
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

int testAnalyticalAJacobianVsFD(int errcode, double relativeErrorThreshold, double threshold) // 3
{
  Matrix A_analytic = ko_.computeAMatrix();

  Matrix A_FD = ko_.getEKF().getAMatrixFD(dx_);

  std::cout << std::endl
            << "Analytical : " << std::endl
            << displayVectorWithIndex(A_analytic).format(CleanFmt_) << std::endl;
  std::cout << std::endl << "FD : " << std::endl << displayVectorWithIndex(A_FD).format(CleanFmt_) << std::endl;

  bool stopIT = false;

  std::cout << std::endl
            << "\033[1;34m"
            << "New iteration"
            << "\033[0m\n"
            << std::endl;

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

  if(error_ > relativeErrorThreshold)
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

  /* Kinetics Observer initialization */
  ori.setRandom();
  worldContactOri1.setRandom();

  Kinematics worldContactPose1;
  worldContactPose1.position = worldContactPos1;
  worldContactPose1.orientation = worldContactOri1;
  Kinematics centroidContactPose1;
  centroidContactPose1.position = centroidContactPos1;
  centroidContactPose1.orientation = centroidContactOri1;
  centroidContactPose1.linVel = centroidContactLinVel1;
  centroidContactPose1.angVel = centroidContactAngVel1;

  ko_.setCenterOfMass(com, com_d, com_dd);

  // ko_.setMass(4000);

  ko_.setSamplingTime(dt_);
  ko_.setWithUnmodeledWrench(true);
  ko_.useRungeKutta(false);
  ko_.setWithGyroBias(true);

  ko_.setAngularMomentum(angularMomentum, angularMomentum_d);
  inertiaMatrix_ = inertiaMatrix_ * inertiaMatrix_.transpose();
  inertiaMatrix_d_ = inertiaMatrix_d_ * inertiaMatrix_d_.transpose();

  ko_.setInertiaMatrix(inertiaMatrix_, inertiaMatrix_d_);

  ko_.addContact(worldContactPose1, 0, K1, K2, K3, K4);
  ko_.updateContactWithNoSensor(centroidContactPose1, 0);

  if(secondContactAndGyro_)
  {
    worldContactOri2.setRandom();

    Kinematics worldContactPose2;
    worldContactPose2.position = worldContactPos2;
    worldContactPose2.orientation = worldContactOri2;
    Kinematics centroidContactPose2;
    centroidContactPose2.position = centroidContactPos2;
    centroidContactPose2.orientation = centroidContactOri2;
    centroidContactPose2.linVel = centroidContactLinVel2;
    centroidContactPose2.angVel = centroidContactAngVel2;

    Matrix3 K1_2 = lin_stiffness_ * Matrix3::Identity();
    Matrix3 K2_2 = lin_damping_ * Matrix3::Identity();
    Matrix3 K3_2 = ang_stiffness_ * Matrix3::Identity();
    Matrix3 K4_2 = ang_damping_ * Matrix3::Identity();

    ko_.addContact(worldContactPose1, 1, K1_2, K2_2, K3_2, K4_2);
    ko_.updateContactWithNoSensor(centroidContactPose2, 1);

    stateVector_.resize(position.size() + 4 + linvel.size() + angvel.size() + gyroBias1.size() + gyroBias2.size()
                        + extForces.size() + extTorques.size() + worldContactPos1.size()
                        + worldContactOri1.toVector4().size() + contactForces1.size() + contactTorques1.size()
                        + worldContactPos2.size() + worldContactOri2.toVector4().size() + contactForces2.size()
                        + contactTorques2.size());
    stateVector_ << position, ori.toVector4(), linvel, angvel, gyroBias1, gyroBias1, extForces, extTorques,
        worldContactPos1, worldContactOri1.toVector4(), contactForces1, contactTorques1, worldContactPos2,
        worldContactOri2.toVector4(), contactForces2, contactTorques2;
  }
  else
  {
    stateVector_.resize(position.size() + 4 + linvel.size() + angvel.size() + gyroBias1.size() + extForces.size()
                        + extTorques.size() + worldContactPos1.size() + worldContactOri1.toVector4().size()
                        + contactForces1.size() + contactTorques1.size());
    stateVector_ << position, ori.toVector4(), linvel, angvel, gyroBias1, extForces, extTorques, worldContactPos1,
        worldContactOri1.toVector4(), contactForces1, contactTorques1;
  }

  std::cout << std::endl << "State vector : " << std::endl << stateVector_ << std::endl;

  ko_.setInitWorldCentroidStateVector(stateVector_);

  ko_.getEKF().updateStatePrediction();

  dx_.resize(ko_.getStateTangentSize());
  dx_.setZero();
  dx_.setConstant(1e-5);

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

  std::cout << std::endl << "dx_ : " << std::endl << dx_ << std::endl;

  std::cout << "Starting testAccelerationsJacobians." << std::endl;
  if((returnVal = testAccelerationsJacobians(++errorcode, 0.1, 1e-4)))
  {
    std::cout << "testAccelerationsJacobians Failed, error code: " << returnVal << std::endl;
  }
  else
  {
    std::cout << "testAccelerationsJacobians succeeded" << std::endl;
  }

  std::cout << "Starting testOrientationsJacobians." << std::endl;
  if((returnVal = testOrientationsJacobians(++errorcode, 0.1, 1e-4)))
  {
    std::cout << "testOrientationsJacobians Failed, error code: " << returnVal << std::endl;
  }
  else
  {
    std::cout << "testOrientationsJacobians succeeded" << std::endl;
  }

  std::cout << "Starting testAnalyticalAJacobianVsFD." << std::endl;
  if((returnVal = testAnalyticalAJacobianVsFD(++errorcode, 0.05, 1e-4)))
  {
    std::cout << "testAnalyticalAJacobianVsFD Failed, error code: " << returnVal << std::endl;
  }
  else
  {
    std::cout << "testAnalyticalAJacobianVsFD succeeded" << std::endl;
  }

  std::cout << "test succeeded" << std::endl;
  return 0;
}