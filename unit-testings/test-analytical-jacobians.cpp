#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

using namespace stateObservation::kine;

namespace stateObservation
{

double dt_ = 0.005;

/// finite difference step.
const double h_ = 1e-5;

/// relative tolerance on each coefficient, w.r.t. the largest coefficient of its row
const double relTol_ = 1e-5;

std::mt19937 gen_;
std::uniform_real_distribution<double> uniform_(-1.0, 1.0);

Vector3 randVec3()
{
  return Vector3(uniform_(gen_), uniform_(gen_), uniform_(gen_));
}

Orientation randOri()
{
  Orientation o;
  o.setRandom();
  return o;
}

///////////////////////////////////////////////////////////////////////
/// -------------------Intermediary functions for the tests-------------
///////////////////////////////////////////////////////////////////////

/// @brief Compares two Jacobian matrices coefficient by coefficient.
/// @param name  name used in the error messages
/// @param analytic the analytical Jacobian matrix
/// @param fd the finite differences Jacobian matrix
/// @return the number of mismatching coefficients
int compareJacobians(const std::string & name,
                     const Matrix & analytic,
                     const Matrix & fd)
{
  if(analytic.rows() != fd.rows() || analytic.cols() != fd.cols())
  {
    std::cout << "\033[1;31m" << name << ": size mismatch, analytic is " << analytic.rows() << "x" << analytic.cols()
              << " and FD is " << fd.rows() << "x" << fd.cols() << "\033[0m" << std::endl;
    return 1;
  }
  if(!analytic.allFinite() || !fd.allFinite())
  {
    std::cout << "\033[1;31m" << name << ": non-finite coefficients\033[0m" << std::endl;
    return 1;
  }

  int mismatches = 0;
  for(Index i = 0; i < analytic.rows(); ++i)
  {
    const double scale = std::max(analytic.row(i).cwiseAbs().maxCoeff(), fd.row(i).cwiseAbs().maxCoeff());
    for(Index j = 0; j < analytic.cols(); ++j)
    {
      const double error = std::abs(analytic(i, j) - fd(i, j));
      if(error > relTol_ * scale)
      {
        if(mismatches < 20)
        {
          std::cout << "\033[1;31m" << name << "(" << i << "," << j << "):  analytic " << std::setw(14)
                    << analytic(i, j) << "    FD " << std::setw(14) << fd(i, j) << "    error " << error
                    << "    (scale " << scale << ")\033[0m" << std::endl;
        }
        ++mismatches;
      }
    }
  }
  if(mismatches > 20)
  {
    std::cout << "\033[1;31m" << name << ": ... and " << mismatches - 20 << " more\033[0m\n";
  }
  return mismatches;
}

///////////////////////////////////////////////////////////////////////
/// -------------------Tests implementation-------------
///////////////////////////////////////////////////////////////////////

/// @brief Checks the Jacobian matrices of the local linear and angular accelerations
/// (Eqs. jac_a_R, jac_a_Fe, jac_a_Fi, jac_omegadot_omega, jac_omegadot_Te,
/// jac_omegadot_Fi and jac_omegadot_Ti of the appendix) against finite differences.
int testAccelerationsJacobians(KineticsObserver & ko_, int errcode, double /* unused */, double /* unused */) // 1
{
  const Index n = ko_.getStateTangentSize();
  const Vector x = ko_.getEKF().getCurrentEstimatedState();

  /* Finite differences Jacobian */
  Matrix accJacobianFD = Matrix::Zero(6, n);
  Vector accPlus = Vector6::Zero();
  Vector accMinus = Vector6::Zero();
  Vector increment(n), xPlus(n), xMinus(n);

  for(Index i = 0; i < n; ++i)
  {
    increment.setZero();
    increment[i] = h_;
    ko_.stateSum(x, increment, xPlus);
    increment[i] = -h_;
    ko_.stateSum(x, increment, xMinus);

    ko_.computeLocalAccelerations(xPlus, accPlus);
    ko_.computeLocalAccelerations(xMinus, accMinus);

    accJacobianFD.col(i) = (accPlus - accMinus) / (2.0 * h_);
  }

  /* Analytical jacobian, written from the expressions of the appendix */

  LocalKinematics worldCentroidKinematics(x, KineticsObserver::flagsStateKine);
  Matrix accJacobianAnalytical = Matrix::Zero(6, n);
  const Matrix3 I_inv = ko_.getInertiaMatrix()().inverse();

  // Jacobian matrices of the linear acceleration
  accJacobianAnalytical.block<3, KineticsObserver::sizeOriTangent>(0, ko_.oriIndexTangent()) =
      -cst::gravityConstant
      * (worldCentroidKinematics.orientation.toMatrix3().transpose() * kine::skewSymmetric(Vector3(0, 0, 1)));
  // when the unmodeled wrench is disabled it is not part of the estimated state any
  // more: stateSum() leaves those coordinates untouched, so their columns are zero
  if(ko_.withUnmodeledWrench_)
  {
    accJacobianAnalytical.block<3, KineticsObserver::sizeForceTangent>(0, ko_.unmodeledForceIndexTangent()) =
        Matrix3::Identity() / ko_.getMass();
    accJacobianAnalytical.block<3, KineticsObserver::sizeTorqueTangent>(3, ko_.unmodeledTorqueIndexTangent()) = I_inv;
  }

  // Jacobian matrices of the angular acceleration
  accJacobianAnalytical.block<3, KineticsObserver::sizeAngVelTangent>(3, ko_.angVelIndexTangent()) =
      I_inv
      * (kine::skewSymmetric(ko_.getInertiaMatrix()() * worldCentroidKinematics.angVel()) - ko_.getInertiaMatrixDot()()
         - kine::skewSymmetric(worldCentroidKinematics.angVel()) * ko_.getInertiaMatrix()()
         + kine::skewSymmetric(ko_.getAngularMomentum()()));

  // Jacobian matrices with respect to the contacts
  for(KineticsObserver::Input::VectorContactConstIterator i = ko_.input_.contacts_.begin();
      i != ko_.input_.contacts_.end(); ++i)
  {
    if(i->isSet)
    {
      // Jacobian matrix of the linear acceleration with respect to the contact force
      accJacobianAnalytical.block<3, KineticsObserver::sizeForceTangent>(0, ko_.contactForceIndexTangent(i)) =
          (1.0 / ko_.getMass()) * i->centroidContactKine.orientation.toMatrix3();
      // Jacobian matrix of the angular acceleration with respect to the contact force
      accJacobianAnalytical.block<3, KineticsObserver::sizeTorqueTangent>(3, ko_.contactForceIndexTangent(i)) =
          (I_inv * kine::skewSymmetric(i->centroidContactKine.position()))
          * (i->centroidContactKine.orientation).toMatrix3();
      // Jacobian matrix of the angular acceleration with respect to the contact torque
      accJacobianAnalytical.block<3, KineticsObserver::sizeTorqueTangent>(3, ko_.contactTorqueIndexTangent(i)) =
          I_inv * i->centroidContactKine.orientation.toMatrix3();
    }
  }

  return compareJacobians("accelerations", accJacobianAnalytical, accJacobianFD) ? errcode : 0;
}

/// @brief Checks the Jacobian matrix of the orientation integration with respect to the
/// increment rotation vector theta (subsection "Jacobian matrices for the orientation
/// state-transition model" of the appendix) against finite differences.
int testOrientationsJacobians(KineticsObserver & ko_, int errcode, double /* unused */, double /* unused */) // 2
{
  const Vector currentState = ko_.getEKF().getCurrentEstimatedState();
  Vector accelerations = Vector6::Zero();
  ko_.computeLocalAccelerations(currentState, accelerations);
  LocalKinematics kineTestOri(currentState);

  kineTestOri.linAcc = accelerations.segment<3>(0);
  kineTestOri.angAcc = accelerations.segment<3>(3);

  const Vector3 theta = dt_ * kineTestOri.angVel() + dt_ * dt_ / 2 * kineTestOri.angAcc();

  /* Finite differences Jacobian */
  Matrix rotationJacobianDeltaFD = Matrix::Zero(3, 3);
  Vector3 increment = Vector3::Zero();
  for(Index i = 0; i < 3; ++i)
  {
    increment.setZero();
    increment[i] = h_;

    Orientation oriMinus = kineTestOri.orientation;
    Orientation oriPlus = kineTestOri.orientation;
    oriMinus.integrateRightSide(theta - increment);
    oriPlus.integrateRightSide(theta + increment);

    rotationJacobianDeltaFD.col(i) = oriMinus.differentiate(oriPlus) / (2.0 * h_);
  }

  /* Analytical Jacobian */
  const double normTheta = theta.norm();
  const Matrix rotationJacobianDeltaAnalytical =
      2.0 / normTheta
      * (((normTheta - 2.0 * sin(normTheta / 2.0)) / (2.0 * theta.squaredNorm())) * kineTestOri.orientation.toMatrix3()
             * theta * theta.transpose()
         + sin(normTheta / 2.0) * kineTestOri.orientation.toMatrix3()
               * kine::rotationVectorToRotationMatrix(theta / 2.0));

  return compareJacobians("orientation integration", rotationJacobianDeltaAnalytical, rotationJacobianDeltaFD) ? errcode
                                                                                                               : 0;
}

/// @brief Checks the analytical state-transition Jacobian matrix A against central
/// finite differences taken on the state manifold.
int testAnalyticalAJacobianVsFD(KineticsObserver & ko_, int errcode, double /* unused */, double /* unused */) // 3
{
  const Matrix A_analytic = ko_.computeAMatrix();

  const Index n = ko_.getStateTangentSize();
  const Vector x = ko_.getEKF().getCurrentEstimatedState();

  Matrix A_FD(n, n);
  Vector increment(n), xPlus(n), xMinus(n), fPlus, fMinus, difference(n);
  for(Index i = 0; i < n; ++i)
  {
    increment.setZero();
    increment[i] = h_;
    ko_.stateSum(x, increment, xPlus);
    increment[i] = -h_;
    ko_.stateSum(x, increment, xMinus);

    fPlus = ko_.stateDynamics(xPlus, InputT<>(), 0);
    fMinus = ko_.stateDynamics(xMinus, InputT<>(), 0);

    ko_.stateDifference(fPlus, fMinus, difference);
    A_FD.col(i) = difference / (2.0 * h_);
  }

  return compareJacobians("A", A_analytic, A_FD) ? errcode : 0;
}

/// @brief Checks the analytical observation Jacobian matrix C against central finite
/// differences taken on the state manifold.
int testAnalyticalCJacobianVsFD(KineticsObserver & ko_, int errcode, double /* unused */, double /* unused */) // 4
{
  const Matrix C_analytic = ko_.computeCMatrix();

  const Index n = ko_.getStateTangentSize();
  const Index m = ko_.measurementTangentSize_;
  // C linearises the measurement model around the *predicted* state
  const TimeIndex k = ko_.getEKF().getCurrentTime() + 1;
  const Vector xBar = ko_.getEKF().updateStatePrediction();

  Matrix C_FD(m, n);
  Vector increment(n), xPlus(n), xMinus(n), yPlus, yMinus, difference(m);
  for(Index i = 0; i < n; ++i)
  {
    increment.setZero();
    increment[i] = h_;
    ko_.stateSum(xBar, increment, xPlus);
    increment[i] = -h_;
    ko_.stateSum(xBar, increment, xMinus);

    yPlus = ko_.measureDynamics(xPlus, InputT<>(), k);
    yMinus = ko_.measureDynamics(xMinus, InputT<>(), k);

    ko_.measurementDifference(yPlus, yMinus, difference);
    C_FD.col(i) = difference / (2.0 * h_);
  }

  return compareJacobians("C", C_analytic, C_FD) ? errcode : 0;
}

///////////////////////////////////////////////////////////////////////
/// -------------------Scenario construction-------------
///////////////////////////////////////////////////////////////////////

struct Scenario
{
  std::string name;
  int nbContacts = 1;
  int nbIMUs = 1;
  bool withUnmodeledWrench = true;
  bool withGyroBias = true;
  bool withWrenchSensors = true;
  bool withAbsolutePoseSensor = false;
  bool withAbsoluteOriSensor = false;
};

/// @brief Builds a Kinetics Observer in a random but valid state matching the scenario.
/// @return the number of errors reported by the four tests
int runScenario(const Scenario & scenario, unsigned seed)
{
  gen_.seed(seed);
  tools::ProbabilityLawSimulation::setSeed(seed);

  KineticsObserver ko(unsigned(scenario.nbContacts), unsigned(scenario.nbIMUs));
  ko.setSamplingTime(dt_);
  ko.setWithUnmodeledWrench(scenario.withUnmodeledWrench);
  ko.setWithGyroBias(scenario.withGyroBias);

  ko.setCenterOfMass(randVec3() / 10, randVec3() / 10, randVec3() / 10);

  // a valid inertia matrix is symmetric positive definite, its derivative is only symmetric
  Matrix3 inertiaMatrix = tools::ProbabilityLawSimulation::getUniformMatrix<Matrix3>();
  inertiaMatrix = inertiaMatrix * inertiaMatrix.transpose() + 3 * Matrix3::Identity();
  Matrix3 inertiaMatrixDot = tools::ProbabilityLawSimulation::getGaussianMatrix<Matrix3>();
  inertiaMatrixDot = 0.5 * (inertiaMatrixDot + inertiaMatrixDot.transpose());
  ko.setCoMInertiaMatrix(inertiaMatrix, inertiaMatrixDot);
  ko.setCoMAngularMomentum(randVec3() / 10, randVec3() / 10);

  std::vector<Vector3> worldContactPos(size_t(scenario.nbContacts));
  std::vector<Orientation> worldContactOri(size_t(scenario.nbContacts));

  for(int c = 0; c < scenario.nbContacts; ++c)
  {
    const double linStiffness = 1e4 * (0.5 + 0.5 * std::abs(uniform_(gen_)));
    const double linDamping = 5e1 * (0.5 + 0.5 * std::abs(uniform_(gen_)));
    const double angStiffness = 1e3 * (0.5 + 0.5 * std::abs(uniform_(gen_)));
    const double angDamping = 1e1 * (0.5 + 0.5 * std::abs(uniform_(gen_)));

    // non isotropic stiffness/damping, so that a transposition error cannot go unnoticed
    const Matrix3 K1 = linStiffness * Vector3(1.0, 0.8, 1.2).asDiagonal();
    const Matrix3 K2 = linDamping * Vector3(1.0, 0.7, 1.3).asDiagonal();
    const Matrix3 K3 = angStiffness * Vector3(1.0, 0.9, 1.1).asDiagonal();
    const Matrix3 K4 = angDamping * Vector3(1.0, 0.6, 1.4).asDiagonal();

    worldContactPos[size_t(c)] = randVec3() / 10;
    worldContactOri[size_t(c)] = randOri();
    Kinematics worldContactPose;
    worldContactPose.position = worldContactPos[size_t(c)];
    worldContactPose.orientation = worldContactOri[size_t(c)];

    Kinematics centroidContactPose;
    centroidContactPose.position = randVec3() / 10;
    centroidContactPose.orientation = randOri();
    centroidContactPose.linVel = randVec3() / 10;
    centroidContactPose.angVel = randVec3() / 10;

    ko.addContact(worldContactPose, c, K1, K2, K3, K4);
    if(scenario.withWrenchSensors)
    {
      ko.updateContactWithWrenchSensor(Vector6::Zero(), centroidContactPose, unsigned(c));
    }
    else
    {
      ko.updateContactWithNoSensor(centroidContactPose, unsigned(c));
    }
  }

  for(int j = 0; j < scenario.nbIMUs; ++j)
  {
    Kinematics centroidIMUPose;
    centroidIMUPose.position = randVec3() / 10;
    centroidIMUPose.orientation = randOri();
    centroidIMUPose.linVel = randVec3() / 10;
    centroidIMUPose.angVel = randVec3() / 10;
    centroidIMUPose.linAcc = randVec3() / 10;
    centroidIMUPose.angAcc = randVec3() / 10;
    ko.setIMU(Vector3::Zero(), Vector3::Zero(), centroidIMUPose, j);
  }

  if(scenario.withAbsolutePoseSensor)
  {
    Kinematics absPose;
    absPose.position = randVec3();
    absPose.orientation = randOri();
    ko.setAbsolutePoseSensor(absPose);
  }
  if(scenario.withAbsoluteOriSensor)
  {
    ko.setAbsoluteOriSensor(randOri());
  }

  /* State vector */
  Vector stateVector(ko.getStateSize());
  stateVector.setZero();
  Index index = 0;
  stateVector.segment<3>(index) = randVec3() / 10;
  index += 3; // position
  stateVector.segment<4>(index) = randOri().toVector4();
  index += 4; // orientation
  stateVector.segment<3>(index) = randVec3() / 10;
  index += 3; // linear velocity
  stateVector.segment<3>(index) = randVec3() / 10;
  index += 3; // angular velocity
  for(int j = 0; j < scenario.nbIMUs; ++j)
  {
    stateVector.segment<3>(index) = randVec3() / 10;
    index += 3; // gyrometer bias
  }
  stateVector.segment<3>(index) = randVec3() / 10;
  index += 3; // unmodeled force
  stateVector.segment<3>(index) = randVec3() / 10;
  index += 3; // unmodeled torque
  for(int c = 0; c < scenario.nbContacts; ++c)
  {
    stateVector.segment<3>(index) = worldContactPos[size_t(c)];
    index += 3;
    stateVector.segment<4>(index) = worldContactOri[size_t(c)].toVector4();
    index += 4;
    stateVector.segment<3>(index) = randVec3() * 100;
    index += 3; // contact force
    stateVector.segment<3>(index) = randVec3() * 10;
    index += 3; // contact torque
  }
  BOOST_ASSERT(index == ko.getStateSize() && "the test builds a state vector of the wrong size");
  if(index != ko.getStateSize())
  {
    std::cout << "\033[1;31mstate vector size mismatch: " << index << " vs " << ko.getStateSize() << "\033[0m\n";
    return 1;
  }

  ko.setInitWorldCentroidStateVector(stateVector);
  ko.updateMeasurements();
  ko.getEKF().updateStatePrediction();

  std::cout << "--- " << scenario.name << " (seed " << seed << ")" << std::endl;

  int errors = 0;
  errors += testAccelerationsJacobians(ko, 1, 0, 0) ? 1 : 0;
  errors += testOrientationsJacobians(ko, 1, 0, 0) ? 1 : 0;
  errors += testAnalyticalAJacobianVsFD(ko, 1, 0, 0) ? 1 : 0;
  errors += testAnalyticalCJacobianVsFD(ko, 1, 0, 0) ? 1 : 0;

  if(errors == 0)
  {
    std::cout << "    ok" << std::endl;
  }
  return errors;
}

} // end namespace stateObservation

using namespace stateObservation;

int main()
{
  std::vector<Scenario> scenarios;

  {
    Scenario s;
    s.name = "1 contact, 1 IMU";
    scenarios.push_back(s);
  }
  {
    Scenario s;
    s.name = "2 contacts, 2 IMUs";
    s.nbContacts = 2;
    s.nbIMUs = 2;
    scenarios.push_back(s);
  }
  {
    Scenario s;
    s.name = "3 contacts, 1 IMU";
    s.nbContacts = 3;
    scenarios.push_back(s);
  }
  {
    Scenario s;
    s.name = "contacts without wrench sensor";
    s.nbContacts = 2;
    s.withWrenchSensors = false;
    scenarios.push_back(s);
  }
  {
    Scenario s;
    s.name = "without unmodeled wrench";
    s.withUnmodeledWrench = false;
    scenarios.push_back(s);
  }
  {
    Scenario s;
    s.name = "without gyrometer bias";
    s.withGyroBias = false;
    scenarios.push_back(s);
  }
  {
    Scenario s;
    s.name = "with absolute pose sensor";
    s.withAbsolutePoseSensor = true;
    scenarios.push_back(s);
  }
  {
    Scenario s;
    s.name = "with absolute orientation sensor";
    s.withAbsoluteOriSensor = true;
    scenarios.push_back(s);
  }
  {
    Scenario s;
    s.name = "2 contacts, 2 IMUs, both absolute sensors";
    s.nbContacts = 2;
    s.nbIMUs = 2;
    s.withAbsolutePoseSensor = true;
    s.withAbsoluteOriSensor = true;
    scenarios.push_back(s);
  }

  int errors = 0;
  for(unsigned seed : {1u, 7u, 42u})
  {
    for(const Scenario & s : scenarios)
    {
      errors += runScenario(s, seed);
    }
  }

  if(errors)
  {
    std::cout << "\033[1;31mtest failed: " << errors << " failing Jacobian comparisons\033[0m" << std::endl;
    return errors;
  }

  std::cout << "test succeeded" << std::endl;
  return 0;
}
