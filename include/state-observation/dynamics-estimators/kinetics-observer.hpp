/**
 * \file      kinetics-observer.hpp
 * \author    Mehdi Benallegue
 * \date      2018
 * \brief     Unified Kinetics estimator
 *
 * \details
 *
 *
 */

#ifndef KINETICSOBSERVER_HPP
#define KINETICSOBSERVER_HPP

#include <set>

#include <boost/utility.hpp>

#include <state-observation/api.h>
#include <state-observation/dynamical-system/dynamical-system-functor-base.hpp>
#include <state-observation/noise/noise-base.hpp>
#include <state-observation/observer/extended-kalman-filter.hpp>
#include <state-observation/sensors-simulation/accelerometer-gyrometer.hpp>
#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>
#include <state-observation/tools/state-vector-arithmetics.hpp>

namespace stateObservation
{

/// @brief This observer estimates the kinematics, the external forces, the bias on the gyrometers measurements, and the
/// contacts forces and pose.

/// @details  Our observer estimates the localkinematics of the centroid's frame within the world frame. The reason to
/// choose the centroid's frame is that it simplifies many expressions, for example the expressions of the
/// accelerations. This estimation is based on the assumption of viscoelastic contacts and using three kinds of
/// measurements: IMUs, Force/Torque measurements (contact and other ones) and any absolute position measurements.
/// Inputs are given in a frame whose choice is at the user's discretion, we therefore call it the user frame.

///
class STATE_OBSERVATION_DLLAPI KineticsObserver : protected DynamicalSystemFunctorBase, protected StateVectorArithmetics
{
public:
  typedef kine::Kinematics Kinematics;
  typedef kine::LocalKinematics LocalKinematics;
  typedef kine::Orientation Orientation;

  // ////////////////////////////////////////////////////////////
  /// @name Constructors and destructors
  // ///////////////////////////////////////////////////////////
  /// @{

  /// @brief Construct a new Kinetics Observer
  ///
  /// @param maxContacts maximum number of contacts between the robot and the environment. These do not include the
  /// additional forces nor the estimated unmodeled forces
  /// @param maxNumberOfIMU the maximum number of IMUs. They don't have to give measurements at each iterations and they
  /// don't have to be synchronized
  KineticsObserver(unsigned maxContacts = 4, unsigned maxNumberOfIMU = 1);

  /// @brief Destroy the Kinetics Observer
  ///
  virtual ~KineticsObserver();

  /// @}

  // ////////////////////////////////////////////////////////////
  /// @name Setting and getting parameters
  /// For initialization and update of parameters that should not evolve over
  /// time on a sample basis.
  // ///////////////////////////////////////////////////////////

  /// @{

  /// @brief Get the Sampling Time
  ///
  /// @return const double &
  double getSamplingTime() const;

  /// @brief Set the Sampling Time
  ///
  void setSamplingTime(double);

  /// @brief Set if the unmodeled and unmeasured external wrench should be
  ///         estimated.
  /// @details Activating this estimation will assume that the contact exists
  ///          therefore, it is likely to modify the value of the estimated state.
  ///          The estimation will also be slower.
  ///
  /// @param b
  void setWithUnmodeledWrench(bool b = true);

  /// @brief Sets if the estimation computes also the accelerations
  /// @details This will not modify the estimated value, but just compute
  ///          the modeled acceleration, which gives a model-based filtered
  ///           acceleration
  ///
  /// @param b
  void setWithAccelerationEstimation(bool b = true);

  /// @brief Returns if the estimation computes also the accelerations
  ///
  /// @return True if the acceleration is also estimated. Returns false otherwise.
  bool getWithAccelerationEstimation() const;

  /// @brief Returns if the process covariance matrix associated with the contact rest poses must be adapted such that
  /// the process variance on the average pose is zero.
  ///
  /// @return True if the process covariance is adapted. Returns false otherwise.
  bool getWithAdaptativeContactProcessCov() const;

  /// @brief Set if the gyrometers bias is computed or not.
  ///        This parameter is global for all the IMUs.
  ///
  /// @param b
  void setWithGyroBias(bool b = true);

  /// @brief Set if the damping component of the visco-elastic contact force must be used in the annalytical A matrix.
  ///
  /// @param b
  void setWithDampingInMatrixA(bool b = true);

  /// @brief Set if the process covariance matrix associated with the contact rest poses must be adapted such that the
  /// process variance on the average pose is zero.
  ///
  /// @param b
  void setWithAdaptativeContactProcessCov(bool b = true);

  /// @brief Set the total mass of the robot. This can be changed online
  ///
  /// @return sets
  void setMass(double);

  /// @brief Returns the mass of the robot
  ///
  /// @return the mass of the robot.
  double getMass() const;

  /// @brief Returns the global inertia matrix of the robot at the center of mass.
  ///
  /// @return The global inertia matrix of the robot at the center of mass.
  const IndexedMatrix3 & getInertiaMatrix() const;

  /// @brief Returns the derivative of the global inertia matrix of the robot at the center of mass.
  ///
  /// @return The derivative of the global inertia matrix of the robot at the center of mass.
  const IndexedMatrix3 & getInertiaMatrixDot() const;

  /// @brief Returns the angular momentum of the robot at the center of mass.
  ///
  /// @return The angular momentum of the robot at the center of mass.
  const IndexedVector3 & getAngularMomentum() const;

  /// @brief Returns the derivative of the angular momentum of the robot at the center of mass.
  ///
  /// @return The derivative of the angular momentum of the robot at the center of mass.
  const IndexedVector3 & getAngularMomentumDot() const;

  /// @brief Returns the position of the CoM of the robot in the user frame
  ///
  /// @return The position of the center of mass of the robot in the user frame.
  const IndexedVector3 & getCenterOfMass() const;

  /// @brief Returns the linear velocity of the CoM of the robot in the user frame
  ///
  /// @return The linear velocity of the center of mass of the robot in the user frame.
  const IndexedVector3 & getCenterOfMassDot() const;

  /// @brief Returns the linear acceleration of the CoM of the robot in the user frame
  ///
  /// @return The linear acceleration of the center of mass of the robot in the user frame.
  const IndexedVector3 & getCenterOfMassDotDot() const;

  /// @brief Returns the input additional wrench, expressed in the centroid frame
  ///
  /// @return The input additional wrench, expressed in the centroid frame.
  Vector6 getAdditionalWrench() const;

  /// @}

  // ///////////////////////////////////////////////////////////
  /// @name Setting kinematic sensors
  /// These are the methods to be called at each iteration to give the control
  /// inputs and the sensor measurement for IMUs and absolute pose sensors.
  /// //////////////////////////////////////////////////////////

  /// @{

  /// @brief Set the measurements of an IMU and give the Kinematic of the IMU in the user frame.
  ///
  /// @details The overload that does not have the covariance matrices as an
  /// inputs uses default ones.
  ///
  /// The IMU is located in a sensor frame. We suppose we know the kinematics of
  /// this sensor frame in the centroid's frame
  ///
  /// @return the number of the IMU (useful in case there are several ones)
  /// @param accelero measured value
  /// @param gyrometer measured gyro value
  /// @param userImuKinematics sets the kinematics of the IMU in the user frame. The best is to provide the position,
  /// the orientation, the angular and linear velocities and the linear acceleration Nevertheless if velocities or
  /// accelerations are not available they will be automatically computed through finite differences
  /// @param num the number of the IMU (useful in case there are several ones).
  ///           If not set it will be generated automatically.
  Index setIMU(const Vector3 & accelero,
               const Vector3 & gyrometer,
               const Kinematics & userImuKinematics,
               Index num = -1);

  /// @brief @copybrief setIMU(const Vector3&,const Vector3&,const Kinematics &,int)
  /// Provides also the associated covariance matrices
  /// @details
  /// This version specifies the covariance matrices of these measurements.
  /// @copydetails setIMU(const Vector3&,const Vector3&,const Kinematics &,int)
  /// @param acceleroCov The covariance matrix of the accelerometer
  /// @param gyroCov The covariance matrix of the gyrometer
  Index setIMU(const Vector3 & accelero,
               const Vector3 & gyrometer,
               const Matrix3 & acceleroCov,
               const Matrix3 & gyroCov,
               const Kinematics & userImuKinematics,
               Index num = -1);

  /// @brief set the default covariance matrix for IMU.
  /// @details this is used to set the covariances wgen not given explicitely
  /// (see setIMU(const Vector3&,const Vector3&,const Kinematics &,int)).
  /// @param acceleroCov The covariance matrix of the accelerometer
  /// @param gyroCov The covariance matrix of the gyrometer
  void setIMUDefaultCovarianceMatrix(const Matrix3 & acceleroCov, const Matrix3 & gyroCov);

  /// @brief Set an Absolute Pose Sensor measurement
  /// The measurement is the kinematics namely position and orientation of the observed frame in
  /// the global frame.
  /// @details The overload with the measurement only uses default covariance
  /// matrix.
  /// @param measurement The measurement in the form of a Kinematics object
  void setAbsolutePoseSensor(const Kinematics & measurement);

  /// @brief @copybrief setAbsolutePoseSensor(const Kinematics &)
  ///
  /// @details This version sets the Covariance matrix explicitely.
  /// @copydetails setAbsolutePoseSensor(const Kinematics &)
  /// @param CovarianceMatrix the covariance matrix
  void setAbsolutePoseSensor(const Kinematics & measurement, const Matrix6 & CovarianceMatrix);

  /// @brief Set the Absolute Pose Sensor Default Covariance Matrix
  /// @param covMat
  void setAbsolutePoseSensorDefaultCovarianceMatrix(const Matrix6 & covMat);

  /// @brief Set an Absolute Orientation Sensor measurement
  /// The measurement is the orientation of the observed frame in the global frame.
  /// @details The overload with the measurement only uses default covariance  matrix.
  /// @param measurement The measurement in the form of an Orientation object
  void setAbsoluteOriSensor(const Orientation & measurement);

  /// @brief @copybrief setAbsoluteOriSensor(const Orientation &)
  ///
  /// @details This version sets the Covariance matrix explicitely.
  /// @copydetails setAbsoluteOriSensor(const Orientation &)
  /// @param CovarianceMatrix the covariance matrix
  void setAbsoluteOriSensor(const Orientation & measurement, const Matrix3 & CovarianceMatrix);

  /// @brief Set the Absolute Orientation Sensor Default Covariance Matrix
  /// @param covMat
  void setAbsoluteOriSensorDefaultCovarianceMatrix(const Matrix3 & covMat);

  /// @}

  // ///////////////////////////////////////////////////////////
  /// @name Adding, managing and deleting contacts
  /// @details This class does NOT detect new contacts with the environment. Use these classes instead.
  /// Call these only when a new contact is created or removed from the environment, otherwise the contacts
  /// will remain the same at each new iteration.
  // ///////////////////////////////////////////////////////////
  /// @{

  /// @brief Set a new contact with the environment
  ///
  /// @param pose  is the initial guess on the position of the contact in the WORLD frame. Only position and orientation
  /// are enough. If the contact is compliant, you need to set the "rest" pose of the contact (i.e. the pose that gives
  /// zero reaction force)
  /// @param initialCovarianceMatrix is the covariance matrix expressing the uncertainty in the pose of the initial
  /// guess in the 6x6 upper left corner ( if no good initial guess is available give a rough position with a high
  /// initial covariance matrix, if the position is certain, set it to zero.) and the initial wrench in the 6x6 lower
  /// right corner.
  /// @param processCovarianceMatrix is the covariance matrix expressing the rate at which the contact slides or drifts
  /// in the 6x6 upper left corner (set to zero for no sliding) and the certainty in the reaction force model
  /// (viscoelastic) in the prediction of the contact force
  /// @param contactNumber the number id of the contact to add. If no predefined id, use -1 (default) in order to set
  /// the number automatically
  /// @param linearStiffness the linear stiffness of the contact viscoelastic model, if unknown, set to
  /// Matrix3::Constant(-1) (default) to use the default one
  /// @param linearDamping  the linear damping of the contact viscoelastic model, if unknown, set to
  /// Matrix3::Constant(-1) (default) to use the default one
  /// @param angularStiffness the angular stiffness of the contact viscoelastic model, if unknown, set to
  /// Matrix3::Constant(-1) (default) to use the default one
  /// @param angularDamping the angular damping of the contact viscoelastic model, if unknown, set to
  /// Matrix3::Constant(-1) (default) to use the default one
  /// @return int the id number of the contact just added (returns contactNumber if it is positive)
  Index addContact(const Kinematics & pose,
                   const Matrix12 & initialCovarianceMatrix,
                   const Matrix12 & processCovarianceMatrix,
                   Index contactNumber = -1,
                   const Matrix3 & linearStiffness = Matrix3::Constant(-1),
                   const Matrix3 & linearDamping = Matrix3::Constant(-1),
                   const Matrix3 & angularStiffness = Matrix3::Constant(-1),
                   const Matrix3 & angularDamping = Matrix3::Constant(-1));

  /// @brief Set a new contact with the environment (use default covariance matrices)
  ///
  /// @param pose  is the initial guess on the position of the contact in the WORLD frame. Only position and orientation
  /// are enough
  /// @param contactNumber the number id of the contact to add. If no predefined id, use -1 (default) in order to set
  /// the number automatically
  /// @param linearStiffness the linear stiffness of the contact viscoelastic model, if unknown, set to
  /// Matrix3::Constant(-1) (default) to use the default one
  /// @param linearDamping  the linear damping of the contact viscoelastic model, if unknown, set to
  /// Matrix3::Constant(-1) (default) to use the default one
  /// @param angularStiffness the angular stiffness of the contact viscoelastic model, if unknown, set to
  /// Matrix3::Constant(-1) (default) to use the default one
  /// @param angularDamping the angular damping of the contact viscoelastic model, if unknown, set to
  /// Matrix3:::Constant(-1) (default) to use the default one
  /// @return int the id number of the contact just added (returns contactNumber if it is positive)
  Index addContact(const Kinematics & pose,
                   Index contactNumber = -1,
                   const Matrix3 & linearStiffness = Matrix3::Constant(-1),
                   const Matrix3 & linearDamping = Matrix3::Constant(-1),
                   const Matrix3 & angularStiffness = Matrix3::Constant(-1),
                   const Matrix3 & angularDamping = Matrix3::Constant(-1));

  /// @brief Remove a contact
  ///
  /// @param contactnbr the number of the contact to remove
  void removeContact(Index contactnbr);

  /// @brief remove all the contacts
  void clearContacts();

  /// @brief Get the Current Number Of Contacts
  ///
  /// @return Index The current number of contacts
  Index getNumberOfSetContacts() const;

  /// @brief Get the List Of Contact ids
  ///
  /// @return std::vector<int> a vector listing the contact ids
  std::vector<Index> getListOfContacts() const;

  /// @}

  // ///////////////////////////////////////////////////////////
  /// @name Updating contact information
  /// @details Calling one of the two following methods (updateContactWithWrenchSensor() and
  /// updateContactWithNoSensor()) is MANDATORY for every contact and at every iteration.
  /// - If the contact is equipped with wrench sensor call updateContactWithWrenchSensor()
  /// - If not call updateContactWithNoSensor()
  /// - if the contact is lost, it needs to be explicitely removed using removeContact()
  // //////////////////////////////////////////////////////////
  /// @{

  /// @brief Update the contact when it is NOT equipped with wrench sensor
  /// @param localKine the new kinematics of the contact expressed in the centroid's frame frame
  ///                  the best is to provide the position, the orientation, the angular and the linear velocities.
  ///                  Otherwise they will be computed automatically
  /// @param contactNumber The number id of the contact
  void updateContactWithNoSensor(const Kinematics & localKine, unsigned contactNumber);

  /// @brief Update the contact when it is equipped with wrench sensor
  ///
  /// @param wrenchMeasurement wrenchMeasurement is the measurment vector composed with 3D forces and 3D torques
  /// @copydetails updateContactWithNoSensor()
  void updateContactWithWrenchSensor(const Vector6 & wrenchMeasurement,
                                     const Kinematics & localKine,
                                     unsigned contactNumber);

  /// @brief @copybrief updateContactWithWrenchSensor(const Vector6 &,const Kinematics &,unsigned)
  ///
  /// @details This version sets the Covariance matrix explicitely.
  /// @param wrenchCovMatrix The covariance matrix of the wrench measurement
  /// @copydetails updateContactWithWrenchSensor(const Vector6 &,const Kinematics &,unsigned)
  void updateContactWithWrenchSensor(const Vector6 & wrenchMeasurement,
                                     const Matrix6 & wrenchCovMatrix,
                                     const Kinematics & localKine,
                                     unsigned contactNumber);

  /// @brief Set the Contact Wrench Sensor Default Covariance Matrix
  ///
  /// @param wrenchSensorCovMat the new default covariance matrix
  void setContactWrenchSensorDefaultCovarianceMatrix(const Matrix6 & wrenchSensorCovMat);

  /// @}

  // /////////////////////////////////////////////
  /// @name Setting additional inputs to the dynamical system
  /// @details It is highly recommended to set these inputs at each iteration
  /// ////////////////////////////////////////////////

  /// @{
  /// @brief Set the Center Of Mass kinematics expressed in the user frame
  ///
  /// @param com position
  /// @param com_dot velocity
  /// @param com_dot_dot acceleration
  void setCenterOfMass(const Vector3 & com, const Vector3 & com_dot, const Vector3 & com_dot_dot);

  /// @brief Set the Center Of Mass kinematics expressed in the user frame
  /// @details The acceleration will be computed through finite differences
  ///
  /// @param com position
  /// @param com_dot velocity
  void setCenterOfMass(const Vector3 & com, const Vector3 & com_dot);

  /// @brief Set the Center Of Mass kinematics expressed in the user frame
  /// @details The velocity and acceleration will be computed through finite differences
  ///
  /// @param com position
  void setCenterOfMass(const Vector3 & com);

  /// @brief Set the 3x3 inertia matrix and its derivative expressed in the user frame
  ///
  /// @param I Set the inertia matrix at the CoM
  /// @param I_dot Derivative of inertia matrix
  void setCoMInertiaMatrix(const Matrix3 & I, const Matrix3 & I_dot);

  /// @brief Set the 3x3 inertia matrix expressed in the user frame
  /// @details The derivative will be computed using finite differences
  ///
  /// @param I Inertia matrix
  /// @param I_dot Derivative of inertia matrix
  void setCoMInertiaMatrix(const Matrix3 & I);

  /// @brief Set the inertia matrix and its derivative as a Vector6 expressed in the user frame
  ///
  /// @param I Inertia matrix as a vector containing the diagonal and the three non
  /// diagonal values concatenated
  /// @param I_dot Derivative of inertia matrix expressed in the same way
  void setCoMInertiaMatrix(const Vector6 & I, const Vector6 & I_dot);

  /// @brief Set the inertia matrix as a Vector6 expressed in the user frame
  /// @details The derivative will be computed using finite differences
  ///
  /// @param I Inertia matrix as a vector containing the diagonal and the three non
  /// diagonal values concatenated
  void setCoMInertiaMatrix(const Vector6 & I);

  /// @brief Set the Angular Momentum around the CoM and its derviative expressed in the user frame
  ///
  /// @param sigma The angular momentum
  /// @param sigma_dot The angular momentum derivative
  void setCoMAngularMomentum(const Vector3 & sigma, const Vector3 & sigma_dot);

  /// @brief Set the Angular Momentum around the CoM  expressed in the user frame
  /// @details The derivative will be computed using finite differences
  ///
  /// @param sigma The angular momentum
  void setCoMAngularMomentum(const Vector3 & sigma);

  /// @brief Set any Additional resultant known wrench (e.g. measured external forces and moments but no contact ones)
  /// expressed in the local estimated frame.
  /// @details Set to zero if no forces or unknown
  ///
  /// @param force
  /// @param torque
  void setAdditionalWrench(const Vector3 & force, const Vector3 & torque);

  /// @}

  // ///////////////////////////////////////////////////////////
  /// @name Running and getting the estimations
  /// /////////////////////////////////////////////////////////
  /// @{

  /// @brief Updates the measurements.
  /// @details Updates the measurement sensors and the associated vectors and covariance matrices
  void updateMeasurements();

  /// @brief Runs the estimation.
  /// @details This is the function that allows to
  /// 1- compute the estimation
  /// 2- move to the next iteration (increments time, resets the measurements, etc)
  ///
  /// @return const Vector& The state vector
  const Vector & update();

  /// @brief updates the process and the state covariance matrices associated with contacts.
  /// @details if \ref withAdaptativeContactProcessCov_ is set to true: modifies the process covariance matrix Q such
  /// that the process noise on the rest contact positions allows them to move, but their average position remains
  /// unchanged. This allows for the relaxation of internal forces but prevents drifting.
  void updateContactCovariances();

  /// @brief Returns the predicted Kinematics object of the centroid in the world frame at the time of the measurement
  /// predictions

  /// @brief Converts a given wrench from the user to the centroid frame
  /// @details Performs the conversion of a wrench {force, torque} from the user frame to the centroid frame.
  ///
  void convertWrenchFromUserToCentroid(const Vector3 & forceUserFrame,
                                       const Vector3 & momentUserFrame,
                                       Vector3 & forceCentroidFrame,
                                       Vector3 & momentCentroidFrame);

  /// @brief Converts a given wrench from the centroid to the user frame
  /// @details Performs the conversion of a wrench {force, torque} from the centroid frame to the user frame.
  ///
  void convertWrenchFromCentroidToUser(const Vector3 & forceCentroidFrame,
                                       const Vector3 & momentCentroidFrame,
                                       Vector3 & forceUserFrame,
                                       Vector3 & momentUserFrame);

  /// @brief Get the estimated local Kinematics of the centroid frame in the world frame (local, which means expressed
  /// in the centroid frame).
  /// @details the kinematics are the main output of this observer. It includes the linear and angular position and
  /// velocity but not the accelerations by default. To get the acceleration call estimateAccelerations(). This
  /// method does NOT update the estimation, for this use update().
  ///
  /// @return Kinematics
  LocalKinematics getLocalCentroidKinematics() const;

  /// @brief Get the estimated Kinematics of the centroid frame in the world frame.
  /// @details It includes the linear and angular position and
  /// velocity but not the accelerations by default. To get the acceleration call estimateAccelerations(). This
  /// method does NOT update the estimation, for this use update().
  ///
  /// @return Kinematics
  Kinematics getGlobalCentroidKinematics() const;

  /// @brief gets the Kinematics that include the linear and angular accelerations.
  /// @details This method computes the estimated accelerations from the observed state of the robot. It means this
  /// acceleration is filtered by the model
  ///
  /// @return Kinematics
  LocalKinematics estimateAccelerations();

  /// @brief Get the local kinematics of a given frame (in the user frame) in the centroid frame.
  /// @details The kinematics are linear and angular positions, velocities and optionally accelerations.
  /// @param userBodyKine
  /// @return LocalKinematics
  LocalKinematics getLocalKinematicsOf(const Kinematics & userBodyKine);

  /// @brief Get the global kinematics of a given frame (in the user frame) in the centroid frame.
  /// @details The kinematics are linear and angular positions, velocities and optionally accelerations.
  ///
  /// @param kine
  /// @return Kinematics
  Kinematics getGlobalKinematicsOf(const Kinematics & userBodyKin) const;

  /// get the contact force provided by the estimator
  /// which is different from a contact sensor measurement

  /// @brief Get the Estimated Contact Wrench
  /// This is useful in the case of uncertain wrench sensors or when contact force measurement is not available.
  ///
  /// @param contactNbr
  /// @return Vector6 Wrench
  Vector6 getContactWrench(Index contactNbr) const;

  /// @brief Get the Contact 6D pose n in the global frame
  /// @details The contact position may be uncertain, this estimator uses the input data, the kinematic and the dynamic
  /// models to estimate the position of the contact in the environment This position is the "rest position" of the
  /// contact and therefore is different from what is obtained using forward kinematics because it does not include the
  /// contact deformation due to the contact forces
  ///
  /// @param contactNbr The contact number id
  /// @return Kinematics The pose
  Kinematics getContactPose(Index contactNbr) const;

  /// @brief Get the Unmodeled External Wrench (requires setWithUnmodeledWrench() to true before to update())
  /// @details In the presence of unmodeled and unmeasured external forces and moments, the dynamics of the robot
  /// behaves differently, this difference can be exploited to estimate the resultant of these forces and moments.
  ///
  /// @return Vector6
  Vector6 getUnmodeledWrench() const;

  /// @brief Get the Unmodeled External Wrench (requires setWithUnmodeledWrench() to true before to update()) expressed
  /// in the desired frame.
  /// @details The kinematics of the frame in which we want to express the wrench must be given in the user frame.
  /// M(target) = M(centroid) + targetCentroidKine.position().cross(F(centroid))
  /// With targetCentroidKine.position() = targetUserKine.position + targetUserKine.orientation *
  /// userCentroidKine.position()
  /// @return Vector6
  Vector6 getUnmodeledWrenchIn(const Kinematics & userTargetframeKine);

  /// @}

  /// ///////////////////////////////////////////////////////////
  /// @name Set values for state components
  /// These methods allow to update some parts of the state of the system based on guesses obtained independently
  // /////////////////////////////////////////////////////////

  /// @{
  /// @brief Set the State Kinematics
  /// @details Sets a value for the kinematics part of the state
  ///
  /// @param localKine are the new local kinematics of the state
  /// @param resetContactWrenches set if the contact wrenches should be reset
  /// @param resetCovariance set if the covariance of the state should be reset
  void setWorldCentroidStateKinematics(const LocalKinematics & localKine,
                                       bool resetContactWrenches = true,
                                       bool resetCovariance = true);

  /// @brief Set the State Kinematics
  /// @details Sets a value for the kinematics part of the state
  ///
  /// @param localKine are the new kinematics of the state
  /// @param resetContactWrenches set if the contact wrenches should be reset
  /// @param resetCovariance set if the covariance of the state should be reset
  void setWorldCentroidStateKinematics(const Kinematics & kine, bool resetCovariance = true);

  /// @brief Set the state contact kinematics and wrench.
  /// @details Sets a value for the contact part of the state. Might be useful to reset this state for instance.
  ///
  /// @param index index of the contact
  /// @param worldContactRestPose new state rest pose of the contact
  /// @param wrench new state wrench of the contact
  /// @param resetCovariance set if the associated part of the state covariance matrix should be reset
  void setStateContact(Index index,
                       Kinematics worldContactRestPose,
                       const Vector6 & wrench,
                       bool resetCovariance = true);

  // TODO
  // void setVelocityGuess(const Kinematics)

  /// @brief Set the Gyro Bias
  /// Allows to initializa the value of the gyro bias of the IMU
  /// corresponding to the numberOfIMU
  ///
  /// @param numberOfIMU number id of the IMU
  /// @param resetCovariance set if the covariance of the IMU bias should be reset
  void setGyroBias(const Vector3 &, unsigned numberOfIMU = 1, bool resetCovariance = true);

  /// @brief Set the State Unmodeled Wrench
  /// @details this modifies the current guess for external unmodeled Wrench. This is different from
  /// setAdditionalWrench() since it modifies a state component. This function is likely useful when initializing the
  /// estimation and reduce the convergence time
  ///
  /// @param resetCovariance set if the covariance should be reset
  void setStateUnmodeledWrench(const Vector6 &, bool resetCovariance = true);

  /// @}

  // /////////////////////////////////////////////////////////////
  /// @name Estimator resets
  /// This allows to reset default values for specific parameters of the estimator
  // /////////////////////////////////////////////////////////////
  /// @{

  /// @brief Reset the default values for the sensors covariance matrices
  /// @details this is useful in case of misbehavior of the estimator or the sensors
  void resetSensorsDefaultCovMats();

  /// @brief  reset all the sensor inputs and provided contact information but keeps the contacts themselves
  void resetInputs();
  /// @}

  // /////////////////////////////////////////////////////////////
  /// @name Covariance matrices operations
  // /////////////////////////////////////////////////////////////
  /// @{

  /// @brief Set the Default value for Kinematics Init Covariance
  void setKinematicsInitCovarianceDefault(const Matrix &);
  /// @brief Set the Default value for Kinematics Init Covariance
  void setKinematicsInitCovarianceDefault(const Matrix3 & P_pos,
                                          const Matrix3 & P_ori,
                                          const Matrix3 & P_linVel,
                                          const Matrix3 & P_angVel);

  /// @brief Set the Default value for Gyro Bias Init Covariance
  void setGyroBiasInitCovarianceDefault(const Matrix3 & covMat);

  /// @brief Set the default value for init Unmodeled Wrench covariance matrix
  ///
  /// @param initCovMat
  void setUnmodeledWrenchInitCovMatDefault(const Matrix6 & initCovMat);

  /// @brief Set the default valut for the Initial Covariance Matrix of the contact in the state
  ///
  /// @param contactCovMat
  void setContactInitCovMatDefault(const Matrix12 & contactCovMat);

  /// @brief Set the Kinematics State Covariance
  void setKinematicsStateCovariance(const Matrix &);

  /// @brief Set the Gyro Bias State Covariance
  ///
  /// @param covMat the nwe covariance matrix
  /// @param imuNumber  the number id of the IMU
  void setGyroBiasStateCovariance(const Matrix3 & covMat, unsigned imuNumber);

  /// @brief Set the Unmodeled Wrench State Cov Mat
  ///
  /// @param newCovMat
  void setUnmodeledWrenchStateCovMat(const Matrix6 & newCovMat);

  /// @brief Set the Contact State Covariance Matrix
  ///
  /// @param contactNbr
  /// @param contactCovMat the contact number id
  void setContactStateCovMat(Index contactNbr, const Matrix12 & contactCovMat);

  /// @brief Set the default Kinematics Process Covariance
  void setKinematicsProcessCovarianceDefault(const Matrix12 &);

  /// @brief Set the default Kinematics Process Covariance
  void setKinematicsProcessCovarianceDefault(const Matrix3 & P_pos,
                                             const Matrix3 & P_ori,
                                             const Matrix3 & P_linVel,
                                             const Matrix3 & P_angVel);

  /// @brief Set the default Gyro Bias Process Covariance
  void setGyroBiasProcessCovarianceDefault(const Matrix3 & covMat);

  /// @brief Set the default Unmodeled Wrench Process Covariance
  void setUnmodeledWrenchProcessCovarianceDefault(const Matrix6 & covMat);

  /// @brief Set the default contact Process Covariance
  void setContactProcessCovarianceDefault(const Matrix12 & covMat);

  /// @brief Set the Kinematics Process Covariance
  void setKinematicsProcessCovariance(const Matrix12 &);

  /// @brief Set the Gyro Bias Process Covariance
  ///
  /// @param covMat the new process covariance matrix
  /// @param imuNumber the number id of the IMU
  void setGyroBiasProcessCovariance(const Matrix3 & covMat, unsigned imuNumber);

  /// @brief Set the Unmodeled Wrench Process Covariance Mattix
  ///
  /// @param processCovMat
  void setUnmodeledWrenchProcessCovMat(const Matrix6 & processCovMat);

  /// @brief Set the Contact Process Covariance Matrix
  ///
  /// @param contactNbr the contact number id
  /// @param contactCovMat the new process covariance matrix
  void setContactProcessCovMat(Index contactNbr, const Matrix12 & contactCovMat);

  /// @brief Set the Contact Process Covariance Matrix
  /// @details This version allows to set independently the part associated to each state variable of the contact.
  /// @param contactNbr id of the contact
  /// @param restPosProcessCov the new process covariance matrix for the rest position
  /// @param restOriProcessCov the new process covariance matrix for the rest orientation
  /// @param forceProcessCov the new process covariance matrix for the contact's force
  /// @param torqueProcessCov the new process covariance matrix for the contact's torque
  void setContactProcessCovMat(Index contactNbr,
                               const Matrix3 * restPosProcessCov = nullptr,
                               const Matrix3 * restOriProcessCov = nullptr,
                               const Matrix3 * forceProcessCov = nullptr,
                               const Matrix3 * torqueProcessCov = nullptr);

  /// Resets the covariance matrices to their original values
  void resetStateCovarianceMat();
  void resetStateKinematicsCovMat();
  void resetStateGyroBiasCovMat(Index i);
  void resetStateUnmodeledWrenchCovMat();
  void resetStateContactsCovMat();
  void resetStateContactCovMat(Index contactNbr);

  void resetProcessCovarianceMat();
  void resetProcessKinematicsCovMat();
  void resetProcessGyroBiasCovMat(Index i);
  void resetProcessUnmodeledWrenchCovMat();
  void resetProcessContactsCovMat();
  void resetProcessContactCovMat(Index contactNbr);
  /// @

  // /////////////////////////////////////////////////////////////
  /// @name State vector representation operations (advanced use)
  /// @details this is constituted with
  /// - Getters and setters for the state vector (from getStateSize() to setStateVector())
  /// - Getters for the indexes of the state Vector (from kineIndex() to contactWrenchIndex())
  /// - Getters for the indexes of the tangent state Vector (from kineIndexTangent() to contactWrenchIndextangent())
  // /////////////////////////////////////////////////////////////

  /// ////////////////////////////////
  /// Getters and setters for the state vector and full covariance matrices
  /// ////////////////////////////////

  /// @{
  /// @brief Get the State Vector Size.
  ///
  /// @return Index
  Index getStateSize() const;

  /// @{
  /// @brief Get the State Vector Tangent Size.
  ///
  /// @return Index
  Index getStateTangentSize() const;

  /// @brief Get the Measurement vector Size.
  ///
  /// @return Index
  Index getMeasurementSize() const;

  /// @brief Get the State Covariance matrix
  ///
  /// @return Matrix
  Matrix getStateCovarianceMat() const;

  /// @brief Set the State Covariance Matrix
  /// This is useful in case of a setting a guess on a whole state vect9or
  ///
  /// @param P The covariance matrix
  void setStateCovarianceMat(const Matrix & P);

  /// @brief Set the covariance matrices for the process noises
  ///
  /// @param Q process noise
  void setProcessNoiseCovarianceMat(const Matrix & Q);

  /// ////////////////////////////////
  /// Getters and setters for the state vectors
  /// ////////////////////////////////

  /// @brief Gets the current value of the state estimation in the form of a state vector \f$\hat{x_{k}}\f$
  ///
  /// @return const Vector&
  const Vector & getCurrentStateVector() const;

  /// @brief Get the State Vector Internal Time Index
  /// This is for advanced use but may be used to check how many states have been estimated up to now
  ///
  /// @return TimeIndex
  TimeIndex getStateVectorTimeIndex() const;

  /// @brief Initializes the state vector.
  /// @param initStateVector the initial state vector.
  void setInitWorldCentroidStateVector(const Vector & initStateVector);

  /// @brief Set a value of the state x_k provided from another source
  /// @details can be used for initialization of the estimator
  ///
  /// @param newvalue The new value for the state vector
  /// @param resetCovariance set if the state covariance should be reset
  void setStateVector(const Vector & newvalue, bool resetCovariance = true);

  /// @brief Get the Measurement Vector
  ///
  /// @return Vector
  Vector getMeasurementVector();

  /// ///////////////////////////////////////////////////////////
  ///  Getters for the indexes of the state Vector
  /// //////////////////////////////////////////////////////////

  /// @brief Get the kinematics index of the state vector
  ///
  /// @return unsigned
  inline Index kineIndex() const;

  /// @brief Get the position index of the state vector
  ///
  /// @return Index
  inline Index posIndex() const;

  /// @brief Get the orientation index of the state vector
  ///
  /// @return Index
  inline Index oriIndex() const;

  /// @brief Get the linear velocity index of the state vector
  ///
  /// @return Index
  inline Index linVelIndex() const;

  /// @brief Get the angular velocity index of the state vector
  ///
  /// @return Index
  inline Index angVelIndex() const;

  /// @brief Get the gyro bias index of the state vector
  ///
  /// @return Index
  inline Index gyroBiasIndex(Index IMUNumber) const;

  /// @brief Get the unmodeled external wrench index of the state vector
  ///
  /// @return Index
  inline Index unmodeledWrenchIndex() const;

  /// @brief Get the unmodeled external linear force  index of the state vector
  ///
  /// @return Index
  inline Index unmodeledForceIndex() const;

  /// @brief Get the unmodeled external torque force  index of the state vector
  ///
  /// @return Index
  inline Index unmodeledTorqueIndex() const;

  /// @brief Get the index for the contact segment in the state vector
  ///
  /// @return Index
  inline Index contactsIndex() const;

  /// @brief Get the index of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactIndex(Index contactNbr) const;

  /// @brief Get the index of the kinematics of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactKineIndex(Index contactNbr) const;

  /// @brief Get the index of the position of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactPosIndex(Index contactNbr) const;

  /// @brief Get the index of the orientation of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactOriIndex(Index contactNbr) const;

  /// @brief Get the index of the linear force of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactForceIndex(Index contactNbr) const;

  /// @brief Get the index of the toraue of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactTorqueIndex(Index contactNbr) const;

  /// @brief Get the index of the wrench of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactWrenchIndex(Index contactNbr) const;

  // ///////////////////////////////////////////////////////////
  /// Getters for the indexes of the tangent state Vector
  /// //////////////////////////////////////////////////////////

  /// @brief Get the kinematics index of the tangent state vector
  ///
  /// @return Index
  inline Index kineIndexTangent() const;

  /// @brief Get the position index of the tangent state vector
  ///
  /// @return Index
  inline Index posIndexTangent() const;

  /// @brief Get the orientation index of the tangent state vector
  ///
  /// @return Index
  inline Index oriIndexTangent() const;

  /// @brief Get the linear velocity index of the tangent state vector
  ///
  /// @return Index
  inline Index linVelIndexTangent() const;

  /// @brief Get the angular velocity index of the tangent state vector
  ///
  /// @return Index
  inline Index angVelIndexTangent() const;

  /// @brief Get the gyro bias index of the tangent state vector
  ///
  /// @return Index
  inline Index gyroBiasIndexTangent(Index IMUNumber) const;

  /// @brief Get the unmodeled external wrench index of the tangent state vector
  ///
  /// @return Index
  inline Index unmodeledWrenchIndexTangent() const;

  /// @brief Get the unmodeled external linear force index of the tangent state vector
  ///
  /// @return Index
  inline Index unmodeledForceIndexTangent() const;

  /// @brief Get the unmodeled external torque force  index of the tangent state vector
  ///
  /// @return Index
  inline Index unmodeledTorqueIndexTangent() const;

  /// @brief Get the index for the contact segment in the tangent state vector
  ///
  /// @return Index
  inline Index contactsIndexTangent() const;

  /// @brief Get the index of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactIndexTangent(Index contactNbr) const;

  /// @brief Get the index of the kinematics of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactKineIndexTangent(Index contactNbr) const;

  /// @brief Get the index of the position of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactPosIndexTangent(Index contactNbr) const;

  /// @brief Get the index of the orientation of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactOriIndexTangent(Index contactNbr) const;

  /// @brief Get the index of the linear force of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactForceIndexTangent(Index contactNbr) const;

  /// @brief Get the index of the toraue of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactTorqueIndexTangent(Index contactNbr) const;

  /// @brief Get the index of the wrench of a specific contact in the sate tangent vector
  ///
  /// @param contactNbr The contact number id
  /// @return Index
  inline Index contactWrenchIndexTangent(Index contactNbr) const;

  /// @}

  // /////////////////////////////////////////////////////////////
  /// @name Getters for the extended Kalman filter (advanced use)
  // /////////////////////////////////////////////////////////////
  /// @{

  /// Gets a const reference on the extended Kalman filter
  const ExtendedKalmanFilter & getEKF() const;

  /// Gets a reference on the extended Kalman filter
  /// modifying this object may lead to instabilities
  ExtendedKalmanFilter & getEKF();
  /// @}

protected:
  struct Input
  {
    Input(unsigned maxContacts, unsigned maxNumberOfIMU) : contacts_(maxContacts), imuSensors_(maxNumberOfIMU) {}
    struct Sensor
    {
      Sensor(Index signalSize) : measIndex(-1), measIndexTangent(-1), size(signalSize), time(0) {}
      ~Sensor() {}
      Index measIndex;
      Index measIndexTangent;
      Index size;
      TimeIndex time;

      inline Vector extractFromVector(const Vector & v)
      {
        return v.segment(size, measIndex);
      }
    };

    struct IMU : public Sensor
    {
      ~IMU() {}
      IMU() : Sensor(sizeIMUSignal) {}

      Kinematics userImuKinematics; // the kinematics of the IMU in the user's frame
      LocalKinematics centroidImuKinematics; // the kinematics of the IMU in the IMU's frame
      Vector6 acceleroGyro;
      Matrix3 covMatrixAccelero;
      Matrix3 covMatrixGyro;

      Index stateIndex;
      Index stateIndexTangent;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::vector<IMU, Eigen::aligned_allocator<IMU>> VectorIMU;
    typedef VectorIMU::iterator VectorIMUIterator;
    typedef VectorIMU::const_iterator VectorIMUConstIterator;

    struct Contact : public Sensor
    {
      Contact() : Sensor(sizeWrench), isSet(false), withRealSensor(false), stateIndex(-1), stateIndexTangent(-1)
      {
        worldRestPose.angVel = worldRestPose.linVel = Vector3::Zero();
      }
      ~Contact() {}

      /// State ///
      Kinematics worldRestPose; // the rest pose of the contact in the world frame

      /// Measurements ///
      Vector6
          wrenchMeasurement; /// Describes the measured wrench (forces + torques) at the contact in the sensor's frame

      /// Input ///
      Kinematics userContactKine; /// Describes the kinematics of the contact point in the centroid's frame.
      Kinematics centroidContactKine; /// Describes the kinematics of the contact point in the centroid's frame.
      CheckedMatrix6 sensorCovMatrix; /// measurement covariance matrix of the wrench sensor attached to the contact.

      // process covariance associated to the rest position of the contact.
      Matrix3 restPosProcessCovMat;
      // process covariance associated to the rest orientation of the contact.
      Matrix3 restOriProcessCovMat;

      Matrix3 linearStiffness; /// linear stiffness associated to the contact, used in the visco-elastic model
      Matrix3 linearDamping; /// linear damping associated to the contact, used in the visco-elastic model
      Matrix3 angularStiffness; /// angular stiffness associated to the contact, used in the visco-elastic model
      Matrix3 angularDamping; /// angular damping associated to the contact, used in the visco-elastic model

      /// Status ///

      bool isSet;
      bool withRealSensor;
      Index stateIndex;
      Index stateIndexTangent;

      static const Kinematics::Flags::Byte contactKineFlags = /// flags for the components of the kinematics
          Kinematics::Flags::position | Kinematics::Flags::orientation | Kinematics::Flags::linVel
          | Kinematics::Flags::angVel;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::vector<Contact, Eigen::aligned_allocator<Contact>> VectorContact;
    typedef VectorContact::iterator VectorContactIterator;
    typedef VectorContact::const_iterator VectorContactConstIterator;

    struct AbsolutePoseSensor : public Sensor
    {
      AbsolutePoseSensor() : Sensor(sizePose) {}

      Kinematics pose;
      static const Kinematics::Flags::Byte poseFlags = Kinematics::Flags::position | Kinematics::Flags::orientation;
      CheckedMatrix6 covMatrix;
    };

    struct AbsoluteOriSensor : public Sensor
    {
      AbsoluteOriSensor() : Sensor(sizePose) {}

      Orientation ori;
      CheckedMatrix3 covMatrix;
    };

  public:
    AbsolutePoseSensor absPoseSensor_;
    AbsoluteOriSensor absOriSensor_;
    VectorContact contacts_;
    VectorIMU imuSensors_;
  };

protected:
  ///////////// DYNAMICAL SYSTEM IMPLEMENTATION
  /// @brief Applies the state-transition model to the given state vector using the given input to predict the future
  /// state.
  /// @param x The current state vector
  /// @param u The current input
  /// @param k The current time index
  /// @return Vector&
  virtual Vector stateDynamics(const Vector & x, const InputBase & u, TimeIndex k);

  /// @brief Applies the measurement model to the given state vector using the given input to predict the sensor
  /// measurements.
  /// @param x The current state vector
  /// @param u The current input
  /// @param k The current time index
  /// @return Vector&
  virtual Vector measureDynamics(const Vector & x, const InputBase & u, TimeIndex k);

  /// @brief Adds the unmodeled and contact wrenches from the state to the given wrench.
  /// @param centroidStateVector The current state vector
  /// @param force The force we want to add the forces to. Must be expressed in the centroid frame.
  /// @param torque The torque we want to add the torques to. Must be expressed in the centroid frame.
  void addUnmodeledAndContactWrench_(const Vector & centroidStateVector, Vector3 & force, Vector3 & torque);

  /// @brief Adds the unmodeled wrench from the state to the given wrench.
  /// @param centroidStateVector The current state vector
  /// @param force The force we want to add the unmodeled force to. Must be expressed in the centroid frame.
  /// @param torque The torque we want to add the unmodeled torque to. Must be expressed in the centroid frame.
  void addUnmodeledWrench_(const Vector & centroidStateVector, Vector3 & force, Vector3 & torque);

  /// @brief adds the contribution of a contact wrench at the centroid to the total wrench
  ///
  /// @param centroidContactKine The Kinematics of the current contact at the centroid
  /// @param centroidContactForce The contact force at the centroid to add to the total force
  /// @param centroidContactTorque The contact torque at the centroid to add to the total torque
  /// @param totalCentroidForce The total force exerced at the centroid, that will be completed with the contact's force
  /// @param totalCentroidTorque The total torque exerced at the centroid, that will be completed with the contact's
  /// torque
  void addContactWrench_(const Kinematics & centroidContactKine,
                         const Vector3 & centroidContactForce,
                         const Vector3 & centroidContactTorque,
                         Vector3 & totalCentroidForce,
                         Vector3 & totalCentroidTorque);

  /// @brief Computes the local accelerations of the centroid frame in the world frame and adds them to its local
  /// kinematics.
  ///
  /// @param localStateKine The local kinematics of the centroid frame in the world frame that still don't contain the
  /// accelerations.
  /// @param centroidContactForce The contact force at the centroid to add to the total force
  /// @param totalForceLocal Total force exerted on the centroid.
  /// @param totalMomentLocal Total torque exerted on the centroid.
  /// @param linAcc The empty vector of the linear acceleration we want to compute.
  /// @param angAcc The empty vector of the angular acceleration we want to compute.

  void computeLocalAccelerations_(LocalKinematics & localStateKine,
                                  const Vector3 & totalForceLocal,
                                  const Vector3 & totalMomentLocal,
                                  Vector3 & linAcc,
                                  Vector3 & angAcc);

  /// @brief Computes the force exerted at a contact using the visco-elastic model on the given state vector.
  /// @param i Contact to estimate
  /// @param worldCentroidStateKinematics State vector used in the visco-elastic model
  /// @param worldRestContactPose Rest pose of the contact
  /// @param contactForce Empty vector of the contact force to estimate
  /// @param contactTorque Empty vector of the contact force to estimate
  void computeContactWrench_(const Input::Contact & contact,
                             Kinematics & worldCentroidStateKinematics,
                             Kinematics & worldRestContactPose,
                             Vector6 & contactWrench);

  /// @brief @copybrief computeContactForce_(VectorContactIterator i, LocalKinematics & worldCentroidStateKinematics,
  /// Kinematics & worldRestContactPose, Vector3 & contactForce, Vector3 & contactTorque). Compute the resulting wrench
  /// for all currently set contacts.
  /// @param worldCentroidStateKinematics State vector used in the visco-elastic model
  /// @param contactForce Empty vector of the contact force to estimate
  /// @param contactTorque Empty vector of the contact force to estimate
  void computeContactForces_(LocalKinematics & worldCentroidStateKinematics,
                             Vector3 & contactForce,
                             Vector3 & contactTorque);

  /// Sets a noise which disturbs the state dynamics
  virtual void setProcessNoise(NoiseBase *);

  /// Removes the process noise
  virtual void resetProcessNoise();
  /// Gets the process noise
  virtual NoiseBase * getProcessNoise() const;

  /// Sets a noise which disturbs the measurements
  virtual void setMeasurementNoise(NoiseBase *);
  /// Removes the measurement noise
  virtual void resetMeasurementNoise();
  /// Gets a pointer on the measurement noise
  virtual NoiseBase * getMeasurementNoise() const;

  /// Gets the input size
  virtual Index getInputSize() const;

public:
  /// @{
  /// @brief Returns the wrench exerted at the contact, expressed in the frame of the centroid
  /// @return Vector6
  Vector6 getCentroidContactWrench(Index numContact) const;

  /// @brief Returns the kinematics of the contact in the centroid frame, given as an input when updating the contact
  /// (obtained from its kinematics in the user frame).
  /// @return Kinematics
  Kinematics getCentroidContactInputKine(Index numContact) const;

  /// @brief Returns the kinematics of the contact in the world frame, obtained from the state kinematics of the
  /// centroid in the world frame.
  /// @return Kinematics
  Kinematics getWorldContactKineFromCentroid(Index numContact) const;

  /// @brief Returns the estimated rest kinematics of the contact in the world frame.
  /// @return Kinematics
  Kinematics getContactStateRestKinematics(Index numContact) const;

  /// @brief Returns the kinematics of the contact in the user frame, given as an input when updating the contact.
  /// @return Kinematics
  Kinematics getUserContactInputKine(Index numContact) const;

  /// @brief Returns the wrench predicted by the visco-elastic model from the currently estimated state
  /// @return The predicted wrench
  Vector6 getCurrentViscoElasticWrench(Index numContact);

  /// @brief Get the measurement index of the required IMU : allows to access its corresponding measurements in the
  /// measurement vector for example
  ///
  /// @return Index
  Index getIMUMeasIndexByNum(Index num) const;

  Index getContactMeasIndexByNum(Index num) const;

  bool getContactIsSetByNum(Index num) const;

  ///////////////////////////////////////////////////////////////
  /// @name State vector representation arithmetics and derivation (advanced use)
  ///////////////////////////////////////////////////////////////

  /// @{

  /// @brief the sum operator for the state vector
  /// @details it amounts at a time-integration of the state vector using a tangent vector (constant for 1 second)
  ///
  /// @param stateVector The state vector
  /// @param tangentVector The tangent Vector
  inline Vector stateSum(const Vector & stateVector, const Vector & tangentVector);

  /// @brief @copybrief stateSum(const Vector&,const Vector&). This version does not allocate a new vector
  /// @copydetails stateSum(const Vector&,const Vector&)
  /// @param sum The result of the operation
  virtual void stateSum(const Vector & stateVector, const Vector & tangentVector, Vector & sum);

  /// @brief the difference operator for the state statevector1 ⊖ statevector2
  /// @details it amounts at a time derivation betweeen state vectors with a time step of one second. Therefore the
  /// result is a tangent vector
  ///
  /// @param stateVector1 Operator 1
  /// @param stateVector2 Operator 2
  inline Vector stateDifference(const Vector & stateVector1, const Vector & stateVector2);

  /// @brief @copybrief stateDifference(const Vector &, const Vector &) This version prevents a nwe vector allocation
  /// @copydetails stateDifference(const Vector&,const Vector&)
  /// @param difference The result
  virtual void stateDifference(const Vector & stateVector1, const Vector & stateVector2, Vector & difference);

  /// @brief the difference operator for the measurement statevector1 ⊖ statevector2
  /// @details it amounts at a time derivation betweeen state vectors with a time step of one second. Therefore the
  /// result is a tangent vector
  ///
  /// @param measureVector1 Operator 1
  /// @param measureVector2 Operator 2
  /// @param difference The result
  virtual void measurementDifference(const Vector & measureVector1, const Vector & measureVector2, Vector & difference);

  /// @brief Define if we use dinite differences Jacobian or analytic
  ///
  /// @param b true means we use finite differences
  virtual void useFiniteDifferencesJacobians(bool b = true);

  /// @brief Set the Finite Difference time step
  ///
  /// @param dx the timestep
  virtual void setFiniteDifferenceStep(const Vector & dx);

  virtual Matrix computeAMatrix();

  virtual Matrix computeCMatrix();

  /// @brief computes the local acceleration from the given state vector
  void computeLocalAccelerations(const Vector & x, Vector & acceleration);

  /// @brief Comparison between the Jacobians of the linear and angular accelerations with respect to the state,
  /// obtained with finite differences and analyticially.
  friend int testAccelerationsJacobians(KineticsObserver & ko,
                                        int errcode,
                                        double relativeErrorThreshold,
                                        double threshold); // declared out of namespace state-observation

  /// @brief Comparison between the analytical Jacobian matrix A and the one obtained by finite differences. Used to
  /// test the analytical method.
  /// @param threshold Threshold on the relative error between both Jacobians (in percentage)
  friend int testAnalyticalAJacobianVsFD(KineticsObserver & ko,
                                         int errcode,
                                         double relativeErrorThreshold,
                                         double threshold); // declared out of namespace state-observation

  friend int testAnalyticalCJacobianVsFD(KineticsObserver & ko,
                                         int errcode,
                                         double relativeErrorThreshold,
                                         double threshold); // declared out of namespace state-observation

  /// @brief Comparison between the Jacobians of orientation integration with respect to an increment vector delta,
  /// obtained with finite differences and analyticially.
  friend int testOrientationsJacobians(KineticsObserver & ko,
                                       int errcode,
                                       double relativeErrorThreshold,
                                       double threshold); // declared out of namespace state-observation
  /// @}

protected:
  void stateNaNCorrection_();

  /// @brief update of the state kinematics worldCentroidStateKinematics_ and of the contacts pose with the newly
  /// estimated state
  void updateLocalKineAndContacts_();

  /// updates the global kinematics of the centroid from the local ones, that can be more interpretable
  void updateGlobalKine_();

protected:
  unsigned maxContacts_;
  unsigned maxImuNumber_;

  Input input_;

  std::set<Index> removedContacts_;

  Index stateSize_;
  Index stateTangentSize_;
  Index measurementSize_;
  Index measurementTangentSize_;

  Vector worldCentroidStateVector_;
  Vector worldCentroidStateVectorDx_;
  Vector oldWorldCentroidStateVector_;

  LocalKinematics worldCentroidStateKinematics_;
  Kinematics worldCentroidKinematics_;

  Vector3 additionalForce_;
  Vector3 additionalTorque_;

  Vector measurementVector_;
  Matrix measurementCovMatrix_;

  stateObservation::ExtendedKalmanFilter ekf_;
  bool finiteDifferencesJacobians_;
  bool withGyroBias_;
  bool withUnmodeledWrench_;
  bool withAccelerationEstimation_;
  bool withDampingInMatrixA_;
  bool withAdaptativeContactProcessCov_;

  IndexedVector3 com_, comd_, comdd_;
  IndexedVector3 sigma_, sigmad_;
  IndexedMatrix3 I_, Id_;

  TimeIndex k_est_; // time index of the last estimation
  TimeIndex k_data_; // time index of the current measurements

  double mass_;

  double dt_;

  NoiseBase * processNoise_;
  NoiseBase * measurementNoise_;

  Index numberOfContactRealSensors_;
  Index currentIMUSensorNumber_;

  // indicates if a contact has been added or removed since the last iteration
  // bool contactsChanged_;
  unsigned nb_prevContacts_;
  // indicates if a contact's process covariance on the rest position has been modified since the last iteration
  bool contactRestPosProcessChanged_;
  // indicates if a contact's process covariance on the rest orientation has been modified since the last iteration
  bool contactRestOriProcessChanged_;

  /// function to call before adding any measurement
  /// detects if there is a new estimation beginning and then
  /// calls the reset of the iteration
  void startNewIteration_();

  /// @brief Function that is called at the end of an iteration
  /// @details Can be used to reinitialize variables that might be used before startNewIteration_ on the next iteration
  /// (ex: when adding or removing contacts.)
  void endIteration_();

  /// @brief Converts a LocalKinematics object from the user's frame to the centroid's frame, which is used for most of
  /// the computations
  /// @param userKine the LocalKinematics object expressed in the user's frame. It is likely to correspond to the IMU's
  /// LocalKinematics, defined by the user in its frame.
  /// @param centroidKine the LocalKinematics object corresponding to the converted LocalKinematics in the centroid's
  /// frame.

  void convertUserToCentroidFrame_(const Kinematics & userKine, Kinematics & centroidKine, TimeIndex k_data);

  /// @brief Converts a Kinematics object from the user's frame to the centroid's frame, which is used for most of the
  /// computations
  /// @param userKine the Kinematics object expressed in the user's frame. It is likely to correspond to the contact's
  /// Kinematics, defined by the user in its frame.

  Kinematics convertUserToCentroidFrame_(const Kinematics & userKine, TimeIndex k_data);

  /// Getters for the indexes of the state Vector using private types

  inline Index gyroBiasIndex(Input::VectorIMUConstIterator i) const;
  inline Index gyroBiasIndexTangent(Input::VectorIMUConstIterator i) const;

  inline Index contactIndex(Input::VectorContactConstIterator i) const;
  inline Index contactKineIndex(Input::VectorContactConstIterator i) const;
  inline Index contactPosIndex(Input::VectorContactConstIterator i) const;
  inline Index contactOriIndex(Input::VectorContactConstIterator i) const;
  inline Index contactForceIndex(Input::VectorContactConstIterator i) const;
  inline Index contactTorqueIndex(Input::VectorContactConstIterator i) const;
  inline Index contactWrenchIndex(Input::VectorContactConstIterator i) const;

  /// Getters for the indexes of the state Vector using private types
  inline Index contactIndexTangent(Input::VectorContactConstIterator i) const;
  inline Index contactKineIndexTangent(Input::VectorContactConstIterator i) const;
  inline Index contactPosIndexTangent(Input::VectorContactConstIterator i) const;
  inline Index contactOriIndexTangent(Input::VectorContactConstIterator i) const;
  inline Index contactForceIndexTangent(Input::VectorContactConstIterator i) const;
  inline Index contactTorqueIndexTangent(Input::VectorContactConstIterator i) const;
  inline Index contactWrenchIndexTangent(Input::VectorContactConstIterator i) const;

public: ///////////SIZE OF VECTORS
  inline static constexpr Index sizeAcceleroSignal = 3;
  inline static constexpr Index sizeGyroSignal = 3;
  inline static constexpr Index sizeIMUSignal = sizeAcceleroSignal + sizeGyroSignal;

  inline static constexpr Index sizePos = 3;
  inline static constexpr Index sizePosTangent = 3;
  inline static constexpr Index sizeOri = 4;
  inline static constexpr Index sizeOriTangent = 3;
  inline static constexpr Index sizeLinVel = sizePos;
  inline static constexpr Index sizeLinVelTangent = sizeLinVel;
  inline static constexpr Index sizeLinAccTangent = sizeLinVelTangent;
  inline static constexpr Index sizeAngVel = sizeOriTangent;
  inline static constexpr Index sizeAngVelTangent = sizeAngVel;
  inline static constexpr Index sizeGyroBias = sizeGyroSignal;
  inline static constexpr Index sizeGyroBiasTangent = sizeGyroBias;

  inline static constexpr Index sizeForce = 3;
  inline static constexpr Index sizeForceTangent = sizeForce;
  inline static constexpr Index sizeTorque = 3;
  inline static constexpr Index sizeTorqueTangent = sizeTorque;

  inline static constexpr Index sizeWrench = sizeForce + sizeTorque;

  inline static constexpr Index sizeStateKine = sizePos + sizeOri + sizeLinVel + sizeAngVel;
  inline static constexpr Index sizeStateBase = sizeStateKine + sizeForce + sizeTorque;
  inline static constexpr Index sizeStateKineTangent = sizePos + sizeOriTangent + sizeLinVel + sizeAngVel;
  inline static constexpr Index sizeStateTangentBase = sizeStateKineTangent + sizeForce + sizeTorque;

  inline static constexpr Index sizePose = sizePos + sizeOri;
  inline static constexpr Index sizePoseTangent = sizePos + sizeOriTangent;

  inline static constexpr Index sizeContactKine = sizePose;
  inline static constexpr Index sizeContactKineTangent = sizePoseTangent;

  inline static constexpr Index sizeContact = sizeContactKine + sizeWrench;
  inline static constexpr Index sizeContactTangent = sizeContactKineTangent + sizeWrench;

  inline static constexpr Kinematics::Flags::Byte flagsStateKine =
      Kinematics::Flags::position | Kinematics::Flags::orientation | Kinematics::Flags::linVel
      | Kinematics::Flags::angVel;

  inline static constexpr Kinematics::Flags::Byte flagsContactKine =
      Kinematics::Flags::position | Kinematics::Flags::orientation;

  inline static constexpr Kinematics::Flags::Byte flagsPoseKine =
      Kinematics::Flags::position | Kinematics::Flags::orientation;

  inline static constexpr Kinematics::Flags::Byte flagsPosKine = Kinematics::Flags::position;

  inline static constexpr Kinematics::Flags::Byte flagsIMUKine =
      Kinematics::Flags::position | Kinematics::Flags::orientation | Kinematics::Flags::linVel
      | Kinematics::Flags::angVel | Kinematics::Flags::linAcc | Kinematics::Flags::angAcc;

  ////////////DEFAULT VALUES //////
  static const double defaultMass;

  static const double statePoseInitVarianceDefault;
  static const double stateOriInitVarianceDefault;
  static const double stateLinVelInitVarianceDefault;
  static const double stateAngVelInitVarianceDefault;
  static const double gyroBiasInitVarianceDefault;
  static const double unmodeledWrenchInitVarianceDefault;
  static const double contactForceInitVarianceDefault;
  static const double contactTorqueInitVarianceDefault;

  static const double statePoseProcessVarianceDefault;
  static const double stateOriProcessVarianceDefault;
  static const double stateLinVelProcessVarianceDefault;
  static const double stateAngVelProcessVarianceDefault;
  static const double gyroBiasProcessVarianceDefault;
  static const double unmodeledWrenchProcessVarianceDefault;
  static const double contactPositionProcessVarianceDefault;
  static const double contactOrientationProcessVarianceDefault;
  static const double contactForceProcessVarianceDefault;
  static const double contactTorqueProcessVarianceDefault;

  static const double acceleroVarianceDefault;
  static const double gyroVarianceDefault;
  static const double forceSensorVarianceDefault;
  static const double torqueSensorVarianceDefault;
  static const double positionSensorVarianceDefault;
  static const double orientationSensorVarianceDefault;

  static const double linearStiffnessDefault;
  static const double angularStiffnessDefault;
  static const double linearDampingDefault;
  static const double angularDampingDefault;

  ////////////

  bool nanDetected_ = false;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  /// Default Stiffness and damping
  Matrix3 linearStiffnessMatDefault_;
  Matrix3 angularStiffnessMatDefault_;
  Matrix3 linearDampingMatDefault_;
  Matrix3 angularDampingMatDefault_;

  ////////////Sensor Covariance mnatrices
  Matrix3 acceleroCovMatDefault_;
  Matrix3 gyroCovMatDefault_;
  Matrix6 contactWrenchSensorCovMatDefault_;
  Matrix6 absPoseSensorCovMatDefault_;
  Matrix3 absOriSensorCovMatDefault_;

  Matrix3 statePosInitCovMat_;
  Matrix3 stateOriInitCovMat_;
  Matrix3 stateLinVelInitCovMat_;
  Matrix3 stateAngVelInitCovMat_;
  Matrix3 gyroBiasInitCovMat_;
  Matrix6 unmodeledWrenchInitCovMat_;
  Matrix12 contactInitCovMatDefault_;

  Matrix3 statePosProcessCovMat_;
  Matrix3 stateOriProcessCovMat_;
  Matrix3 stateLinVelProcessCovMat_;
  Matrix3 stateAngVelProcessCovMat_;
  Matrix3 gyroBiasProcessCovMat_;
  Matrix6 unmodeledWrenchProcessCovMat_;
  Matrix3 contactPositionProcessCovMat_;
  Matrix3 contactOrientationProcessCovMat_;
  Matrix3 contactForceProcessCovMat_;
  Matrix3 contactTorqueProcessCovMat_;
  Matrix12 contactProcessCovMatDefault_;

  Matrix12 stateKinematicsInitCovMat_;
  Matrix12 stateKinematicsProcessCovMat_;

  // contains the M matrices for each possible number of contacts. The M matrix allows to associate a process covariance
  // to the contact rest position while ensuring that their average position is associated with a zero covariance. In a
  // simpler way, it allows the contacts rest position to move slightly over time, but we don't allow their average to
  // move, thus preventing drift.
  std::vector<Eigen::MatrixXd> m_matrices_;
  // similar to m_matrices, but for the process on the rest orientation of the contacts.
  std::vector<Eigen::MatrixXd> m_prime_matrices_;

  /// default derivation steps
  static const double defaultdx;

  /// a structure to optimize computations
  struct Opt
  {
    Opt() : locKine(locKine1), ori(locKine.orientation), ori1(locKine1.orientation), ori2(locKine2.orientation) {}

    LocalKinematics locKine1, locKine2;
    LocalKinematics & locKine;
    Orientation & ori;
    Orientation & ori1;
    Orientation & ori2;
  } opt_;

private:
  Index setIMU(const Vector3 & accelero,
               const Vector3 & gyrometer,
               const Kinematics & userImuKinematics,
               Index num,
               const Matrix3 * acceleroCov,
               const Matrix3 * gyroCov);
};

#include <state-observation/dynamics-estimators/kinetics-observer.hxx>

} // namespace stateObservation

#endif /// KINETICSOBSERVER_HPP
