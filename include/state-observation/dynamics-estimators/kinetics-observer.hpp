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

#include <map>
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

/// @brief This observer estimated the kinematics and the external forces.

/// @details  The provided kinematics is the position, the orientation, the velocities and even the accelerations of a
/// frame 1 within another frame 2. The object Kinematics is the expression of these kinematics in the global frame 2,
/// while the LocalKinematics object is their expression in the local frame 1. Our observer estimates the local
/// kinematics of the centroid's frame within the world frame. The reason to choose the centroid's frame is that it
/// simplifies many expressions, for example the expressions of the accelerations. This estimation is based on the
/// assumption of viscoelastic contacts and using three kinds of measurements: IMUs, Force/Torque measurements (contact
/// and other ones) and any absolute position measurements.
///
class STATE_OBSERVATION_DLLAPI KineticsObserver : protected DynamicalSystemFunctorBase,
                                                  protected kine::Kinematics::RecursiveAccelerationFunctorBase,
                                                  protected kine::LocalKinematics::RecursiveAccelerationFunctorBase,
                                                  protected StateVectorArithmetics
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

  bool getWithAccelerationEstimation() const;

  /// @brief Set if the gyrometers bias is computed or not.
  ///        This parameter is global for all the IMUs.
  ///
  /// @param b
  void setWithGyroBias(bool b = true);

  /// @brief Set the total mass of the robot. This can be changed online
  ///
  /// @return sets
  void setMass(double);

  const double & getMass() const;

  const IndexedMatrix3 & getInertiaMatrix() const;

  const IndexedMatrix3 & getInertiaMatrixDot() const;

  const IndexedVector3 & getAngularMomentum() const;

  const IndexedVector3 & getAngularMomentumDot() const;

  const IndexedVector3 & getCenterOfMass() const;

  const IndexedVector3 & getCenterOfMassDot() const;

  const IndexedVector3 & getCenterOfMassDotDot() const;

  const Vector6 getAdditionalWrench() const;

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
  /// @param centroidImuKinematics sets the kinematics of the IMU in the user's
  /// frame, expressed in the user's frame. The best is to provide the position, the orientation,
  /// the angular and linear velocities and the linear acceleration
  /// Nevertheless if velocities or accelerations are not available they will be
  /// automatically computed through finite differences
  /// @param num the number of the IMU (useful in case there are several ones).
  ///           If not set it will be generated automatically.
  int setIMU(const Vector3 & accelero, const Vector3 & gyrometer, const Kinematics & userImuKinematics, int num = -1);

  /// @brief @copybrief setIMU(const Vector3&,const Vector3&,const Kinematics &,int)
  /// Provides also the associated covariance matrices
  /// @details
  /// This version specifies the covariance matrices of these measurements.
  /// @copydetails setIMU(const Vector3&,const Vector3&,const Kinematics &,int)
  /// @param acceleroCov The covariance matrix of the accelerometer
  /// @param gyroCov The covariance matrix of the gyrometer
  int setIMU(const Vector3 & accelero,
             const Vector3 & gyrometer,
             const Matrix3 & acceleroCov,
             const Matrix3 & gyroCov,
             const Kinematics & userImuKinematics,
             int num = -1);

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
  int addContact(const Kinematics & pose,
                 const Matrix12 & initialCovarianceMatrix,
                 const Matrix12 & processCovarianceMatrix,
                 int contactNumber = -1,
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
  int addContact(const Kinematics & pose,
                 int contactNumber = -1,
                 const Matrix3 & linearStiffness = Matrix3::Constant(-1),
                 const Matrix3 & linearDamping = Matrix3::Constant(-1),
                 const Matrix3 & angularStiffness = Matrix3::Constant(-1),
                 const Matrix3 & angularDamping = Matrix3::Constant(-1));

  /// @brief Remove a contact
  ///
  /// @param contactnbr the number of the contact to remove
  void removeContact(int contactnbr);

  /// @brief remove all the contacts
  void clearContacts();

  /// @brief Get the Number Of Contacts
  ///
  /// @return Index The number of contacts
  Index getNumberOfContacts() const;

  /// @brief Get the Current Number Of Contacts
  ///
  /// @return Index The current number of contacts
  Index getNumberOfSetContacts() const;

  /// @brief Get the List Of Contact ids
  ///
  /// @return std::vector<int> a vector listing the contact ids
  std::vector<int> getListOfContacts() const;

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

  void setInitWorldCentroidStateVector(const Vector & initStateVector);

  void setAllCovariances(const Matrix3 & statePositionInitCovariance,
                         const Matrix3 & stateOriInitCovariance,
                         const Matrix3 & stateLinVelInitCovariance,
                         const Matrix3 & stateAngVelInitCovariance,
                         const Matrix3 & gyroBiasInitCovariance,
                         const Matrix6 & unmodeledWrenchInitCovariance,
                         const Matrix12 & contactInitCovariance,
                         const Matrix3 & statePositionProcessCovariance,
                         const Matrix3 & stateOriProcessCovariance,
                         const Matrix3 & stateLinVelProcessCovariance,
                         const Matrix3 & stateAngVelProcessCovariance,
                         const Matrix3 & gyroBiasProcessCovariance,
                         const Matrix6 & unmodeledWrenchProcessCovariance,
                         const Matrix12 & contactProcessCovariance,
                         const Matrix3 & positionSensorCovariance,
                         const Matrix3 & orientationSensorCoVariance,
                         const Matrix3 & acceleroSensorCovariance,
                         const Matrix3 & gyroSensorCovariance,
                         const Matrix6 & contactSensorCovariance);
  /// @}

  // /////////////////////////////////////////////
  /// @name Setting additional inputs to the dynamical system
  /// @details It is highly recommended to set these inputs at each iteration
  /// ////////////////////////////////////////////////

  /// @{
  /// @brief Set the Center Of Mass kinematics expressed in the local estimated frame
  ///
  /// @param com position
  /// @param com_dot velocity
  /// @param com_dot_dot acceleration
  void setCenterOfMass(const Vector3 & com, const Vector3 & com_dot, const Vector3 & com_dot_dot);

  /// @brief Set the Center Of Mass kinematics expressed in the local estimated frame
  /// @details The acceleration will be computed through finite differences
  ///
  /// @param com position
  /// @param com_dot velocity
  void setCenterOfMass(const Vector3 & com, const Vector3 & com_dot);

  /// @brief Set the Center Of Mass kinematics expressed in the local estimated frame
  /// @details The velocity and acceleration will be computed through finite differences
  ///
  /// @param com position
  void setCenterOfMass(const Vector3 & com);

  /// @brief Set the 3x3 inertia matrix and its derivative expressed in the local frame
  ///
  /// @param I Set the inertia matrix at the CoM
  /// @param I_dot Derivative of inertia matrix
  void setCoMInertiaMatrix(const Matrix3 & I, const Matrix3 & I_dot);

  /// @brief Set the 3x3 inertia matrix expressed in the local frame
  /// @details The derivative will be computed using finite differences
  ///
  /// @param I Inertia matrix
  /// @param I_dot Derivative of inertia matrix
  void setCoMInertiaMatrix(const Matrix3 & I);

  /// @brief Set the inertia matrix and its derivative as a Vector6 expressed in the local frame
  ///
  /// @param I Inertia matrix as a vector containing the diagonal and the three non
  /// diagonal values concatenated
  /// @param I_dot Derivative of inertia matrix expressed in the same way
  void setCoMInertiaMatrix(const Vector6 & I, const Vector6 & I_dot);

  /// @brief Set the inertia matrix as a Vector6 expressed in the local frame
  /// @details The derivative will be computed using finite differences
  ///
  /// @param I Inertia matrix as a vector containing the diagonal and the three non
  /// diagonal values concatenated
  void setCoMInertiaMatrix(const Vector6 & I);

  /// @brief Set the Angular Momentum around the CoM and its derviative expressed in the local frame
  ///
  /// @param sigma The angular momentum
  /// @param sigma_dot The angular momentum derivative
  void setCoMAngularMomentum(const Vector3 & sigma, const Vector3 & sigma_dot);

  /// @brief Set the Angular Momentum around the CoM  expressed in the local frame
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

  /// @brief Returns the predicted Kinematics object of the centroid in the world frame at the time of the measurement
  /// predictions

  /// @brief Converts a given wrench from the user to the centroid frame
  /// @details Performs the conversion of a wrench {force, torque} from the user frame to the centroid frame.
  ///
  void convertWrenchFromUserToCentroid(const Vector3 & forceUserFrame,
                                       const Vector3 & momentUserFrame,
                                       Vector3 & forceCentroidFrame,
                                       Vector3 & momentCentroidFrame);

  /// @brief Get the Kinematics of the observed local frame
  /// @details the kinematics are the main output of this observer. It includes the linear and angular position and
  /// velocity but not the accelerations by default. To get the acceleration call estimateAccelerations(). This
  /// method does NOT update the estimation, for this use update().
  ///
  /// @return Kinematics

  LocalKinematics getLocalCentroidKinematics() const;

  Kinematics getGlobalCentroidKinematics() const;

  /// @brief gets the Kinematics that include the linear and angular accelerations.
  /// @details This method computes the estimated accelerations from the observed state of the robot. It means this
  /// acceleration is filtered by the model
  ///
  /// @return Kinematics
  LocalKinematics estimateAccelerations();

  /// @brief Get the global-frame kinematics of a local-frame defined kinematics
  /// @details The kinematics are linear and angular positions, velocities and optionally accalerations. This method
  /// translates these kinematics from the local frame to the global frame.
  /// To enable accelerations, run estimateAccelerations() beforehand.
  ///
  /// @param localKinematics
  /// @return Kinematics
  LocalKinematics getLocalKinematicsOf(const LocalKinematics & localKinematics) const;
  /*
  Kinematics getGlobalKinematicsOf(const LocalKinematics & localKinematics) const;
  */
  Kinematics getGlobalKinematicsOf(const Kinematics & kin) const;

  /// get the contact force provided by the estimator
  /// which is different from a contact sensor measurement

  /// @brief Get the Estimated Contact Wrench
  /// This is useful in the case of uncertain wrench sensors or when contact force measurement is not available.
  ///
  /// @param contactNbr
  /// @return Vector6 Wrench
  Vector6 getContactWrench(int contactNbr) const;

  /// @brief Get the Contact 6D pose n in the global frame
  /// @details The contact position may be uncertain, this estimator uses the input data, the kinematic and the dynamic
  /// models to estimate the position of the contact in the environment This position is the "rest position" of the
  /// contact and therefore is different from what is obtained using forward kinematics because it does not include the
  /// contact deformation due to the contact forces
  ///
  /// @param contactNbr The contact number id
  /// @return Kinematics The pose
  Kinematics getContactPosition(int contactNbr) const;

  /// @brief Get the Unmodeled External Wrench (requires setWithUnmodeledWrench() to true before to update())
  /// @details In the presence of unmodeled and unmeasured external forces and moments, the dynamics of the robot
  /// behaves differently, this difference can be exploited to estimate the resultant of these forces and moments.
  ///
  /// @return Vector6
  Vector6 getUnmodeledWrench() const;
  /// @}

  /// ///////////////////////////////////////////////////////////
  /// @name Set values for state components
  /// These methods allow to update some parts of the state of the system based on guesses obtained independently
  // /////////////////////////////////////////////////////////

  /// @{
  /// @brief Set the State Kinematics
  /// @details Sets a value for the kinematics part of the state
  ///
  /// @param kine is the new kinematics of the state
  /// @param resetContactWrenches set if the contact wrenches should be reset
  /// @param resetCovariance set if the covariance of the state should be reset
  void setWorldCentroidStateKinematics(const LocalKinematics & kine,
                                       bool resetContactWrenches = true,
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

  /// @brief Reset the default values for the covariance matrix
  /// @details this is useful in case of misbehavior of the estimator or the sensors
  void resetSensorsDefaultCovMat();

  /// @brief  reset all the sensor inputs and provided contact information but keeps the contacts themselves
  void resetInputs();
  /// @}

  // /////////////////////////////////////////////////////////////
  /// @name Covariance matrices operations
  // /////////////////////////////////////////////////////////////
  /// @{

  /// @brief Set the Kinematics State Covariance
  void setKinematicsStateCovariance(const Matrix &);

  /// @brief Set the Default value for Kinematics Init Covariance
  void setKinematicsInitCovarianceDefault(const Matrix &);

  /// @brief Set the Kinematics Process Covariance
  void setKinematicsProcessCovariance(const Matrix &);

  /// @brief Set the Gyro Bias State Covariance
  ///
  /// @param covMat the nwe covariance matrix
  /// @param imuNumber  the number id of the IMU
  void setGyroBiasStateCovariance(const Matrix3 & covMat, unsigned imuNumber);

  /// @brief Set the Default value for Gyro Bias Init Covariance
  void setGyroBiasInitCovarianceDefault(const Matrix3 & covMat);

  /// @brief Set the Gyro Bias Process Covariance
  ///
  /// @param covMat the new process covariance matrix
  /// @param imuNumber the number id of the IMU
  void setGyroBiasProcessCovariance(const Matrix3 & covMat, unsigned imuNumber);

  /// @brief Set the Unmodeled Wrench State Cov Mat
  ///
  /// @param newCovMat
  void setUnmodeledWrenchStateCovMat(const Matrix6 & newCovMat);

  /// @brief Set the default value for init Unmodeled Wrench covariance matrix
  ///
  /// @param initCovMat
  void setUnmodeledWrenchInitCovMatDefault(const Matrix6 & initCovMat);

  /// @brief Set the Unmodeled Wrench Process Covariance Mattix
  ///
  /// @param processCovMat
  void setUnmodeledWrenchProcessCovMat(const Matrix6 & processCovMat);

  /// @brief Set the Contact State Covariance Matrix
  ///
  /// @param contactNbr
  /// @param contactCovMat the contact number id
  void setContactStateCovMat(int contactNbr, const Matrix12 & contactCovMat);

  /// @brief Set the default valut for the Initial Covariance Matrix of the contact in the state
  ///
  /// @param contactCovMat
  void setContactInitCovMatDefault(const Matrix12 & contactCovMat);

  /// @brief Set the Contact Process Covariance Matrix
  ///
  /// @param contactNbr
  /// @param contactCovMat the contact number id
  void setContactProcessCovMat(int contactNbr, const Matrix12 & contactCovMat);

  /// Resets the covariance matrices to their original values
  void resetStateCovarianceMat();
  void resetStateKinematicsCovMat();
  void resetStateGyroBiasCovMat(unsigned i);
  void resetStateUnmodeledWrenchCovMat();
  void resetStateContactsCovMat();
  void resetStateContactCovMat(unsigned contactNbr);

  void resetProcessCovarianceMat();
  void resetProcessKinematicsCovMat();
  void resetProcessGyroBiasCovMat(unsigned i);
  void resetProcessUnmodeledWrenchCovMat();
  void resetProcessContactsCovMat();
  void resetProcessContactCovMat(unsigned contactNbr);
  /// @}

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
  const TimeIndex getStateVectorTimeIndex() const;

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
  inline unsigned kineIndex() const;

  /// @brief Get the position index of the state vector
  ///
  /// @return unsigned
  inline unsigned posIndex() const;

  /// @brief Get the orientation index of the state vector
  ///
  /// @return unsigned
  inline unsigned oriIndex() const;

  /// @brief Get the linear velocity index of the state vector
  ///
  /// @return unsigned
  inline unsigned linVelIndex() const;

  /// @brief Get the angular velocity index of the state vector
  ///
  /// @return unsigned
  inline unsigned angVelIndex() const;

  /// @brief Get the gyro bias index of the state vector
  ///
  /// @return unsigned
  inline unsigned gyroBiasIndex(unsigned IMUNumber) const;

  /// @brief Get the unmodeled external wrench index of the state vector
  ///
  /// @return unsigned
  inline unsigned unmodeledWrenchIndex() const;

  /// @brief Get the unmodeled external linear force  index of the state vector
  ///
  /// @return unsigned
  inline unsigned unmodeledForceIndex() const;

  /// @brief Get the unmodeled external torque force  index of the state vector
  ///
  /// @return unsigned
  inline unsigned unmodeledTorqueIndex() const;

  /// @brief Get the index for the contact segment in the state vector
  ///
  /// @return unsigned
  inline unsigned contactsIndex() const;

  /// @brief Get the index of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactIndex(unsigned contactNbr) const;

  /// @brief Get the index of the kinematics of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactKineIndex(unsigned contactNbr) const;

  /// @brief Get the index of the position of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactPosIndex(unsigned contactNbr) const;

  /// @brief Get the index of the orientation of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactOriIndex(unsigned contactNbr) const;

  /// @brief Get the index of the linear force of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactForceIndex(unsigned contactNbr) const;

  /// @brief Get the index of the toraue of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactTorqueIndex(unsigned contactNbr) const;

  /// @brief Get the index of the wrench of a specific contact in the sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactWrenchIndex(unsigned contactNbr) const;

  // ///////////////////////////////////////////////////////////
  /// Getters for the indexes of the tangent state Vector
  /// //////////////////////////////////////////////////////////

  /// @brief Get the kinematics index of the tangent state vector
  ///
  /// @return unsigned
  inline unsigned kineIndexTangent() const;

  /// @brief Get the position index of the tangent state vector
  ///
  /// @return unsigned
  inline unsigned posIndexTangent() const;

  /// @brief Get the orientation index of the tangent state vector
  ///
  /// @return unsigned
  inline unsigned oriIndexTangent() const;

  /// @brief Get the linear velocity index of the tangent state vector
  ///
  /// @return unsigned
  inline unsigned linVelIndexTangent() const;

  /// @brief Get the angular velocity index of the tangent state vector
  ///
  /// @return unsigned
  inline unsigned angVelIndexTangent() const;

  /// @brief Get the gyro bias index of the tangent state vector
  ///
  /// @return unsigned
  inline unsigned gyroBiasIndexTangent(unsigned IMUNumber) const;

  /// @brief Get the unmodeled external wrench index of the tangent state vector
  ///
  /// @return unsigned
  inline unsigned unmodeledWrenchIndexTangent() const;

  /// @brief Get the unmodeled external linear force index of the tangent state vector
  ///
  /// @return unsigned
  inline unsigned unmodeledForceIndexTangent() const;

  /// @brief Get the unmodeled external torque force  index of the tangent state vector
  ///
  /// @return unsigned
  inline unsigned unmodeledTorqueIndexTangent() const;

  /// @brief Get the index for the contact segment in the tangent state vector
  ///
  /// @return unsigned
  inline unsigned contactsIndexTangent() const;

  /// @brief Get the index of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactIndexTangent(unsigned contactNbr) const;

  /// @brief Get the index of the kinematics of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactKineIndexTangent(unsigned contactNbr) const;

  /// @brief Get the index of the position of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactPosIndexTangent(unsigned contactNbr) const;

  /// @brief Get the index of the orientation of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactOriIndexTangent(unsigned contactNbr) const;

  /// @brief Get the index of the linear force of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactForceIndexTangent(unsigned contactNbr) const;

  /// @brief Get the index of the toraue of a specific contact in the tangent sate vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactTorqueIndexTangent(unsigned contactNbr) const;

  /// @brief Get the index of the wrench of a specific contact in the sate tangent vector
  ///
  /// @param contactNbr The contact number id
  /// @return unsigned
  inline unsigned contactWrenchIndexTangent(unsigned contactNbr) const;

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
  struct Sensor
  {
    Sensor(int signalSize) : measIndex(-1), measIndexTangent(-1), size(signalSize), time(0) {}
    virtual ~Sensor() {}
    int measIndex;
    int measIndexTangent;
    int size;
    TimeIndex time;

    inline Vector extractFromVector(const Vector & v)
    {
      return v.segment(size, measIndex);
    }
  };

  struct IMU : public Sensor
  {
    virtual ~IMU() {}
    IMU() : Sensor(sizeIMUSignal) {}
    int num;
    Kinematics userImuKinematics; // the kinematics of the IMU in the user's frame
    LocalKinematics centroidImuKinematics; // the kinematics of the IMU in the IMU's frame
    Vector6 acceleroGyro;
    Matrix3 covMatrixAccelero;
    Matrix3 covMatrixGyro;

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
    virtual ~Contact() {}

    struct Temp
    {
      Kinematics worldContactPose; // the pose of the contact in the world frane obtained by forward kinematics from the
                                   // centroid's frame. This is a temporary variable used for convenience. It must be
                                   // called with care as it is called in several functions in the code.
    };
    int num;
    Temp temp;

    Kinematics worldRestPose; // the rest pose of the contact in the world frame
    Kinematics worldRefPose_DEBUG; // the input rest pose of the contact in the world frame

    Vector6 wrenchMeasurement; /// Describes the measured wrench (forces + torques) at the contact in the sensor's frame
    CheckedMatrix6 sensorCovMatrix;

    Matrix3 linearStiffness;
    Matrix3 linearDamping;
    Matrix3 angularStiffness;
    Matrix3 angularDamping;

    bool isSet;
    bool withRealSensor;
    int stateIndex;
    int stateIndexTangent;
    Kinematics userContactKine; /// Describes the kinematics of the contact point in the centroid's frame.
    Kinematics centroidContactKine; /// Describes the kinematics of the contact point in the centroid's frame.
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

protected:
  ///////////// DYNAMICAL SYSTEM IMPLEMENTATION
  /// @brief Applies the state-transition model to the given state vector using the given input to predict the future
  /// state.
  /// @param x The current state vector
  /// @param u The current input vector
  /// @param k The current time index
  /// @return Vector&
  virtual Vector stateDynamics(const Vector & x, const Vector & u, TimeIndex k);

  /// @brief Applies the measurement model to the given state vector using the given input to predict the sensor
  /// measurements.
  /// @param x The current state vector
  /// @param u The current input vector
  /// @param k The current time index
  /// @return Vector&
  virtual Vector measureDynamics(const Vector & x, const Vector & u, TimeIndex k);

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

  /// @brief Computes the accelerations of the centroid in the world frame after a small increment of the kinematics.
  /// @details Uses the predicted kinematics in the visco-elastic model to obtain an estimate of the contact
  /// forces.These forces replace the contact forces of the previously know state to compute the accelerations. Used to
  /// compute the acceleration of the centroid in the world frame over the steps of the Runge-Kutta intergation.
  /// @param predictedWorldCentroidKinematics Newly predicted kinematics of the centroid in the world frame.
  virtual void computeRecursiveGlobalAccelerations_(Kinematics & predictedWorldCentroidKinematics);

  /// @brief @copybrief computeLocalAccelerations_(LocalKinematics & localStateKine, const Vector3 & totalForceLocal,
  /// const Vector3 & totalMomentLocal, Vector3 & linAcc, Vector3 & angAcc). Version adapted to local kinematics.
  /// @details @copydetails computeLocalAccelerations_(LocalKinematics & localStateKine, const Vector3 &
  /// totalForceLocal, const Vector3 & totalMomentLocal, Vector3 & linAcc, Vector3 & angAcc)
  virtual void computeRecursiveLocalAccelerations_(LocalKinematics & predictedWorldCentroidKinematics);

  /// @brief Computes the force exerted at a contact using the visco-elastic model on the given state vector.
  /// @param i Contact to estimate
  /// @param worldCentroidStateKinematics State vector used in the visco-elastic model
  /// @param worldRestContactPose Rest pose of the contact
  /// @param contactForce Empty vector of the contact force to estimate
  /// @param contactTorque Empty vector of the contact force to estimate
  void computeContactForce_(VectorContactIterator i,
                            LocalKinematics & worldCentroidStateKinematics,
                            Kinematics & worldRestContactPose,
                            Vector3 & contactForce,
                            Vector3 & contactTorque);

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
  const Vector6 getWorldContactWrench(const int & numContact) const;

  const Vector6 getCentroidContactWrench(const int & numContact) const;

  const Kinematics getCentroidContactInputPose(const int & numContact) const;

  const Kinematics getWorldContactInputRefPose(const int & numContact) const;

  const Kinematics getWorldContactPose(const int & numContact) const;

  const Kinematics getUserContactInputPose(const int & numContact) const;

  /// @brief Get the measurement index of the required IMU : allows to access its corresponding measurements in the
  /// measurement vector for example
  ///
  /// @return const int &
  const int getIMUMeasIndexByNum(const int & num) const;

  const int getContactMeasIndexByNum(const int & num) const;

  const bool getContactIsSetByNum(const int & num) const;

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

  /// @brief Define if we use the Runge-Kutta approximation method or not
  ///
  /// @param b true means we use finite differences
  virtual void useRungeKutta(bool b = true);

  virtual Matrix computeAMatrix();

  virtual Matrix computeCMatrix();

  /// @brief computes the local acceleration from a the state vector
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
  Vector stateNaNCorrection_();

  /// @brief update of the state kinematics worldCentroidStateKinematics_ and of the contacts pose with the newly
  /// estimated state
  void updateLocalKineAndContacts_();

  /// updates the global kinematics of the centroid from the local ones, that can be more interpretable
  void updateGlobalKine_();

protected:
  unsigned maxContacts_;
  unsigned maxImuNumber_;

  AbsolutePoseSensor absPoseSensor_;
  VectorContact contacts_;
  VectorIMU imuSensors_;

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

  Vector3 initTotalCentroidForce_; // Initial total force used in the Runge-Kutta integration, coming from the current
                                   // state's variables
  Vector3 initTotalCentroidTorque_;

  Vector3 initialVEContactForces_; // Initial contact forces used in the Runge-Kutta integration, coming from the
                                   // visco-elastic model used on the state's kinematics.
  Vector3 initialVEContactTorques_;

  Vector measurementVector_;
  Matrix measurementCovMatrix_;

  stateObservation::ExtendedKalmanFilter ekf_;
  bool finiteDifferencesJacobians_;
  bool withGyroBias_;
  bool withUnmodeledWrench_;
  bool withAccelerationEstimation_;

  bool withRungeKutta_; // defines whether to use the Runge-Kutta approximation method or not

  IndexedVector3 com_, comd_, comdd_;
  IndexedVector3 sigma_, sigmad_;
  IndexedMatrix3 I_, Id_;

  TimeIndex k_est_; // time index of the last estimation
  TimeIndex k_data_; // time index of the current measurements

  double mass_;

  double dt_;

  NoiseBase * processNoise_;
  NoiseBase * measurementNoise_;

  int numberOfContactRealSensors_;
  int currentIMUSensorNumber_;

  /// function to call before adding any measurement
  /// detects if there is a new estimation beginning and then
  /// calls the reset of the iteration
  void startNewIteration_();

  /// @brief Converts a LocalKinematics object from the user's frame to the centroid's frame, which is used for most of
  /// the computations
  /// @param userKine the LocalKinematics object expressed in the user's frame. It is likely to correspond to the IMU's
  /// LocalKinematics, defined by the user in its frame.
  /// @param centroidKine the LocalKinematics object corresponding to the converted LocalKinematics in the centroid's
  /// frame.

  inline void convertUserToCentroidFrame_(const Kinematics & userKine, Kinematics & centroidKine, TimeIndex k_data);

  /// @brief Converts a Kinematics object from the user's frame to the centroid's frame, which is used for most of the
  /// computations
  /// @param userKine the Kinematics object expressed in the user's frame. It is likely to correspond to the contact's
  /// Kinematics, defined by the user in its frame.

  inline Kinematics convertUserToCentroidFrame_(const Kinematics & userKine, TimeIndex k_data);

  /// Getters for the indexes of the state Vector using private types
  inline unsigned contactIndex(VectorContactConstIterator i) const;
  inline unsigned contactKineIndex(VectorContactConstIterator i) const;
  inline unsigned contactPosIndex(VectorContactConstIterator i) const;
  inline unsigned contactOriIndex(VectorContactConstIterator i) const;
  inline unsigned contactForceIndex(VectorContactConstIterator i) const;
  inline unsigned contactTorqueIndex(VectorContactConstIterator i) const;
  inline unsigned contactWrenchIndex(VectorContactConstIterator i) const;

  /// Getters for the indexes of the state Vector using private types
  inline unsigned contactIndexTangent(VectorContactConstIterator i) const;
  inline unsigned contactKineIndexTangent(VectorContactConstIterator i) const;
  inline unsigned contactPosIndexTangent(VectorContactConstIterator i) const;
  inline unsigned contactOriIndexTangent(VectorContactConstIterator i) const;
  inline unsigned contactForceIndexTangent(VectorContactConstIterator i) const;
  inline unsigned contactTorqueIndexTangent(VectorContactConstIterator i) const;
  inline unsigned contactWrenchIndexTangent(VectorContactConstIterator i) const;

public: ///////////SIZE OF VECTORS
  static const unsigned sizeAcceleroSignal = 3;
  static const unsigned sizeGyroSignal = 3;
  static const unsigned sizeIMUSignal = sizeAcceleroSignal + sizeGyroSignal;

  static const unsigned sizePos = 3;
  static const unsigned sizePosTangent = 3;
  static const unsigned sizeOri = 4;
  static const unsigned sizeOriTangent = 3;
  static const unsigned sizeLinVel = sizePos;
  static const unsigned sizeLinVelTangent = sizeLinVel;
  static const unsigned sizeLinAccTangent = sizeLinVelTangent;
  static const unsigned sizeAngVel = sizeOriTangent;
  static const unsigned sizeAngVelTangent = sizeAngVel;
  static const unsigned sizeGyroBias = sizeGyroSignal;
  static const unsigned sizeGyroBiasTangent = sizeGyroBias;

  static const unsigned sizeForce = 3;
  static const unsigned sizeForceTangent = sizeForce;
  static const unsigned sizeTorque = 3;
  static const unsigned sizeTorqueTangent = sizeTorque;

  static const unsigned sizeWrench = sizeForce + sizeTorque;

  static const unsigned sizeStateKine = sizePos + sizeOri + sizeLinVel + sizeAngVel;
  static const unsigned sizeStateBase = sizeStateKine + sizeForce + sizeTorque;
  static const unsigned sizeStateKineTangent = sizePos + sizeOriTangent + sizeLinVel + sizeAngVel;
  static const unsigned sizeStateTangentBase = sizeStateKineTangent + sizeForce + sizeTorque;

  static const unsigned sizePose = sizePos + sizeOri;
  static const unsigned sizePoseTangent = sizePos + sizeOriTangent;

  static const unsigned sizeContactKine = sizePose;
  static const unsigned sizeContactKineTangent = sizePoseTangent;

  static const unsigned sizeContact = sizeContactKine + sizeWrench;
  static const unsigned sizeContactTangent = sizeContactKineTangent + sizeWrench;

  static const Kinematics::Flags::Byte flagsStateKine = Kinematics::Flags::position | Kinematics::Flags::orientation
                                                        | Kinematics::Flags::linVel | Kinematics::Flags::angVel;

  static const Kinematics::Flags::Byte flagsContactKine = Kinematics::Flags::position | Kinematics::Flags::orientation;

  static const Kinematics::Flags::Byte flagsPoseKine = Kinematics::Flags::position | Kinematics::Flags::orientation;

  static const Kinematics::Flags::Byte flagsPosKine = Kinematics::Flags::position;

  static const Kinematics::Flags::Byte flagsIMUKine = Kinematics::Flags::position | Kinematics::Flags::orientation
                                                      | Kinematics::Flags::linVel | Kinematics::Flags::angVel
                                                      | Kinematics::Flags::linAcc | Kinematics::Flags::angAcc;

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
};

#include <state-observation/dynamics-estimators/kinetics-observer.hxx>

} // namespace stateObservation

#endif /// KINETICSOBSERVER_HPP
