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
/// local frame, called also observed frame in the global frame. This local frame can be any frame that is attached to
/// the robot with a known transformation. It could be attached to the CoM, to the base link of the robot or the control
/// frame that is supposed to be attached to the world frame but that actually is not. This estimation is based on the
/// assumption of viscoelastic contacts and using three kinds of measurements: IMUs, Force/Torque measurements (contact
/// and other ones) and any absolute position measurements.
///
class STATE_OBSERVATION_DLLAPI KineticsObserver : protected DynamicalSystemFunctorBase, protected StateVectorArithmetics
{
public:
  typedef kine::Kinematics Kinematics;
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

  /// @brief Set if the gyrometers bias is computed or not.
  ///        This parameter is global for all the IMUs.
  ///
  /// @param b
  void setWithGyroBias(bool b = true);

  /// @brief Set the total mass of the robot. This can be changed online
  ///
  /// @return sets
  void setMass(double);

  /// @}

  // ///////////////////////////////////////////////////////////
  /// @name Setting kinematic sensors
  /// These are the methods to be called at each iteration to give the control
  /// inputs and the sensor measurement for IMUs and absolute pose sensors.
  /// //////////////////////////////////////////////////////////

  /// @{

  /// @brief Set the measurements of an IMU and give the Kinematic of the IMU
  ///
  /// @details The overload that does not have the covariance matrices as an
  /// inputs uses default ones.
  ///
  /// The IMU is located in a sensor frame. We suppose we know the kinematics of
  /// this sensor frame in the local frame (for example the base frame or the
  /// control frame).
  ///
  /// @return the number of the IMU (useful in case there are several ones)
  /// @param accelero measured value
  /// @param gyrometer measured gyro value
  /// @param localKine sets the kinematics of the IMU expressed in the observed local
  /// frame. The best is to provide the position, the orientation,
  /// the angular and linear velocities and the linear acceleration
  /// Nevertheless if velocities or accelerations are not available they will be
  /// automatically computed through finite differences
  /// @param num the number of the IMU (useful in case there are several ones).
  ///           If not set it will be generated automatically.
  int setIMU(const Vector3 & accelero, const Vector3 & gyrometer, const Kinematics & localKine, int num = -1);

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
             const Kinematics & localKine,
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
  /// @param pose  is the initial guess on the position of the contact. Only position and orientation are enough. If the
  /// contact is compliant, you need to set the "rest" pose of the contact (i.e. the pose that gives zero reaction
  /// force)
  /// @param contactWrench  is the initial wrench on the contact expressed in the local frame of the contact frame
  /// (e.g. the force-sensor measurement)
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
                 const Vector6 & contactWrench,
                 const Matrix12 & initialCovarianceMatrix,
                 const Matrix12 & processCovarianceMatrix,
                 int contactNumber = -1,
                 const Matrix3 & linearStiffness = Matrix3::Constant(-1),
                 const Matrix3 & linearDamping = Matrix3::Constant(-1),
                 const Matrix3 & angularStiffness = Matrix3::Constant(-1),
                 const Matrix3 & angularDamping = Matrix3::Constant(-1));

  /// @brief Set a new contact with the environment (use default covariance matrices)
  ///
  /// @param pose  is the initial guess on the position of the contact. Only position and orientation are enough
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
                 const Vector6 & contactWrench = Vector6::Zero(),
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

  /// @brief Get the Current Number Of Contacts
  ///
  /// @return Index The current number of contacts
  Index getNumberOfContacts() const;

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
  /// @param localKine the new kinematics of the contact expressed in the local observed frame
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
  /// @param I Inertia matrix
  /// @param I_dot Derivative of inertia matrix
  void setInertiaMatrix(const Matrix3 & I, const Matrix3 & I_dot);

  /// @brief Set the 3x3 inertia matrix expressed in the local frame
  /// @details The derivative will be computed using finite differences
  ///
  /// @param I Inertia matrix
  /// @param I_dot Derivative of inertia matrix
  void setInertiaMatrix(const Matrix3 & I);

  /// @brief Set the inertia matrix and its derivative as a Vector6 expressed in the local frame
  ///
  /// @param I Inertia matrix as a vector containing the diagonal and the three non
  /// diagonal values concatenated
  /// @param I_dot Derivative of inertia matrix expressed in the same way
  void setInertiaMatrix(const Vector6 & I, const Vector6 & I_dot);

  /// @brief Set the inertia matrix as a Vector6 expressed in the local frame
  /// @details The derivative will be computed using finite differences
  ///
  /// @param I Inertia matrix as a vector containing the diagonal and the three non
  /// diagonal values concatenated
  void setInertiaMatrix(const Vector6 & I);

  /// @brief Set the Angular Momentum and its derviative expressed in the local frame
  ///
  /// @param sigma The angular momentum
  /// @param sigma_dot The angular momentum derivative
  void setAngularMomentum(const Vector3 & sigma, const Vector3 & sigma_dot);

  /// @brief Set the Angular Momentum expressed in the local frame
  /// @details The derivative will be computed using finite differences
  ///
  /// @param sigma The angular momentum
  void setAngularMomentum(const Vector3 & sigma);

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

  /// @brief Runs the estimation.
  /// @details This is the function that allows to
  /// 1- compute the estimation
  /// 2- move to the next iteration (increments time, resets the measurements, etc)
  ///
  /// @return const Vector& The state vector
  const Vector & update();

  /// @brief Get the Kinematics of the observed local frame
  /// @details the kinemactics are the main output of this observer. It includes the linear and angular position and
  /// velocity but not the accelerations by default. To get the acceleration call estimateAccelerations(). This
  /// method does NOT update the estimation, for this use update().
  ///
  /// @return Kinematics
  Kinematics getKinematics() const;

  /// @brief gets the Kinematics that include the linear and angular accelerations.
  /// @details This method computes the estimated accelerations from the observed state of the robot. It means this
  /// acceleration is filtered by the model
  ///
  /// @return Kinematics
  Kinematics estimateAccelerations();

  /// @brief Get the global-frame kinematics of a local-frame defined kinematics
  /// @details The kinematics are linear and angular positions, velocities and optionally accalerations. This method
  /// translates these kinematics from the local frame to the global frame.
  /// To enable accelerations, run estimateAccelerations() beforehand.
  ///
  /// @param localKinematics
  /// @return Kinematics
  Kinematics getKinematicsOf(const Kinematics & localKinematics) const;

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
  void setStateKinematics(const Kinematics & kine, bool resetContactWrenches = true, bool resetCovariance = true);

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
    Kinematics kinematics;
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
    Contact() : Sensor(sizeWrench), isSet(false), withRealSensor(false), stateIndex(-1), stateIndexTangent(-1) {}
    virtual ~Contact() {}

    Kinematics absPose;
    Vector6 wrench;
    CheckedMatrix6 sensorCovMatrix;

    Matrix3 linearStiffness;
    Matrix3 linearDamping;
    Matrix3 angularStiffness;
    Matrix3 angularDamping;

    bool isSet;
    bool withRealSensor;
    int stateIndex;
    int stateIndexTangent;

    Kinematics localKine; /// describes the kinematics of the contact point in the local frame
    static const Kinematics::Flags::Byte localKineFlags = /// flags for the components of the kinematics
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
  virtual Vector stateDynamics(const Vector & x, const Vector & u, TimeIndex k);

  virtual Vector measureDynamics(const Vector & x, const Vector & u, TimeIndex k);

  void addUnmodeledAndContactWrench_(const Vector & stateVector, Vector3 & force, Vector3 & torque);

  void computeAccelerations_(Kinematics & stateKine,
                             const Vector3 & totalForceLocal,
                             const Vector3 & totalMomentLocal,
                             Vector3 & linAcc,
                             Vector3 & angAcc);

  /// the kinematics is not const to allow more optimized non const operators to work
  void computeContactForces_(VectorContactIterator i,
                             Kinematics & stateKine,
                             Kinematics & contactPose,
                             Vector3 & Force,
                             Vector3 torque);

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

  /// @}

protected:
  Vector stateNaNCorrection_();

  /// updates stateKine_ from the stateVector
  void updateKine_();

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

  Kinematics stateKinematics_;

  Vector stateVector_;
  Vector stateVectorDx_;
  Vector oldStateVector_;

  Vector3 additionalForce_;
  Vector3 additionalTorque_;

  Vector measurementVector_;
  Matrix measurementCovMatrix_;

  stateObservation::ExtendedKalmanFilter ekf_;
  bool finiteDifferencesJacobians_;
  bool withGyroBias_;
  bool withUnmodeledWrench_;
  bool withAccelerationEstimation_;

  IndexedVector3 com_, comd_, comdd_;
  IndexedVector3 sigma_, sigmad_;
  IndexedMatrix3 I_, Id_;

  TimeIndex k_est_;
  TimeIndex k_data_;

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

  virtual Matrix computeAMatrix_();
  virtual Matrix computeCMatrix_();

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

public:
  static const double gravityAccelerationConstant = 9.8;
  ///////////SIZE OF VECTORS

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

  static const Kinematics::Flags::Byte flagsIMUKine = Kinematics::Flags::position | Kinematics::Flags::orientation
                                                      | Kinematics::Flags::linVel | Kinematics::Flags::angVel
                                                      | Kinematics::Flags::linAcc;

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
    Opt() : kine(kine1), ori(kine.orientation), ori1(kine1.orientation), ori2(kine2.orientation) {}

    Kinematics kine1, kine2;
    Kinematics & kine;
    Orientation & ori;
    Orientation & ori1;
    Orientation & ori2;
  } opt_;
};

#include <state-observation/dynamics-estimators/kinetics-observer.hxx>

} // namespace stateObservation

#endif /// KINETICSOBSERVER_HPP
