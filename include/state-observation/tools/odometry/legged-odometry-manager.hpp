#ifndef LEGGEDODOMETRYMANAGERHPP
#define LEGGEDODOMETRYMANAGERHPP

#include <set>
#include <state-observation/api.h>
#include <state-observation/tools/measurements-manager/ContactsManager.hpp>
#include <state-observation/tools/measurements-manager/measurements.hpp>

namespace stateObservation
{
namespace odometry
{
using namespace kine;
typedef Eigen::Vector<double, 7> Vector7;

/**
 * Interface for the implementation of legged odometry. This odometry is based on the tracking of successive contacts
 * for the estimation of the pose of the floating base of the robot.

 * The tilt cannot be estimated from this method (but the yaw can), it has to be estimated beforehand by another
 * observer.
 * One can decide to perform flat or 6D odometry. The flat odometry considers that the robot walks on a flat
 * ground and corrects the estimated height accordingly, it is preferable in this use case.
 *
 * The odometry manager must be initialized once all the configuration parameters are retrieved using the init function,
 * and called on every iteration with \ref LeggedOdometryManager::run(const mc_control::MCController & ctl,
 * mc_rtc::Logger & logger, sva::PTransformd & pose, sva::MotionVecd & vel, sva::MotionVecd & acc).
 **/

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////
// Enhancement of the class Contact for legged odometry.
class LoContact : public measurements::Contact
{
  using measurements::Contact::Contact;

public:
  inline void resetContact() noexcept override
  {
    measurements::Contact::resetContact();
    lifeTime_ = 0.0;
  }

  inline void lambda(double lambda)
  {
    lambda_ = lambda;
  }

  inline void resetLifeTime()
  {
    lifeTime_ = 0.0;
  }
  inline void lifeTimeIncrement(double dt)
  {
    lifeTime_ += dt;
  }
  inline void correctionWeightingCoeff(double weightingCoeff)
  {
    correctionWeightingCoeff_ = weightingCoeff;
  }

  inline double lambda() const noexcept
  {
    return lambda_;
  }

  inline double lifeTime() const noexcept
  {
    return lifeTime_;
  }
  inline double correctionWeightingCoeff() const noexcept
  {
    return correctionWeightingCoeff_;
  }

public:
  // reference of the contact in the world
  Kinematics worldRefKine_;
  // reference of the contact in the world before correction
  Kinematics worldRefKineBeforeCorrection_;
  // new incoming ref kine for the correction
  Kinematics newIncomingWorldRefKine_;
  // indicates whether the contact can be used for the orientation odometry or not
  bool useForOrientation_ = false;
  // current estimation of the kinematics of the floating base in the world, obtained from the reference pose of the
  // contact in the world
  Kinematics worldFbKineFromRef_;
  // current estimation of the kinematics of the contact in the world. Avoids recomputations.
  Kinematics currentWorldKine_;
  // kinematics of the frame of the floating base in the frame of the contact, obtained by forward kinematics.
  Kinematics contactFbKine_;

protected:
  // weighing coefficient for the anchor point computation
  double lambda_;
  // time ellapsed since the creation of the contact.
  double lifeTime_;
  // defines the weighting of the contribution of the newly "measured" reference pose of the contact and the current one
  double correctionWeightingCoeff_;
};

/// @brief Structure that implements all the necessary functions to perform legged odometry.
/// @details Handles the odometry from the contacts detection to the final pose estimation of the floating base. Also
/// allows to compute the position and/or velocity of an anchor point linked to the robot.
struct STATE_OBSERVATION_DLLAPI LeggedOdometryManager
{
public:
  using ContactsManager = measurements::ContactsManager<LoContact>;

  struct KineParams
  {
    /// @brief Structure containing all the kinematic parameters required to run the legged odometry

    /// @var Kinematics & pose /* Pose of the floating base of the robot in the world that we want to update with
    /// the odometry */
    /// @var bool oriIsAttitude /* Informs if the rotation matrix ContactUpdateParameters#tiltOrAttitude stored in this
    /// structure is a tilt or an attitude (full orientation). */
    /// @var Eigen::Matrix3d* tiltOrAttitude /* Input orientation of the floating base in the world, used to perform the
    /// legged odometry. If only a tilt is provided, the yaw will come from the yaw of the contacts. */

    KineParams & positionMeas(const Eigen::Vector3d & worldPosMeas)
    {
      this->worldPosMeas = &worldPosMeas;
      return *this;
    }

    KineParams & tiltMeas(const Eigen::Matrix3d & tiltMeas)
    {
      BOOST_ASSERT_MSG(!tiltOrAttitudeMeas, "An input attitude is already set");
      oriIsAttitude = false;
      tiltOrAttitudeMeas = &tiltMeas;
      return *this;
    }

    KineParams & attitudeMeas(const Eigen::Matrix3d & oriMeas)
    {
      BOOST_ASSERT_MSG(!tiltOrAttitudeMeas, "An input tilt is already set");
      oriIsAttitude = true;
      tiltOrAttitudeMeas = &oriMeas;
      return *this;
    }

    static KineParams fromOther(const KineParams & other)
    {
      KineParams out(other.kineToUpdate.toVector(Kinematics::Flags::pose));
      out.oriIsAttitude = other.oriIsAttitude;
      out.tiltOrAttitudeMeas = other.tiltOrAttitudeMeas;
      return out;
    }

    explicit KineParams(const Vector7 & pose)
    {
      kineToUpdate.position = pose.segment(0, 3);
      kineToUpdate.orientation.fromVector4(pose.segment(3, 4));
    }

    /* Variables to update */
    // Kinematics of the floating base of the robot in the world that we want to update with the odometry.
    Kinematics kineToUpdate;
    /* Inputs */

    // Input position of the floating base in the world, used to perform the
    // legged odometry.
    const Eigen::Vector3d * worldPosMeas = nullptr;
    // Informs if the rotation matrix ContactUpdateParameters#tiltOrAttitude stored in this structure
    // is a tilt or an attitude (full orientation).
    bool oriIsAttitude = false;
    // Input orientation of the floating base in the world, used to perform the
    // legged odometry. If only a tilt is provided, the yaw will come from the yaw of the contacts.
    const Eigen::Matrix3d * tiltOrAttitudeMeas = nullptr;
  };

  struct ContactInputData
  {
    ContactInputData(const Kinematics & contactFbKine, double lambda) : contactFbKine_(contactFbKine), lambda_(lambda)
    {
    }

    Kinematics contactFbKine_;
    double lambda_;
  };

  template<typename OnNewContactObserver = std::nullptr_t,
           typename OnMaintainedContactObserver = std::nullptr_t,
           typename OnRemovedContactObserver = std::nullptr_t,
           typename OnAddedContactObserver = std::nullptr_t>
  struct ContactUpdateParameters
  {
    /// @brief Structure containing all the functions required to update the contact

    /// @var OnNewContactObserver* onNewContactFn /* Function defined in the observer using the legged odometry that
    /// must be called when a contact is newly detected. */
    /// @var OnMaintainedContactObserver* onMaintainedContactFn /* Function defined in the observer using the legged
    /// odometry that must be called on all the contacts maintained with the environment. */
    /// @var OnRemovedContactObserver* onRemovedContactFn /* Function defined in the observer using the legged odometry
    /// that must be called when a contact is broken. */
    /// @var OnAddedContactObserver* onAddedContactFn /* Function defined in the observer using the legged odometry that
    /// must be called when a contact is newly added to the manager (used to add it to the gui, to logs that must be
    /// written since its first detection, etc.) */

    explicit ContactUpdateParameters(const std::unordered_map<std::string, ContactInputData> & contactData)
    : contactData_(contactData)
    {
    }

    template<typename OnNewContactOther>
    ContactUpdateParameters<OnNewContactOther,
                            OnMaintainedContactObserver,
                            OnRemovedContactObserver,
                            OnAddedContactObserver>
        onNewContact(OnNewContactOther & onNewContact)
    {
      auto out = ContactUpdateParameters<OnNewContactOther, OnMaintainedContactObserver, OnRemovedContactObserver,
                                         OnAddedContactObserver>::fromOther(*this);
      out.onNewContactFn = &onNewContact;
      return out;
    }

    template<typename OnMaintainedContactOther>
    ContactUpdateParameters<OnNewContactObserver,
                            OnMaintainedContactOther,
                            OnRemovedContactObserver,
                            OnAddedContactObserver>
        onMaintainedContact(OnMaintainedContactOther & onMaintainedContact)
    {
      auto out = ContactUpdateParameters<OnNewContactObserver, OnMaintainedContactOther, OnRemovedContactObserver,
                                         OnAddedContactObserver>::fromOther(*this);
      out.onMaintainedContactFn = &onMaintainedContact;
      return out;
    }

    template<typename OnRemovedContactOther>
    ContactUpdateParameters<OnNewContactObserver,
                            OnMaintainedContactObserver,
                            OnRemovedContactOther,
                            OnAddedContactObserver>
        onRemovedContact(OnRemovedContactOther & onRemovedContact)
    {
      auto out = ContactUpdateParameters<OnNewContactObserver, OnMaintainedContactObserver, OnRemovedContactOther,
                                         OnAddedContactObserver>::fromOther(*this);
      out.onRemovedContactFn = &onRemovedContact;
      return out;
    }

    template<typename OnAddedContactOther>
    ContactUpdateParameters<OnNewContactObserver,
                            OnMaintainedContactObserver,
                            OnRemovedContactObserver,
                            OnAddedContactOther>
        onAddedContact(OnAddedContactOther & onAddedontact)
    {
      auto out = ContactUpdateParameters<OnNewContactObserver, OnMaintainedContactObserver, OnRemovedContactObserver,
                                         OnAddedContactOther>::fromOther(*this);
      out.onAddedContactFn = &onAddedontact;
      return out;
    }

    template<typename OnNewContactOther,
             typename OnMaintainedContactOther,
             typename OnRemovedContactOther,
             typename OnAddedContactOther>
    static ContactUpdateParameters fromOther(const ContactUpdateParameters<OnNewContactOther,
                                                                           OnMaintainedContactOther,
                                                                           OnRemovedContactOther,
                                                                           OnAddedContactOther> & other)
    {
      ContactUpdateParameters out;
      if constexpr(std::is_same_v<OnNewContactOther, OnNewContactObserver>)
      {
        out.onNewContactFn = other.onNewContactFn;
      }
      if constexpr(std::is_same_v<OnMaintainedContactOther, OnMaintainedContactObserver>)
      {
        out.onMaintainedContactFn = other.onMaintainedContactFn;
      }
      if constexpr(std::is_same_v<OnRemovedContactOther, OnRemovedContactObserver>)
      {
        out.onRemovedContactFn = other.onRemovedContactFn;
      }
      if constexpr(std::is_same_v<OnAddedContactOther, OnAddedContactObserver>)
      {
        out.onAddedContactFn = other.onAddedContactFn;
      }
      return out;
    }

    // Function defined in the observer using the legged odometry that must be called when a contact is newly detected.
    OnNewContactObserver * onNewContactFn = nullptr;
    /* Function defined in the observer using the legged odometry that must be called on all the contacts maintained
     * with the environment. */
    OnMaintainedContactObserver * onMaintainedContactFn = nullptr;
    /* Function defined in the observer using the legged odometry that must be called when a contact is broken. */
    OnRemovedContactObserver * onRemovedContactFn = nullptr;
    /* Function defined in the observer using the legged odometry that must be called when a contact is newly added to
     * the manager (used to add it to the gui, to logs that must be written since its first detection, etc.) */
    OnAddedContactObserver * onAddedContactFn = nullptr;
    std::unordered_map<std::string, ContactInputData> contactData_;
  };

protected:
  ///////////////////////////////////////////////////////////////////////
  /// ------------------------Contacts Manager---------------------------
  ///////////////////////////////////////////////////////////////////////

  /// @brief Adaptation of the structure ContactsManager to the legged odometry, using personalized contacts classes.
  struct LeggedOdometryContactsManager : public ContactsManager
  {
  protected:
    // comparison function that sorts the contacts based on their lambda.
    struct sortByLambda
    {
      inline bool operator()(const LoContact & contact1, const LoContact & contact2) const noexcept
      {
        return (contact1.lambda() < contact2.lambda());
      }
    };

  public:
    // list of contacts used for the orientation odometry. At most two contacts can be used for this estimation, and
    // contacts at hands are not considered. The contacts with the highest lambda are used.
    std::set<std::reference_wrapper<LoContact>, sortByLambda> oriOdometryContacts_;
  };

public:
  ////////////////////////////////////////////////////////////////////
  /// ------------------------Configuration---------------------------
  ////////////////////////////////////////////////////////////////////

  /// @brief Configuration structure that helps setting up the odometry parameters
  /// @details The configuration is used once passed in the @ref init(const mc_control::MCController &, Configuration,
  /// ContactsManagerConfiguration) function
  struct Configuration
  {
    /// @brief Configuration's constructor
    /// @details This version allows to set the odometry type directly from a string, most likely obtained from a
    /// configuration file.
    inline Configuration(const std::string & odometryTypeString) noexcept
    {
      odometryType_ = measurements::stringToOdometryType(odometryTypeString);
      BOOST_ASSERT_MSG(
          odometryType_ == measurements::OdometryType::Flat || odometryType_ == measurements::OdometryType::Odometry6d,
          "Odometry type not allowed. Please pick among : [Odometry6d, Flat] or use the other Configuration "
          "constructor for an estimator that can run without odometry.");
    }

    /// @brief Configuration's constructor
    /// @details This versions allows to initialize the type of odometry directly with an OdometryType object.
    inline Configuration(measurements::OdometryType odometryType) noexcept : odometryType_(odometryType) {}

    // Desired kind of odometry (6D or flat)
    measurements::OdometryType odometryType_;

    // Indicates if the orientation must be estimated by this odometry.
    bool withYaw_ = true;
    // Indicates if the reference pose of the contacts must be corrected at the end of each iteration.
    bool correctContacts_ = true;

    inline Configuration & withYawEstimation(bool withYaw) noexcept
    {
      withYaw_ = withYaw;
      return *this;
    }
    inline Configuration & correctContacts(bool correctContacts) noexcept
    {
      correctContacts_ = correctContacts;
      return *this;
    }
  };

  inline LeggedOdometryManager(double dt)
  {
    ctl_dt_ = dt;
  }
  /**
   * @brief  Returns a list of pointers to the contacts maintained during the current iteration.
   *
   * @return const std::vector<LoContact *>&
   */
  inline const std::vector<LoContact *> & newContacts()
  {
    return newContacts_;
  }
  /**
   * @brief  Returns a list of pointers to the contacts created on the current iteration.
   *
   * @return const std::vector<LoContact *>&
   */
  inline const std::vector<LoContact *> & maintainedContacts()
  {
    return maintainedContacts_;
  }

  /// @brief Initializer for the odometry manager.
  /// @param odomConfig Desired configuration of the odometry
  /// @param initPose Initial pose of the floating base
  /// @param worldAnchorKine Kinematics of the anchor frame in the world frame
  void init(const Configuration & odomConfig, const Vector7 & initPose, const Kinematics & worldAnchorKine);

  /// @brief Function that initializes the loop of the legged odometry. To be called at the beginning of each iteration.
  /// @details Updates the joints configuration, the contacts, and sets the velocity and acceleration of the odometry
  /// robot to zero as we internally compute only kinematics in frames attached to the robots. This function is
  /// necessary because one can start computing the anchor point or using the run(const mc_control::MCController &, *
  /// mc_rtc::Logger &, sva::PTransformd &, sva::MotionVecd &, sva::MotionVecd &) function, that require these updates.
  /// @param runParams Functions called when creating/upadting/breaking contacts.
  template<typename OnNewContactObserver = std::nullptr_t,
           typename OnMaintainedContactObserver = std::nullptr_t,
           typename OnRemovedContactObserver = std::nullptr_t,
           typename OnAddedContactObserver = std::nullptr_t>
  void initLoop(const ContactUpdateParameters<OnNewContactObserver,
                                              OnMaintainedContactObserver,
                                              OnRemovedContactObserver,
                                              OnAddedContactObserver> & runParams);

  /// @brief Function that runs the legged odometry loop. Using the input orientation and the current contacts, it
  /// estimates the new state of the robot.
  /// @param kineParams Kinematic parameters necessary for the estimation, either inputs or outputs to modify (please
  /// see the documentation of the KineParams class).
  void run(KineParams & kineParams);

  /// @brief Replaces the current pose of the odometry robot by the given one.
  /// @details Also changes the reference pose of the contacts. Updates the velocity and acceleration with the new
  /// orientation if required.
  /// @param newPose New pose of the odometry robot.
  void replaceRobotPose(const Vector7 & newPose);

  /// @brief Gives the kinematics (position and linear velocity) of the anchor point in the desired frame.
  /// @details If the velocity of the target frame in the world frame is given, the velocity of the anchor point in the
  /// target frame will also be contained in the returned Kinematics object.
  /// @param worldTargetKine Kinematics of the target frame in the world frame.
  Kinematics getAnchorKineIn(Kinematics & worldTargetKine);

  /**
   * @brief Returns the position of the anchor point in the world from the current contacts reference position.
   *
   * @return stateObservation::Vector3&
   */
  const stateObservation::Vector3 & getWorldRefAnchorPos();

  /// @brief Changes the type of the odometry
  /// @details Version meant to be called by the observer using the odometry during the run through the gui.
  /// @param newOdometryType The string naming the new type of odometry to use.
  void setOdometryType(measurements::OdometryType newOdometryType);

  inline void kappa(double kappa) noexcept
  {
    kappa_ = kappa;
  }
  inline void lambdaInf(double lambdaInf) noexcept
  {
    lambdaInf_ = lambdaInf;
  }

  /// @brief Getter for the contacts manager.
  inline LeggedOdometryContactsManager & contactsManager()
  {
    return contactsManager_;
  }

private:
  /// @brief Updates the pose of the contacts and estimates the associated kinematics.
  /// @param runParams Parameters used to run the legged odometry.
  template<typename OnNewContactObserver = std::nullptr_t,
           typename OnMaintainedContactObserver = std::nullptr_t,
           typename OnRemovedContactObserver = std::nullptr_t,
           typename OnAddedContactObserver = std::nullptr_t>
  void updateContacts(const ContactUpdateParameters<OnNewContactObserver,
                                                    OnMaintainedContactObserver,
                                                    OnRemovedContactObserver,
                                                    OnAddedContactObserver> & params);

  /// @brief Updates the floating base pose given as argument by the observer.
  /// @param pose The pose of the floating base in the world that we want to update
  void updateFbKinematicsPvt(Kinematics & pose);

  /// @brief Estimates the floating base from the currently set contacts and updates them.
  /// @param runParams Parameters used to run the legged odometry.
  void updateFbAndContacts(const KineParams & params);

  /// @brief Updates the velocity of the floating base.
  void updateFbVelocity(const Vector3 & linVel, Vector3 * angVel = nullptr);

  /// @brief Updates the position of the floating base in the world.
  /// @details For each maintained contact, we compute the position of the floating base in the contact frame, we
  /// then compute their weighted average and obtain the estimated translation from the anchor point to the floating
  /// base.  We apply this translation to the reference position of the anchor frame in the world to obtain the new
  /// position of the floating base in the word.
  stateObservation::Vector3 getWorldFbPosFromAnchor();

  /// @brief Corrects the reference pose of the contacts after the update of the floating base.
  /// @details The new reference pose is obtained by forward kinematics from the updated floating base.
  void correctContactsRef();

  /// @brief Computes the reference kinematics of the newly set contact in the world.
  /// @param contact The new contact
  void setNewContact(LoContact & contact);

  /// @brief Computes the kinematics of the contact attached to the odometry robot in the world frame from the current
  /// floating base pose and encoders.
  /// @param contact Contact of which we want to compute the kinematics.
  /// @return Kinematics &.
  const Kinematics & getContactKinematics(LoContact & contact);

  /// @brief Gives the kinematics of the contact in the desired frame.
  /// @details If the velocity of the target frame in the world frame is given, the velocity of the anchor point in the
  /// target frame will also be contained in the returned Kinematics object.
  /// @param worldTargetKine Kinematics of the target frame in the world frame.
  Kinematics getContactKineIn(LoContact & contact, Kinematics & worldTargetKine);

  /// @brief Selects which contacts to use for the orientation odometry and computes the orientation of the floating
  /// base for each of them
  /// @details The two contacts with the highest lambda are selected.
  /// @param oriUpdatable Indicates that contacts can be used to estimated the orientation.
  /// @param sumLambdasOrientation Sum of the lambdas of the contacts used for the orientation estimation
  void selectForOrientationOdometry(bool & oriUpdatable, double & sumLambdasOrientation);

protected:
  // category to plot the odometry in
  std::string category_;

  // contacts manager used by this odometry manager
  LeggedOdometryContactsManager contactsManager_;
  // tracked kinematics of the floating base
  Kinematics fbKine_;

  // contacts created on the current iteration
  std::vector<LoContact *> newContacts_;
  // contacts maintained during the current iteration
  std::vector<LoContact *> maintainedContacts_;
  // time constant defining how fast the contact reference poses are corrected by the one of the floating base
  double kappa_ = 1 / (2 * M_PI);
  // gain allowing for the contribution of the contact pose measurement into the reference pose even after a long
  // contact's lifetime.
  double lambdaInf_ = 0.02;
  // timestep used in the controller
  double ctl_dt_;

  // indicates whether we want to update the yaw using this method or not
  bool withYawEstimation_;
  // Indicates if the reference pose of the contacts must be corrected at the end of each iteration.
  bool correctContacts_ = true;

  // position of the anchor point of the robot in the world
  stateObservation::Vector3 worldAnchorPos_;
  // position of the anchor point of the robot in the world, obtained from the contact references.
  stateObservation::Vector3 worldRefAnchorPosition_;
  // position of the anchor point in the frame of the floating base.
  stateObservation::Vector3 fbAnchorPos_;

  // Indicates if the previous anchor point was obtained using contacts
  bool prevAnchorFromContacts_ = true;
  // Indicates if the current anchor point was obtained using contacts
  bool currAnchorFromContacts_ = true;
  // indicated if the position can be updated using contacts. True if a contact is currently set.
  bool posUpdatable_ = false;

  // time stamp, incremented on the intiialization of each iteration.
  stateObservation::TimeIndex k_iter_ = 0;
  // time stamp, incremented once the reading of the joint encoders and the contacts are updated
  stateObservation::TimeIndex k_data_ = 0;
  // time stamp, incremented once the kinematics of the odometry robot have been updated.
  stateObservation::TimeIndex k_est_ = 0;
  // time stamp, incremented once the contact references have been corrected.
  stateObservation::TimeIndex k_correct_ = 0;
  // time stamp, incremented once the anchor frame has been computed.
  stateObservation::TimeIndex k_anchor_ = 0;

public:
  // Indicates if the desired odometry must be a flat or a 6D odometry.
  using OdometryType = measurements::OdometryType;
  measurements::OdometryType odometryType_;
};

} // namespace odometry
} // namespace stateObservation

#include <state-observation/tools/odometry/legged-odometry-manager.hxx>

#endif // LEGGEDODOMETRYMANAGERHPP
