#ifndef LEGGEDODOMETRYMANAGERHPP
#define LEGGEDODOMETRYMANAGERHPP

#include <set>
#include <state-observation/api.h>
#include <state-observation/tools/measurements-manager/ContactsManager.hpp>

#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace stateObservation
{
namespace odometry
{

// allowed odometry types
enum class OdometryType
{
  Odometry6d,
  Flat,
  None
};
namespace internal
{
// map allowing to get the OdometryType value associated to the given string
inline static const std::unordered_map<std::string, OdometryType> strToOdometryType_ = {
    {"6D", OdometryType::Odometry6d},
    {"Flat", OdometryType::Flat},
    {"None", OdometryType::None}};
// map allowing to get the string value associated to the given OdometryType object
inline static const std::unordered_map<OdometryType, std::string> odometryTypToStr_ = {{OdometryType::Odometry6d, "6D"},
                                                                                       {OdometryType::Flat, "Flat"},
                                                                                       {OdometryType::None, "None"}};
} // namespace internal

/// @brief Returns an OdometryType object corresponding to the given string
/// @details Allows to set the odometry type directly from a string, most likely obtained from a configuration file.
/// @param str The string naming the desired odometry
/// @return OdometryType
inline static OdometryType stringToOdometryType(const std::string & str)
{
  auto it = internal::strToOdometryType_.find(str);
  BOOST_ASSERT_MSG(it != internal::strToOdometryType_.end(),
                   (": No known OdometryType value for " + str + ".").c_str());

  return it->second;
}

/// @brief Returns the string value associated to the given OdometryType object
/// @details This can be used to display the name of the method in the gui for example. This function assumes the given
/// type is valid.
/// @param odometryType The current odometry type
/// @return std::string
inline static std::string odometryTypeToString(OdometryType odometryType)
{
  return internal::odometryTypToStr_.at(odometryType);
}

using namespace kine;
typedef Eigen::Matrix<double, 7, 1> Vector7;

/**
 * Interface for the implementation of legged odometry. This odometry is based on the tracking of successive contacts
 * for the estimation of the pose of the body of the robot.

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
  // current estimation of the kinematics of the body in the world, obtained from the reference pose of the
  // contact in the world
  Kinematics worldBodyKineFromRef_;
  // current estimation of the kinematics of the contact in the world. Avoids recomputations.
  Kinematics currentWorldKine_;
  // kinematics of the frame of the body in the frame of the contact, obtained by forward kinematics.
  // Kinematics contactBodyKine_;
  Kinematics bodyContactKine_;

  // weighing coefficient for the anchor point computation
  double lambda_;
  // time ellapsed since the creation of the contact.
  double lifeTime_;
  // defines the weighting of the contribution of the newly "measured" reference pose of the contact and the current one
  double correctionWeightingCoeff_;
};

/// @brief Structure that implements all the necessary functions to perform legged odometry.
/// @details Handles the odometry from the contacts detection to the final pose estimation of the body. Also
/// allows to compute the position and/or velocity of an anchor point linked to the robot.
struct STATE_OBSERVATION_DLLAPI LeggedOdometryManager
{
public:
  using ContactsManager = measurements::ContactsManager<LoContact>;

  struct KineParams
  {
    /// @brief Structure containing all the kinematic parameters required to run the legged odometry

    /// @var Kinematics & pose /* Pose of the body of the robot in the world that we want to update with
    /// the odometry */
    /// @var Eigen::Matrix3d* attitude /* Input orientation of the body in the world, used to perform the
    /// legged odometry.  */
    /// @var Eigen::Vector3d* tilt /* Input tilt of the body in the world, used to perform the
    /// legged odometry. As only a tilt is provided, the yaw will come from the yaw of the contacts. */

    KineParams & positionMeas(const Eigen::Vector3d & worldPosMeas)
    {
      this->worldPosMeas = &worldPosMeas;
      return *this;
    }

    KineParams & tiltMeasurement(const Eigen::Vector3d & tilt)
    {
      BOOST_ASSERT_MSG(attitudeMeas == nullptr, "An input attitude is already set");
      tiltMeas = &tilt;
      return *this;
    }

    KineParams & attitudeMeasurement(const Eigen::Matrix3d & oriMeas)
    {
      BOOST_ASSERT_MSG(tiltMeas == nullptr, "An input tilt is already set");
      attitudeMeas = &oriMeas;
      return *this;
    }

    static KineParams fromOther(const KineParams & other)
    {
      KineParams out(*other.kineToUpdate);
      out.attitudeMeas = other.attitudeMeas;
      out.tiltMeas = other.tiltMeas;
      out.worldPosMeas = other.worldPosMeas;
      return out;
    }

    explicit KineParams(Kinematics & pose)
    {
      kineToUpdate = &pose;
    }

    /* Variables to update */
    // Kinematics of the body of the robot in the world that we want to update with the odometry.
    Kinematics * kineToUpdate;
    /* Inputs */

    // Input position of the body in the world, used to perform the
    // legged odometry.
    const Vector3 * worldPosMeas = nullptr;
    // Input orientation of the body in the world, used to perform the
    // legged odometry. If only a tilt is provided, the yaw will come from the yaw of the contacts.
    const Matrix3 * attitudeMeas = nullptr;
    // Input orientation of the body in the world, used to perform the
    // legged odometry. If only a tilt is provided, the yaw will come from the yaw of the contacts.
    const Vector3 * tiltMeas = nullptr;
  };

  struct ContactInputData
  {
    ContactInputData(const Kinematics & bodyContactKine, double lambda)
    : bodyContactKine_(bodyContactKine), lambda_(lambda)
    {
    }

    // Kinematics contactBodyKine_;
    Kinematics bodyContactKine_;
    double lambda_;
  };

  template<typename OnNewContactObserver = std::nullptr_t,
           typename OnMaintainedContactObserver = std::nullptr_t,
           typename OnRemovedContactObserver = std::nullptr_t,
           typename OnAddedContactObserver = std::nullptr_t>
  struct ContactUpdateFunctions
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

    explicit ContactUpdateFunctions() {}

    template<typename OnNewContactOther>
    ContactUpdateFunctions<OnNewContactOther,
                           OnMaintainedContactObserver,
                           OnRemovedContactObserver,
                           OnAddedContactObserver>
        onNewContact(OnNewContactOther & onNewContact)
    {
      auto out = ContactUpdateFunctions<OnNewContactOther, OnMaintainedContactObserver, OnRemovedContactObserver,
                                        OnAddedContactObserver>::fromOther(*this);

      out.onNewContactFn = &onNewContact;
      return out;
    }

    template<typename OnMaintainedContactOther>
    ContactUpdateFunctions<OnNewContactObserver,
                           OnMaintainedContactOther,
                           OnRemovedContactObserver,
                           OnAddedContactObserver>
        onMaintainedContact(OnMaintainedContactOther & onMaintainedContact)
    {
      auto out = ContactUpdateFunctions<OnNewContactObserver, OnMaintainedContactOther, OnRemovedContactObserver,
                                        OnAddedContactObserver>::fromOther(*this);
      out.onMaintainedContactFn = &onMaintainedContact;
      return out;
    }

    template<typename OnRemovedContactOther>
    ContactUpdateFunctions<OnNewContactObserver,
                           OnMaintainedContactObserver,
                           OnRemovedContactOther,
                           OnAddedContactObserver>
        onRemovedContact(OnRemovedContactOther & onRemovedContact)
    {
      auto out = ContactUpdateFunctions<OnNewContactObserver, OnMaintainedContactObserver, OnRemovedContactOther,
                                        OnAddedContactObserver>::fromOther(*this);
      out.onRemovedContactFn = &onRemovedContact;
      return out;
    }

    template<typename OnAddedContactOther>
    ContactUpdateFunctions<OnNewContactObserver,
                           OnMaintainedContactObserver,
                           OnRemovedContactObserver,
                           OnAddedContactOther>
        onAddedContact(OnAddedContactOther & onAddedontact)
    {
      auto out = ContactUpdateFunctions<OnNewContactObserver, OnMaintainedContactObserver, OnRemovedContactObserver,
                                        OnAddedContactOther>::fromOther(*this);
      out.onAddedContactFn = &onAddedontact;
      return out;
    }

    template<typename OnNewContactOther,
             typename OnMaintainedContactOther,
             typename OnRemovedContactOther,
             typename OnAddedContactOther>
    static ContactUpdateFunctions fromOther(const ContactUpdateFunctions<OnNewContactOther,
                                                                         OnMaintainedContactOther,
                                                                         OnRemovedContactOther,
                                                                         OnAddedContactOther> & other)
    {
      ContactUpdateFunctions out;
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
      odometryType_ = stringToOdometryType(odometryTypeString);
      BOOST_ASSERT_MSG(
          odometryType_ == OdometryType::Flat || odometryType_ == OdometryType::Odometry6d,
          "Odometry type not allowed. Please pick among : [Odometry6d, Flat] or use the other Configuration "
          "constructor for an estimator that can run without odometry.");
    }

    /// @brief Configuration's constructor
    /// @details This versions allows to initialize the type of odometry directly with an OdometryType object.
    inline Configuration(OdometryType odometryType) noexcept : odometryType_(odometryType) {}

    // Desired kind of odometry (6D or flat)
    OdometryType odometryType_;

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
  inline const std::vector<LoContact *> & maintainedContacts() const
  {
    return maintainedContacts_;
  }

  /// @brief Initializer for the odometry manager.
  /// @param odomConfig Desired configuration of the odometry
  /// @param initPose Initial pose of the body
  void init(const Configuration & odomConfig, const Vector7 & initPose);

  /// @brief Function that initializes the loop of the legged odometry. To be called at the beginning of each iteration.
  /// @details Updates the the contacts, and sets the velocity and acceleration of the odometry if necessary.
  /// @param latestContactList List of every currently set contacts.
  /// @param updateFunctions Functions used when updating the contacts.
  /// @param linVel linear velocity of the body in the world.
  /// @param angVel angular velocity of the body in the world.
  template<typename OnNewContactObserver = std::nullptr_t,
           typename OnMaintainedContactObserver = std::nullptr_t,
           typename OnRemovedContactObserver = std::nullptr_t,
           typename OnAddedContactObserver = std::nullptr_t>
  bool initLoop(const std::unordered_set<std::string> & latestContactList,
                const ContactUpdateFunctions<OnNewContactObserver,
                                             OnMaintainedContactObserver,
                                             OnRemovedContactObserver,
                                             OnAddedContactObserver> & updateFunctions,
                const Vector3 * linVel = nullptr,
                const Vector3 * angVel = nullptr);

  /// @brief Function that runs the legged odometry loop. Using the input orientation and the current contacts, it
  /// estimates the new state of the robot.
  /// @param kineParams Kinematic parameters necessary for the estimation, either inputs or outputs to modify (please
  /// see the documentation of the KineParams class).
  void run(KineParams & kineParams);

  /// @brief Replaces the current pose of the odometry robot (that of the body used for the odometry !) by the given
  /// one.
  /// @details Also changes the reference pose of the contacts. Updates the velocity and acceleration with the new
  /// orientation if required.
  /// @param newPose New pose of the odometry robot.
  void replaceOdomBodyPose(const Vector7 & newPose);

  /// @brief Gives the kinematics (position and linear velocity) of the anchor point in the desired frame.
  /// @details If the velocity of the target frame in the world frame is given, the velocity of the anchor point in the
  /// target frame will also be contained in the returned Kinematics object.
  /// @param bodyTargetKine Kinematics of the target frame in the body frame.
  Kinematics getAnchorKineIn(Kinematics & bodyTargetKine);

  /// @brief Gives the kinematics (position and linear velocity) of the anchor point in the body frame.
  /// @param withVel Indicates if the velocity of the anchor point in the body must be computed.
  Kinematics getAnchorKineInBody(bool withVel);

  /**
   * @brief Returns the position of the anchor point in the world from the current contacts reference position.
   *
   * @return Vector3&
   */
  const Vector3 & getWorldRefAnchorPos();

  /// @brief Updates the kinematics of the body in the world.
  /// @details For each maintained contact, we compute the position of the body in the contact frame, we
  /// then compute their weighted average and obtain the estimated translation from the anchor point to the body.  We
  /// apply this translation to the reference position of the anchor frame in the world to obtain the new position of
  /// the body in the word. We do the same for the orientation.
  Kinematics getWorldBodyKineFromAnchor(bool withPos, bool withOri);

  /// @brief Updates the local kinematics of the body in the world.
  /// @details For each maintained contact, we compute the position of the body in the contact frame, we
  /// then compute their weighted average and obtain the estimated translation from the anchor point to the body.  We
  /// apply this translation to the reference position of the anchor frame in the world to obtain the new position of
  /// the body in the word. We do the same for the orientation.
  LocalKinematics getWorldBodyLocalKineFromAnchor();

  /// @brief Changes the type of the odometry
  /// @details Version meant to be called by the observer using the odometry during the run through the gui.
  /// @param newOdometryType The string naming the new type of odometry to use.
  void setOdometryType(OdometryType newOdometryType);

  void setSamplingTime(double dt)
  {
    ctl_dt_ = dt;
  }

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
  /// @brief Updates the contacts
  /// @param latestContactList List of all currently set contacts.
  /// @param runParams Parameters used to run the legged odometry.
  template<typename OnNewContactObserver = std::nullptr_t,
           typename OnMaintainedContactObserver = std::nullptr_t,
           typename OnRemovedContactObserver = std::nullptr_t,
           typename OnAddedContactObserver = std::nullptr_t>
  bool updateContacts(const std::unordered_set<std::string> & latestContactList,
                      const ContactUpdateFunctions<OnNewContactObserver,
                                                   OnMaintainedContactObserver,
                                                   OnRemovedContactObserver,
                                                   OnAddedContactObserver> & updateFunctions);

  /// @brief Updates the body pose given as argument by the observer.
  /// @param pose The pose of the body in the world that we want to update
  void updateBodyKinematicsPvt(Kinematics & pose);

  /// @brief Estimates the body from the currently set contacts and updates them.
  /// @param runParams Parameters used to run the legged odometry.
  void updateBodyAndContacts(const KineParams & params);

  /// @brief Corrects the reference pose of the contacts after the update of the body.
  /// @details The new reference pose is obtained by forward kinematics from the updated body.
  void correctContactsRef();

  /// @brief Computes the reference kinematics of the newly set contact in the world.
  /// @param contact The new contact
  void setNewContact(LoContact & contact);

  /// @brief Computes the kinematics of the contact attached to the odometry robot in the world frame from the current
  /// body pose and encoders.
  /// @param contact Contact of which we want to compute the kinematics.
  /// @return Kinematics &.
  const Kinematics & getContactKinematics(LoContact & contact);

  /// @brief Gives the kinematics of the contact in the desired frame.
  /// @details If the velocity of the target frame in the world frame is given, the velocity of the anchor point in the
  /// target frame will also be contained in the returned Kinematics object.
  /// @param bodyTargetKine Kinematics of the target frame in the body frame.
  Kinematics getContactKineIn(LoContact & contact, Kinematics & bodyTargetKine);

  /// @brief Selects which contacts to use for the orientation odometry and computes the orientation of the body
  ///  for each of them
  /// @details The two contacts with the highest lambda are selected.
  /// @param sumLambdasOrientation Sum of the lambdas of the contacts used for the orientation estimation
  double selectForOrientationOdometry();

protected:
  // category to plot the odometry in
  std::string category_;

  // contacts manager used by this odometry manager
  LeggedOdometryContactsManager contactsManager_;

public:
  // kinematics of the tracked body
  Kinematics bodyKine_;

protected:
  // contacts created on the current iteration
  std::vector<LoContact *> newContacts_;
  // contacts maintained during the current iteration
  std::vector<LoContact *> maintainedContacts_;
  // time constant defining how fast the contact reference poses are corrected by the one of the body
  double kappa_ = 1 / (2 * M_PI);
  // gain allowing for the contribution of the contact pose measurement into the reference pose even after a long
  // contact's lifetime.
  double lambdaInf_ = 0.02;
  // timestep used in the controller
  double ctl_dt_ = 0.0;

  // indicates whether we want to update the yaw using this method or not
  bool withYawEstimation_;
  // Indicates if the reference pose of the contacts must be corrected at the end of each iteration.
  bool correctContacts_ = true;

  // position of the anchor point of the robot in the world, obtained from the contact references.
  Vector3 worldRefAnchorPosition_;
  // position of the anchor point in the frame of the body.
  Vector3 bodyAnchorPos_;

  // Indicates if the previous anchor point was obtained using contacts
  bool prevAnchorFromContacts_ = true;
  // Indicates if the current anchor point was obtained using contacts
  bool currAnchorFromContacts_ = true;
  // indicated if the position can be updated using contacts. True if a contact is currently set.
  bool posUpdatable_ = false;

  // time stamp, incremented on the intiialization of each iteration.
  TimeIndex k_iter_ = 0;
  // time stamp, incremented once the reading of the joint encoders and the contacts are updated
  TimeIndex k_data_ = 0;
  // time stamp, incremented once the kinematics of the odometry robot have been updated.
  TimeIndex k_est_ = 0;
  // time stamp, incremented once the contact references have been corrected.
  TimeIndex k_correct_ = 0;
  // time stamp, incremented once the anchor frame has been computed.
  TimeIndex k_anchor_ = 0;

public:
  // Indicates if the desired odometry must be a flat or a 6D odometry.
  OdometryType odometryType_ = OdometryType::None;
};

} // namespace odometry
} // namespace stateObservation

#include <state-observation/tools/odometry/legged-odometry-manager.hxx>

#endif // LEGGEDODOMETRYMANAGERHPP
