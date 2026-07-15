#include <unordered_set>
namespace stateObservation
{
namespace odometry
{
template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
bool LeggedOdometryManager::initLoop(const std::unordered_set<std::string> & latestContactList,
                                     const ContactUpdateFunctions<OnNewContactObserver,
                                                                  OnMaintainedContactObserver,
                                                                  OnRemovedContactObserver,
                                                                  OnAddedContactObserver> & updateFunctions,
                                     const Vector3 * linVel,
                                     const Vector3 * angVel)
{
  k_iter_++;
  bool contactsChanged = updateContacts(latestContactList, updateFunctions);

  if(linVel != nullptr)
  {
    bodyKine_.linVel = *linVel;
  }
  if(angVel != nullptr)
  {
    bodyKine_.angVel = *angVel;
  }
  k_data_ = k_iter_;

  return contactsChanged;
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
bool LeggedOdometryManager::updateContacts(const std::unordered_set<std::string> & latestContactList,
                                           const ContactUpdateFunctions<OnNewContactObserver,
                                                                        OnMaintainedContactObserver,
                                                                        OnRemovedContactObserver,
                                                                        OnAddedContactObserver> & updateFunctions)
{
  // If the position and orientation of the body can be updated using contacts (that were already set on the
  // previous iteration), they are updated, else we keep the previous estimation. Then we estimate the pose of new
  // contacts using the obtained pose of the body.
  double sumLambdas_position = 0.0;
  posUpdatable_ = false;
  newContacts_.clear();
  maintainedContacts_.clear();
  bool contactsChanged = false;

  auto onNewContact = [this, &contactsChanged, &updateFunctions](LoContact & newContact)
  {
    newContacts_.push_back(&newContact);
    if constexpr(!std::is_same_v<OnNewContactObserver, std::nullptr_t>)
    {
      (*updateFunctions.onNewContactFn)(newContact);
    }
    contactsChanged = true;
  };

  auto onMaintainedContact = [this, &updateFunctions, &sumLambdas_position](LoContact & maintainedContact)
  {
    maintainedContacts_.push_back(&maintainedContact);
    maintainedContact.lifeTimeIncrement(ctl_dt_);

    if constexpr(!std::is_same_v<OnMaintainedContactObserver, std::nullptr_t>)
    {
      (*updateFunctions.onMaintainedContactFn)(maintainedContact);
    }

    maintainedContact.worldBodyKineFromRef_ =
        maintainedContact.worldRefKine_ * maintainedContact.bodyContactKine_.getInverse();

    sumLambdas_position += maintainedContact.lambda();
    posUpdatable_ = true;
  };

  auto onRemovedContact = [this, &contactsChanged, &updateFunctions](LoContact & removedContact)
  {
    if constexpr(!std::is_same_v<OnRemovedContactObserver, std::nullptr_t>)
    {
      (*updateFunctions.onRemovedContactFn)(removedContact);
    }
    contactsChanged = true;
  };

  // detects the contacts currently set with the environment
  contactsManager().updateContacts(latestContactList, onNewContact, onMaintainedContact, onRemovedContact,
                                   *updateFunctions.onAddedContactFn);

  for(auto * mContact : maintainedContacts_)
  {
    mContact->lambda(mContact->lambda() / sumLambdas_position);
  }
  return contactsChanged;
}

} // namespace odometry
} // namespace stateObservation