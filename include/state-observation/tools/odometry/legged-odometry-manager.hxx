#include <unordered_set>
namespace stateObservation
{
namespace odometry
{
template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::initLoop(const std::unordered_set<std::string> & latestContactList,
                                     const ContactUpdateFunctions<OnNewContactObserver,
                                                                  OnMaintainedContactObserver,
                                                                  OnRemovedContactObserver,
                                                                  OnAddedContactObserver> & updateFunctions,
                                     const Vector3 * linVel,
                                     const Vector3 * angVel)
{
  k_iter_++;
  updateContacts(latestContactList, updateFunctions);

  if(linVel != nullptr)
  {
    fbKine_.linVel = *linVel;
  }
  if(angVel != nullptr)
  {
    fbKine_.angVel = *angVel;
  }
  k_data_ = k_iter_;
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::updateContacts(const std::unordered_set<std::string> & latestContactList,
                                           const ContactUpdateFunctions<OnNewContactObserver,
                                                                        OnMaintainedContactObserver,
                                                                        OnRemovedContactObserver,
                                                                        OnAddedContactObserver> & updateFunctions)
{
  // If the position and orientation of the floating base can be updated using contacts (that were already set on the
  // previous iteration), they are updated, else we keep the previous estimation. Then we estimate the pose of new
  // contacts using the obtained pose of the floating base.
  double sumLambdas_position = 0.0;
  posUpdatable_ = false;
  newContacts_.clear();
  maintainedContacts_.clear();

  auto onNewContact = [this, &updateFunctions](LoContact & newContact)
  {
    newContacts_.push_back(&newContact);
    if constexpr(!std::is_same_v<OnNewContactObserver, std::nullptr_t>)
    {
      (*updateFunctions.onNewContactFn)(newContact);
    }
  };

  auto onMaintainedContact = [this, &updateFunctions, &sumLambdas_position](LoContact & maintainedContact)
  {
    maintainedContacts_.push_back(&maintainedContact);
    maintainedContact.lifeTimeIncrement(ctl_dt_);

    maintainedContact.worldFbKineFromRef_ = maintainedContact.worldRefKine_ * maintainedContact.contactFbKine_;

    if constexpr(!std::is_same_v<OnMaintainedContactObserver, std::nullptr_t>)
    {
      (*updateFunctions.onMaintainedContactFn)(maintainedContact);
    }

    sumLambdas_position += maintainedContact.lambda();
    posUpdatable_ = true;
  };

  auto onRemovedContact = [this, &updateFunctions](LoContact & removedContact)
  {
    if constexpr(!std::is_same_v<OnRemovedContactObserver, std::nullptr_t>)
    {
      (*updateFunctions.onRemovedContactFn)(removedContact);
    }
  };

  // detects the contacts currently set with the environment
  contactsManager().updateContacts(latestContactList, onNewContact, onMaintainedContact, onRemovedContact,
                                   *updateFunctions.onAddedContactFn);

  for(auto * mContact : maintainedContacts_)
  {
    mContact->lambda(mContact->lambda() / sumLambdas_position);
  }
}

} // namespace odometry
} // namespace stateObservation