#include <unordered_set>
namespace stateObservation
{
namespace odometry
{
template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::initLoop(const ContactUpdateParameters<OnNewContactObserver,
                                                                   OnMaintainedContactObserver,
                                                                   OnRemovedContactObserver,
                                                                   OnAddedContactObserver> & contactParams)
{
  k_iter_++;

  updateContacts(contactParams);

  k_data_ = k_iter_;
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::updateContacts(const ContactUpdateParameters<OnNewContactObserver,
                                                                         OnMaintainedContactObserver,
                                                                         OnRemovedContactObserver,
                                                                         OnAddedContactObserver> & contactParams)
{
  // If the position and orientation of the floating base can be updated using contacts (that were already set on the
  // previous iteration), they are updated, else we keep the previous estimation. Then we estimate the pose of new
  // contacts using the obtained pose of the floating base.

  double sumLambdas_position = 0.0;
  posUpdatable_ = false;
  newContacts_.clear();
  maintainedContacts_.clear();

  auto onNewContact = [this, &contactParams](LoContact & newContact)
  {
    newContacts_.push_back(&newContact);
    newContact.contactFbKine_ = contactParams.contactData_.at(newContact.name()).contactFbKine_;
    newContact.lambda(contactParams.contactData_.at(newContact.name()).lambda_);
    if constexpr(!std::is_same_v<OnNewContactObserver, std::nullptr_t>)
    {
      (*contactParams.onNewContactFn)(newContact);
    }
  };

  auto onMaintainedContact = [this, &contactParams, &sumLambdas_position](LoContact & maintainedContact)
  {
    maintainedContacts_.push_back(&maintainedContact);
    maintainedContact.contactFbKine_ = contactParams.contactData_.at(maintainedContact.name()).contactFbKine_;
    maintainedContact.lambda(contactParams.contactData_.at(maintainedContact.name()).lambda_);
    maintainedContact.lifeTimeIncrement(ctl_dt_);

    maintainedContact.worldFbKineFromRef_ = maintainedContact.worldRefKine_ * maintainedContact.contactFbKine_;

    if constexpr(!std::is_same_v<OnMaintainedContactObserver, std::nullptr_t>)
    {
      (*contactParams.onMaintainedContactFn)(maintainedContact);
    }

    sumLambdas_position += maintainedContact.lambda();
    posUpdatable_ = true;
  };

  auto onRemovedContact = [this, &contactParams](LoContact & removedContact)
  {
    if constexpr(!std::is_same_v<OnRemovedContactObserver, std::nullptr_t>)
    {
      (*contactParams.onRemovedContactFn)(removedContact);
    }
  };

  std::unordered_set<std::string> latestContactList;
  latestContactList.reserve(contactParams.contactData_.size());

  for(const auto & contactData : contactParams.contactData_)
  {
    latestContactList.insert(contactData.first);
  }
  // detects the contacts currently set with the environment
  contactsManager().updateContacts(latestContactList, onNewContact, onMaintainedContact, onRemovedContact,
                                   *contactParams.onAddedContactFn);

  for(auto * mContact : maintainedContacts_)
  {
    mContact->lambda(mContact->lambda() / sumLambdas_position);
  }
}

} // namespace odometry
} // namespace stateObservation