namespace stateObservation
{
namespace measurements
{
///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

template<typename ContactT>
template<typename OnNewContact, typename OnMaintainedContact, typename OnRemovedContact, typename OnAddedContact>
void ContactsManager<ContactT>::updateContacts(const std::unordered_set<std::string> & latestContactList,
                                               OnNewContact onNewContact,
                                               OnMaintainedContact onMaintainedContact,
                                               OnRemovedContact onRemovedContact,
                                               OnAddedContact onAddedContact)
{
  std::string removed_contact_set;
  std::string new_contact_set;

  std::vector<std::string> orderedContactList(latestContactList.begin(), latestContactList.end());
  if(contactIds_.empty())
  {
    std::sort(orderedContactList.begin(), orderedContactList.end());
  }
  else
  {
    for(const auto & contactName : orderedContactList)
    {
      if(contactIds_.find(contactName) == contactIds_.end())
      {
        throw std::invalid_argument("contact is missing from the configured contact order: " + contactName);
      }
    }
    std::sort(orderedContactList.begin(), orderedContactList.end(), [this](const auto & lhs, const auto & rhs)
              { return contactIds_.at(lhs) < contactIds_.at(rhs); });
  }

  for(const auto & contactName : orderedContactList)
  {
    ContactT & contact = addContactToManager(contactName, onAddedContact);
    contactsDetected_ = true;
    contact.wasAlreadySet(contact.isSet());
    contact.isSet(true);

    if(currentContactsList_.find(contactName) != currentContactsList_.end())
    {
      onMaintainedContact(contact);
    }
    else
    {
      onNewContact(contact);
      if(!new_contact_set.empty())
      {
        new_contact_set += ", ";
      }
      new_contact_set += contactName;
    }
  }
  for(auto & prevContactName : currentContactsList_)
  {
    if(latestContactList.find(prevContactName) == latestContactList.end())
    {
      ContactT & removedContact = *findContact(prevContactName);
      removedContact.isSet(false);
      onRemovedContact(removedContact);
      if(!removed_contact_set.empty())
      {
        removed_contact_set += ", ";
      }
      removed_contact_set += prevContactName;
    }
  }

  currentContactsList_ = latestContactList;
}

template<typename ContactT>
template<typename OnAddedContact>
inline ContactT & ContactsManager<ContactT>::addContactToManager(const std::string & name,
                                                                 [[maybe_unused]] OnAddedContact onAddedContact)
{
  const unsigned id = contactIds_.empty() ? idx_ : contactIds_.at(name);
  const auto [it, inserted] = listContacts_.insert({name, ContactT(id, name)});

  ContactT & contact = (*it).second;
  if(!inserted)
  {
    return contact;
  }

  if constexpr(!std::is_same_v<OnAddedContact, std::nullptr_t>)
  {
    onAddedContact(contact);
  }
  if(contactIds_.empty()) { idx_++; }

  return contact;
}
} // namespace measurements
} // namespace stateObservation
