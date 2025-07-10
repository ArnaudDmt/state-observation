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

  for(auto & contactName : latestContactList)
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
  for(auto prevContactName : currentContactsList_)
  {
    if(currentContactsList_.find(prevContactName) == currentContactsList_.end())
    {
      onRemovedContact(*findContact(prevContactName));
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
  const auto [it, inserted] = listContacts_.insert({name, ContactT(idx_, name)});

  ContactT & contact = (*it).second;
  if(!inserted)
  {
    return contact;
  }

  if constexpr(!std::is_same_v<OnAddedContact, std::nullptr_t>)
  {
    onAddedContact(contact);
  }
  idx_++;

  return contact;
}
} // namespace measurements
} // namespace stateObservation
