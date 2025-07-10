#ifndef CONTACTSMANAGERHPP
#define CONTACTSMANAGERHPP
#include <state-observation/api.h>
#include <state-observation/tools/measurements-manager/Contact.hpp>
#include <unordered_set>

namespace stateObservation
{
namespace measurements
{
/// @brief Structure that implements all the necessary functions to manage the map of contacts. Handles their detection
/// and updates the list of the detected contacts, newly removed contacts, etc., to apply the appropriate functions on
/// them.
/// @details The template allows to define other kinds of contacts and thus add custom parameters to them.
/// @tparam ContactT Contact, associated to a sensor.
template<typename ContactT>
struct STATE_OBSERVATION_DLLAPI ContactsManager
{

protected:
  /// @brief Insert a contact to the map of contacts.
  /// @param name The name of the contact
  /// @param onAddedContact function to call when a contact is added to the manager
  /// @return ContactT &
  template<typename OnAddedContact = std::nullptr_t>
  ContactT & addContactToManager(const std::string & name, OnAddedContact onAddedContact = nullptr);

public:
  /// @brief Updates the list of contacts to inform whether they are newly
  /// set, removed, etc., and execute actions accordingly
  /// @param latestContactList Set containing the names of all contacts detected on the latest iteration.
  /// @param onNewContact Function to call on a newly detected contact
  /// @param onMaintainedContact Function to call on a contact maintainted
  /// since the last iteration
  /// @param onRemovedContact Function to call on a removed contact
  /// @param onAddedContact function to call when a contact is added to the
  /// manager
  /// @return void
  template<typename OnNewContact,
           typename OnMaintainedContact,
           typename OnRemovedContact,
           typename OnAddedContact = std::nullptr_t>
  void updateContacts(const std::unordered_set<std::string> & latestContactList,
                      OnNewContact onNewContact,
                      OnMaintainedContact onMaintainedContact,
                      OnRemovedContact onRemovedContact,
                      OnAddedContact onAddedContact = nullptr);

  /// @brief Get the map of all the contacts
  ///
  /// @return std::unordered_map<std::string, contactsWithSensorT>&
  inline std::unordered_map<std::string, ContactT> & contacts()
  {
    return listContacts_;
  }

  /** Returns true if any contact is detected */
  inline bool contactsDetected() const noexcept
  {
    return contactsDetected_;
  }

  /// @brief Accessor for the a contact associated to a sensor contained in
  /// the map
  /// @details Returns a null pointer if the contact does not exist
  /// @param name The name of the contact to access
  /// @return ContactT *
  ContactT * findContact(const std::string & name)
  {
    auto it = listContacts_.find(name);
    if(it != listContacts_.end())
    {
      return &(it->second);
    }
    return nullptr;
  }

protected:
  // map of contacts used by the manager. unordered map containing all the contacts, currently set or not.
  std::unordered_map<std::string, ContactT> listContacts_;

  // vector containing the name of all the currently set contacts
  std::unordered_set<std::string> currentContactsList_;

  // Index generator, incremented everytime a new contact is created
  unsigned idx_ = 0;
  /** True if any contact is detected, false otherwise */
  bool contactsDetected_ = false;
};
} // namespace measurements
} // namespace stateObservation

#include <state-observation/tools/measurements-manager/ContactsManager.hxx>

#endif // CONTACTSMANAGERHPP