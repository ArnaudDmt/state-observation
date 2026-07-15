#ifndef CONTACTHPP
#define CONTACTHPP
#include <boost/assert.hpp>
#include <Eigen/Core>
#include <string>
#include <string_view>

namespace stateObservation::measurements
{
/**
 * Object making easier the handling of contacts within the observers.
 **/

struct Contact
{
  // constructor if the contact is not associated to a surface
  inline Contact(unsigned id, std::string_view surfaceName) : id_(id), surface_(surfaceName) {}

protected:
  inline Contact() = default;
  inline bool operator<(const Contact & rhs) const noexcept
  {
    return (id() < rhs.id_);
  }

public:
  virtual void resetContact() noexcept
  {
    wasAlreadySet_ = false;
    isSet_ = false;
  }

  inline unsigned id() const noexcept
  {
    return id_;
  }

  inline const std::string & surfaceName() const noexcept
  {
    return surface_;
  }
  inline bool isSet() const noexcept
  {
    return isSet_;
  }
  inline bool wasAlreadySet() const noexcept
  {
    return wasAlreadySet_;
  }

  inline void isSet(bool isSet)
  {
    isSet_ = isSet;
  }
  inline void wasAlreadySet(bool wasAlreadySet)
  {
    wasAlreadySet_ = wasAlreadySet;
  }

protected:
  unsigned id_;

  bool isSet_ = false;
  bool wasAlreadySet_ = false;
  std::string surface_;
};
} // namespace stateObservation::measurements

#endif // CONTACTHPP