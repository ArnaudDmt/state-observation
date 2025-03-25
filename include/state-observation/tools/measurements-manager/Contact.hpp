#pragma once
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
protected:
  inline Contact() = default;
  // constructor if the contact is not associated to a surface
  inline Contact(unsigned id, std::string_view name) : id_(id), name_(name) {}
  // constructor if the contact is associated to a surface
  inline Contact(unsigned id, std::string_view name, std::string_view surface) : Contact(id, name)
  {
    setSurface(surface);
  }
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
  inline const std::string & name() const noexcept
  {
    return name_;
  }
  inline bool isSet() const noexcept
  {
    return isSet_;
  }
  inline bool wasAlreadySet() const noexcept
  {
    return wasAlreadySet_;
  }
  inline const std::string & surface() const
  {
    BOOST_ASSERT(!surface_.empty() && "The contact was created without a surface.");
    return surface_;
  }

  inline void setSurface(std::string_view surfaceName)
  {
    surface_ = surfaceName;
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
  std::string name_;

  bool isSet_ = false;
  bool wasAlreadySet_ = false;
  std::string surface_;
};
} // namespace stateObservation::measurements
