#pragma once
#include <string>
namespace stateObservation::measurements
{

/**
 * Object making easier the handling of sensors within the observers.
 **/

/// @brief Class containing the information of a sensor to facilitate its handling.
struct Sensor
{

protected:
  inline Sensor() = default;
  inline Sensor(int id, std::string_view name) : id_(id), name_(name) {}

  inline bool operator<(const Sensor & rhs) const noexcept
  {
    return (id() < rhs.id_);
  }

public:
  inline int id() const noexcept
  {
    return id_;
  }
  inline const std::string & name() const noexcept
  {
    return name_;
  }

protected:
  int id_;
  std::string name_;
};
} // namespace stateObservation::measurements
