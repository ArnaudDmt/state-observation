#ifndef MEASUREMENTSHPP
#define MEASUREMENTSHPP
#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/measurements-manager/IMU.hpp>

namespace stateObservation
{
namespace measurements
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
inline static std::string odometryTypeToSstring(OdometryType odometryType)
{
  return internal::odometryTypToStr_.at(odometryType);
}

// IMUs can be handled using only a vector containing the IMU objects.
typedef std::vector<IMU> ImuList;
} // namespace measurements
} // namespace stateObservation
#endif // MEASUREMENTSHPP