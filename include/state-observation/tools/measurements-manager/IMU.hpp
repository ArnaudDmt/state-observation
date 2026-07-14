#ifndef IMUHPP
#define IMUHPP
#include <state-observation/tools/measurements-manager/Sensor.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace stateObservation
{
namespace measurements
{
/**
 * Object making easier the handling of sensors within the observers.
 **/

/// @brief Class containing the information of an IMU.
struct IMU : public Sensor
{
public:
  inline IMU(size_t id, std::string_view name) : Sensor(id, name) {}

public:
  Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero();
  stateObservation::kine::Kinematics bodyImuKine;
};
} // namespace measurements
} // namespace stateObservation
#endif // IMUHPP