#include "state-observation/tools/rigid-body-kinematics.hpp"
#include <iostream>
#include <state-observation/observer/viking.hpp>

using Vector3 = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;

using namespace stateObservation;

struct Measurement
{
  double timestamp_sec;

  // Pose
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;

  // Linear velocity (local frame)
  Eigen::Vector3d linear_velocity;

  // IMU
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d acceleration;

  // Orientation measurement from external IMU
  Eigen::Quaterniond ori_meas_ext;
};

Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw)
{
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q.normalized();
}

std::vector<double> loadTimestamps(const std::string & path)
{
  std::ifstream file(path);
  std::string line;
  std::vector<double> times;

  while(std::getline(file, line))
  {
    times.push_back(std::stod(line) / 1000.0); // ms → s
  }
  return times;
}

template<typename T>
std::vector<T> loadVecs(const std::string & path, int cols)
{
  std::ifstream file(path);
  std::string line;
  std::vector<T> data;

  while(std::getline(file, line))
  {
    std::istringstream ss(line);
    T vec;
    for(int i = 0; i < cols; ++i)
    {
      ss >> vec[i];
    }
    data.push_back(vec);
  }
  return data;
}

int main()
{
  auto timestamps = loadTimestamps("times.txt");
  auto poses = loadVecs<Vector3>("poses.txt", 6); // x y z roll pitch yaw
  auto imu_int = loadVecs<Vector3>("IMUint.txt", 9); // gyro (3), lin_vel (3), accel (3)
  auto imu_ext = loadVecs<Vector3>("IMUext.txt", 3); // external orientation rotX rotY rotZ

  size_t N = timestamps.size();
  double dt = 0.01; // Compute from timestamps if necessary
  Viking viking(dt, 1, 1, 1, 3, 1, true);

  kine::Orientation initOri(imu_ext[0](0), imu_ext[0](1), imu_ext[0](2));

  // Initialize with first values
  viking.initEstimator(imu_int[0].segment<3>(3), initOri.toMatrix3().transpose() * Vector3::UnitZ(), Vector3::Zero(),
                       initOri.toVector4(), poses[0].head<3>());

  for(size_t i = 0; i < N; ++i)
  {
    double t = timestamps[i];
    Vector3 pos = poses[i].head<3>();
    Vector3 vel = imu_int[i].segment<3>(3);
    Vector3 acc = imu_int[i].segment<3>(6);
    Vector3 gyro = imu_int[i].head<3>();
    kine::Orientation ori(imu_ext[i](0), imu_ext[i](1), imu_ext[i](2));

    viking.setInput(vel, acc, gyro, i);

    double delay = 0.0004;
    if(i % 2 == 0 && (t - delay) > 0)
    {
      // Naive match
      size_t delayed_index = std::lower_bound(timestamps.begin(), timestamps.end(), t - delay) - timestamps.begin();
      if(delayed_index < N)
      {
        viking.addDelayedPosOriMeasurement(poses[delayed_index].head<3>(), rotVecToQuat(poses[delayed_index].tail<3>()),
                                           1, 1, 1, 1, delay);
      }
    }

    viking.getEstimatedState(i + 1);
  }

  std::cout << "Estimation complete." << std::endl;
  return 0;
}