# Copyright 2008, 2009, 2010, 2013 Olivier Stasse, Mehdi Benallegue JRL, CNRS/AIST
#
# This file is part of state-observation.
# state-observation is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# state-observation is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with state-observation.  If not, see <http://www.gnu.org/licenses/>.

add_library(state-observation SHARED)
target_compile_features(state-observation PUBLIC cxx_std_17)
target_sources(
  state-observation
  PRIVATE tilt-estimator.cpp
          tilt-estimator-humanoid.cpp
          kalman-filter-base.cpp
          observer-base.cpp
          extended-kalman-filter.cpp
          linear-kalman-filter.cpp
          zero-delay-observer.cpp
          sensor-base.cpp
          algebraic-sensor.cpp
          rotation-velocity.cpp
          linear-acceleration.cpp
          magnetic-field.cpp
          definitions.cpp
          logger.cpp
          state-vector-arithmetics.cpp
          accelerometer-gyrometer.cpp
          accelerometer-gyrometer-magnetometer.cpp
          probability-law-simulation.cpp
          gaussian-white-noise.cpp
          dynamical-system-functor-base.cpp
          dynamical-system-simulator.cpp
          imu-dynamical-system.cpp
          imu-mltpctive-dynamical-system.cpp
          imu-magnetometer-dynamical-system.cpp
          flexibility-estimator-base.cpp
          ekf-flexibility-estimator-base.cpp
          fixed-contact-ekf-flex-estimator-imu.cpp
          model-base-ekf-flex-estimator-imu.cpp
          imu-fixed-contact-dynamical-system.cpp
          stable-imu-fixed-contact-dynamical-system.cpp
          imu-elastic-local-frame-dynamical-system.cpp
          bidim-elastic-inv-pendulum-dyn-sys.cpp
          kinetics-observer.cpp
          lipm-dcm-estimator.cpp
          unidim-lipm-dcm-estimator.cpp
          zmp-tracking-gain-estimator.cpp)

jrl_target_headers(
  state-observation
  PUBLIC
  HEADERS
  ../include/state-observation/observer/tilt-estimator.hpp
  ../include/state-observation/observer/tilt-estimator-humanoid.hpp
  ../include/state-observation/observer/extended-kalman-filter.hpp
  ../include/state-observation/observer/kalman-filter-base.hpp
  ../include/state-observation/observer/linear-kalman-filter.hpp
  ../include/state-observation/observer/observer-base.hpp
  ../include/state-observation/observer/zero-delay-observer.hpp
  ../include/state-observation/sensors-simulation/sensor-base.hpp
  ../include/state-observation/sensors-simulation/algebraic-sensor.hpp
  ../include/state-observation/sensors-simulation/accelerometer-gyrometer.hpp
  ../include/state-observation/sensors-simulation/accelerometer-gyrometer-magnetometer.hpp
  ../include/state-observation/noise/noise-base.hpp
  ../include/state-observation/noise/gaussian-white-noise.hpp
  ../include/state-observation/sensors-simulation/algorithm/rotation-velocity.hpp
  ../include/state-observation/sensors-simulation/algorithm/linear-acceleration.hpp
  ../include/state-observation/sensors-simulation/algorithm/magnetic-field.hpp
  ../include/state-observation/tools/definitions.hpp
  ../include/state-observation/tools/definitions.hxx
  ../include/state-observation/tools/hrp2.hpp
  ../include/state-observation/tools/probability-law-simulation.hpp
  ../include/state-observation/tools/probability-law-simulation.hxx
  ../include/state-observation/tools/miscellaneous-algorithms.hpp
  ../include/state-observation/tools/rigid-body-kinematics.hpp
  ../include/state-observation/tools/rigid-body-kinematics.hxx
  ../include/state-observation/tools/logger.hpp
  ../include/state-observation/tools/logger.hxx
  ../include/state-observation/tools/state-vector-arithmetics.hpp
  ../include/state-observation/dynamical-system/dynamical-system-functor-base.hpp
  ../include/state-observation/dynamical-system/dynamical-system-simulator.hpp
  ../include/state-observation/dynamical-system/imu-dynamical-system.hpp
  ../include/state-observation/dynamical-system/imu-mltpctive-dynamical-system.hpp
  ../include/state-observation/dynamical-system/imu-magnetometer-dynamical-system.hpp
  ../include/state-observation/dynamical-system/bidim-elastic-inv-pendulum-dyn-sys.hpp
  ../include/state-observation/flexibility-estimation/flexibility-estimator-base.hpp
  ../include/state-observation/flexibility-estimation/ekf-flexibility-estimator-base.hpp
  ../include/state-observation/flexibility-estimation/fixed-contact-ekf-flex-estimator-imu.hpp
  ../include/state-observation/flexibility-estimation/model-base-ekf-flex-estimator-imu.hpp
  ../include/state-observation/flexibility-estimation/imu-fixed-contact-dynamical-system.hpp
  ../include/state-observation/flexibility-estimation/stable-imu-fixed-contact-dynamical-system.hpp
  ../include/state-observation/flexibility-estimation/imu-elastic-local-frame-dynamical-system.hpp
  ../include/state-observation/dynamics-estimators/kinetics-observer.hpp
  ../include/state-observation/dynamics-estimators/kinetics-observer.hxx
  ../include/state-observation/dynamics-estimators/lipm-dcm-estimator.hpp
  ../include/state-observation/dynamics-estimators/unidim-lipm-dcm-estimator.hpp
  ../include/state-observation/dynamics-estimators/zmp-tracking-gain-estimator.hpp
  ../include/state-observation/examples/imu-attitude-trajectory-reconstruction.hpp
  ../include/state-observation/examples/imu-attitude-trajectory-reconstruction.hxx
  ../include/state-observation/examples/imu-multiplicative-attitude-reconstruction.hpp
  ../include/state-observation/examples/imu-multiplicative-attitude-reconstruction.hxx
  ../include/state-observation/examples/offline-ekf-flexibility-estimation.hpp
  ../include/state-observation/examples/offline-ekf-flexibility-estimation.hxx
  ../include/state-observation/examples/offline-model-base-flex-estimation.hpp
  ../include/state-observation/examples/offline-model-base-flex-estimation.hxx
  BASE_DIRS
  ../include)

target_link_libraries(state-observation PUBLIC Boost::timer Boost::disable_autolinking)
target_link_libraries(state-observation PUBLIC Eigen3::Eigen)
jrl_target_generate_config_header(state-observation PUBLIC VERSION ${PROJECT_VERSION} FILENAME state-observation/api.h)
target_include_directories(
  state-observation
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/$<CONFIG>/include>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(state-observation PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})

jrl_add_export_component(NAME state-observation TARGETS state-observation)
