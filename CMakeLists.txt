# Copyright 2013-2026, CNRS
#
# Author: Mehdi Benallegue
# Author: Arnaud Tanguy
#
# This file is part of state-observation.
# state-observation is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# state-observation is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# state-observation. If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.22)
if(POLICY CMP0063)
  cmake_policy(SET CMP0063 NEW)
endif()

include(cmake/get-jrl-cmakemodules.cmake)

project(
  state-observation
  VERSION 1.7.0
  DESCRIPTION "General implementation of observers."
  HOMEPAGE_URL "https://github.com/jrl-umi3218/state-observation"
  LANGUAGES CXX)

jrl_configure_defaults()

set(CMAKE_CXX_VISIBILITY_PRESET hidden)

jrl_option(BUILD_STATE_OBSERVATION_BENCHMARKS "Build state-observation benchmarks" OFF)
jrl_option(BUILD_STATE_OBSERVATION_TOOLS "Build state-observation tools" ON)
jrl_option(BUILD_TESTING "Build and run unit tests" ON)
jrl_option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

if(INSTALL_DOCUMENTATION)
  jrl_find_package(Doxygen REQUIRED COMPONENTS dot)
endif()

# For MSVC, set local environment variable to enable finding the built dll
# of the main library when launching ctest with RUN_TESTS
if(MSVC)
  set(CMAKE_MSVCIDE_RUN_PATH "\$(SolutionDir)/src/\$(Configuration)")
endif(MSVC)

jrl_find_package(
  Boost
  REQUIRED
  COMPONENTS
  timer
  EXPECTED_TARGETS
  Boost::timer
  Boost::disable_autolinking)
jrl_find_package(Eigen3 REQUIRED EXPECTED_TARGETS Eigen3::Eigen)

set(TEST_COMPUTATION_TIME "OFF")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/include/state-observation/config.h.in"
               "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/include/state-observation/config.h.in" @ONLY)

file(
  GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/include/state-observation/config.h"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/include/state-observation/config.h.in")
install(FILES "${CMAKE_BINARY_DIR}/$<CONFIG>/include/state-observation/config.h" DESTINATION include/state-observation/)

add_subdirectory(src)

if(BUILD_STATE_OBSERVATION_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(unit-testings)
endif()

if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()

if(BUILD_STATE_OBSERVATION_TOOLS)
  add_subdirectory(misc-tools)
endif()

# Export the package files (headers, targets, config files, etc.)
jrl_export_package()
# Print summaries
jrl_print_dependencies_summary()
jrl_print_options_summary()
