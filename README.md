State-Observation
===========

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Hosted By: Cloudsmith](https://img.shields.io/badge/OSS%20hosting%20by-cloudsmith-blue?logo=cloudsmith)](https://cloudsmith.com)
[![CI](https://github.com/jrl-umi3218/state-observation/workflows/CI%20of%20state-observation/badge.svg?branch=master)](https://github.com/jrl-umi3218/state-observation/actions?query=workflow%3A%22CI+of+state-observation%22)
[![Documentation](https://img.shields.io/badge/website-online-brightgreen?logo=read-the-docs&style=flat)](https://jrl-umi3218.github.io/state-observation/)


This software provides tools for running state observation. It has three levels of uses:
* The high level users can exploit the developped estimators.
* The intermediate level users can derive these estimators to fit their system.
* The low level users can develop their novel estimators relying on the available tools.

Please refer to the [Documentaion](https://jrl-umi3218.github.io/state-observation/html/index.html)

Legged odometry and contacts management
---------------------------------------

The library provides tools to perform leg odometry, i.e. estimate the pose of a legged robot by tracking its successive contacts with the environment ([`tools/odometry/legged-odometry-manager.hpp`](include/state-observation/tools/odometry/legged-odometry-manager.hpp)).

### Contacts manager

`measurements::ContactsManager<ContactT>` ([`tools/measurements-manager/ContactsManager.hpp`](include/state-observation/tools/measurements-manager/ContactsManager.hpp)) keeps a map of the contacts, indexed by their name. On each iteration, it receives the set of the names of the currently detected contacts (the detection itself is left to the user) and calls the corresponding user function on each contact:
- `onNewContact`: the contact has just been detected;
- `onMaintainedContact`: the contact was already detected on the previous iteration;
- `onRemovedContact`: the contact is no longer detected;
- `onAddedContact` (optional): the contact is added to the map for the first time (e.g. to create its logs).

### Legged odometry manager

`odometry::LeggedOdometryManager` uses a `ContactsManager<LoContact>` and estimates the position and the yaw of a body of the robot (e.g. its IMU) after being provided a tilt estimation.

Principle:
- When a contact is created, its reference pose in the world is computed from the current estimated pose of the body and the kinematics of the contact in the body frame. It is then considered fixed.
- While the contact is maintained, its reference pose and its kinematics in the body frame give a measurement of the pose of the body. The measurements of all the maintained contacts are averaged with the weights `lambda` given by the user (they are normalized by the manager).
- The yaw is obtained from the orientation of the contacts (at most two contacts are used, the ones with the highest weights, and contacts at the hands are not considered as not always reliable).
- If enabled with `correctContacts`, the reference pose of each contact is decreasingly corrected by its current estimate, allowing to correct partially wrong initializations, slippage, etc.

Types of odometry (`OdometryType`):
- `Odometry6d` (`"6D"`): 3D position and orientation.
- `Flat`: same as `6D`, but the height of the contacts is set to zero (flat ground).
- `None`: no odometry, only the contacts management is used.

Usage:
```cpp
using namespace stateObservation;
odometry::LeggedOdometryManager odometry;

// initialization
odometry.setSamplingTime(dt);
odometry.kappa(10);
odometry.lambdaInf(0.001);
odometry::LeggedOdometryManager::Configuration config(odometry::OdometryType::Odometry6d);
config.correctContacts(true);
odometry.init(config, initialPose); // Vector7: position and quaternion of the body in the world

// on each iteration
auto onNewContact = [](odometry::LoContact & contact) {
  contact.bodyContactKine_ = /* kinematics of the contact in the body frame */;
  contact.lambda(/* weight of the contact, e.g. the norm of the measured force */);
};
auto onMaintainedContact = [](odometry::LoContact & contact) { /* same as onNewContact */ };
auto onRemovedContact = [](odometry::LoContact & contact) {};

auto updateFunctions = odometry::LeggedOdometryManager::ContactUpdateFunctions()
                           .onNewContact(onNewContact)
                           .onMaintainedContact(onMaintainedContact)
                           .onRemovedContact(onRemovedContact);
odometry.initLoop(detectedContacts, updateFunctions); // std::set<std::string> of the detected contacts

kine::Kinematics worldBodyKine = /* current estimate of the body kinematics in the world */;
odometry.run(odometry::LeggedOdometryManager::KineParams(worldBodyKine).tiltMeasurement(tilt)); // updates worldBodyKine
```
Instead of a tilt, a full orientation can be given with `attitudeMeasurement`. Examples of use can be found in the observers of [mc_state_observation](https://github.com/jrl-umi3218/mc_state_observation) (e.g. `MCValinor`).
