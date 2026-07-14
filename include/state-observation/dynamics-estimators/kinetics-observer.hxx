inline Index KineticsObserver::kineIndex() const
{
  return 0;
}
inline Index KineticsObserver::posIndex() const
{
  return kineIndex();
}
inline Index KineticsObserver::oriIndex() const
{
  return posIndex() + sizePos;
}
inline Index KineticsObserver::linVelIndex() const
{
  return oriIndex() + sizeOri;
}
inline Index KineticsObserver::angVelIndex() const
{
  return linVelIndex() + sizeLinVel;
}

inline Index KineticsObserver::gyroBiasIndex(Index numberOfIMU) const
{
  BOOST_ASSERT(numberOfIMU < maxImuNumber_ && "The requested IMU number is higher than the maximum");
  return angVelIndex() + sizeAngVel + sizeGyroBias * numberOfIMU;
}

inline Index KineticsObserver::gyroBiasIndex(Input::VectorIMUConstIterator i) const
{
  BOOST_ASSERT(i->stateIndex > 0 && "The requested imu is not set yet. The iterator may be wrong");

  return Index(i->stateIndex);
}

inline Index KineticsObserver::unmodeledWrenchIndex() const
{
  return gyroBiasIndex(0) + sizeGyroBias * maxImuNumber_;
}
inline Index KineticsObserver::unmodeledForceIndex() const
{
  return unmodeledWrenchIndex();
}
inline Index KineticsObserver::unmodeledTorqueIndex() const
{
  return unmodeledForceIndex() + sizeForce;
}
inline Index KineticsObserver::contactsIndex() const
{
  return unmodeledWrenchIndex() + sizeWrench;
}
inline Index KineticsObserver::contactIndex(Index contactNbr) const
{
  BOOST_ASSERT(contactNbr < maxContacts_ && "The requested contact number is higher than the maximum");
  BOOST_ASSERT(input_.contacts_[contactNbr].isSet && "The requested contact is not set yet, please add it before");
  return Index(input_.contacts_[contactNbr].stateIndex);
}
inline Index KineticsObserver::contactKineIndex(Index contactNbr) const
{
  return contactIndex(contactNbr);
}
inline Index KineticsObserver::contactPosIndex(Index contactNbr) const
{
  return contactKineIndex(contactNbr);
}
inline Index KineticsObserver::contactOriIndex(Index contactNbr) const
{
  return contactPosIndex(contactNbr) + sizePos;
}
inline Index KineticsObserver::contactWrenchIndex(Index contactNbr) const
{
  return contactKineIndex(contactNbr) + sizeContactKine;
}
inline Index KineticsObserver::contactForceIndex(Index contactNbr) const
{
  return contactWrenchIndex(contactNbr);
}
inline Index KineticsObserver::contactTorqueIndex(Index contactNbr) const
{
  return contactForceIndex(contactNbr) + sizeForce;
}

inline Index KineticsObserver::contactIndex(Input::VectorContactConstIterator i) const
{
  BOOST_ASSERT(i->stateIndex > 0 && "The requested contact is not set yet. The iterator may be wrong");

  return Index(i->stateIndex);
}
inline Index KineticsObserver::contactKineIndex(Input::VectorContactConstIterator i) const
{
  return contactIndex(i);
}
inline Index KineticsObserver::contactPosIndex(Input::VectorContactConstIterator i) const
{
  return contactKineIndex(i);
}
inline Index KineticsObserver::contactOriIndex(Input::VectorContactConstIterator i) const
{
  return contactPosIndex(i) + sizePos;
}
inline Index KineticsObserver::contactWrenchIndex(Input::VectorContactConstIterator i) const
{
  return contactKineIndex(i) + sizeContactKine;
}
inline Index KineticsObserver::contactForceIndex(Input::VectorContactConstIterator i) const
{
  return contactWrenchIndex(i);
}
inline Index KineticsObserver::contactTorqueIndex(Input::VectorContactConstIterator i) const
{
  return contactForceIndex(i) + sizeForce;
}

inline Index KineticsObserver::kineIndexTangent() const
{
  return 0;
}
inline Index KineticsObserver::posIndexTangent() const
{
  return 0;
}
inline Index KineticsObserver::oriIndexTangent() const
{
  return posIndexTangent() + sizePos;
}
inline Index KineticsObserver::linVelIndexTangent() const
{
  return oriIndexTangent() + sizeOriTangent;
}
inline Index KineticsObserver::angVelIndexTangent() const
{
  return linVelIndexTangent() + sizeLinVel;
}
inline Index KineticsObserver::gyroBiasIndexTangent(Index numberOfIMU) const
{
  BOOST_ASSERT(numberOfIMU < maxImuNumber_ && "The requested IMU number is higher than the maximum");
  return angVelIndexTangent() + sizeAngVel + sizeGyroBias * numberOfIMU;
}

inline Index KineticsObserver::gyroBiasIndexTangent(Input::VectorIMUConstIterator i) const
{
  BOOST_ASSERT(i->stateIndexTangent > 0 && "The requested imu is not set yet. The iterator may be wrong");

  return Index(i->stateIndexTangent);
}

inline Index KineticsObserver::unmodeledWrenchIndexTangent() const
{
  return gyroBiasIndexTangent(0) + sizeGyroBias * maxImuNumber_;
}
inline Index KineticsObserver::unmodeledForceIndexTangent() const
{
  return unmodeledWrenchIndexTangent();
}
inline Index KineticsObserver::contactsIndexTangent() const
{
  return unmodeledWrenchIndexTangent() + sizeWrench;
}
inline Index KineticsObserver::unmodeledTorqueIndexTangent() const
{
  return unmodeledForceIndexTangent() + sizeForce;
}
inline Index KineticsObserver::contactIndexTangent(Index contactNbr) const
{
  BOOST_ASSERT(contactNbr < maxContacts_ && "The requested contact number is higher than the maximum");
  return input_.contacts_[contactNbr].stateIndexTangent;
}
inline Index KineticsObserver::contactKineIndexTangent(Index contactNbr) const
{
  return contactIndexTangent(contactNbr);
}
inline Index KineticsObserver::contactPosIndexTangent(Index contactNbr) const
{
  return contactKineIndexTangent(contactNbr);
}
inline Index KineticsObserver::contactOriIndexTangent(Index contactNbr) const
{
  return contactPosIndexTangent(contactNbr) + sizePos;
}
inline Index KineticsObserver::contactWrenchIndexTangent(Index contactNbr) const
{
  return contactKineIndexTangent(contactNbr) + sizeContactKineTangent;
}
inline Index KineticsObserver::contactForceIndexTangent(Index contactNbr) const
{
  return contactWrenchIndexTangent(contactNbr);
}
inline Index KineticsObserver::contactTorqueIndexTangent(Index contactNbr) const
{
  return contactForceIndexTangent(contactNbr) + sizeForce;
}

inline Index KineticsObserver::contactIndexTangent(Input::VectorContactConstIterator i) const
{
  BOOST_ASSERT(i->stateIndexTangent > 0 && "The requested contact is not set yet. The iteratot may be wrong");
  return i->stateIndexTangent;
}
inline Index KineticsObserver::contactKineIndexTangent(Input::VectorContactConstIterator i) const
{
  return contactIndexTangent(i);
}
inline Index KineticsObserver::contactPosIndexTangent(Input::VectorContactConstIterator i) const
{
  return contactKineIndexTangent(i);
}
inline Index KineticsObserver::contactOriIndexTangent(Input::VectorContactConstIterator i) const
{
  return contactPosIndexTangent(i) + sizePos;
}
inline Index KineticsObserver::contactWrenchIndexTangent(Input::VectorContactConstIterator i) const
{
  return contactKineIndexTangent(i) + sizeContactKineTangent;
}
inline Index KineticsObserver::contactForceIndexTangent(Input::VectorContactConstIterator i) const
{
  return contactWrenchIndexTangent(i);
}
inline Index KineticsObserver::contactTorqueIndexTangent(Input::VectorContactConstIterator i) const
{
  return contactForceIndexTangent(i) + sizeForce;
}

inline Vector KineticsObserver::stateSum(const Vector & stateVector, const Vector & tangentVector)
{
  Vector sum;
  stateSum(stateVector, tangentVector, sum);
  return sum;
}

inline Vector KineticsObserver::stateDifference(const Vector & stateVector1, const Vector & stateVector2)
{
  Vector diff;
  stateDifference(stateVector1, stateVector2, diff);
  return diff;
}
