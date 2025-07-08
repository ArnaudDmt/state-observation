#include <state-observation/tools/odometry/legged-odometry-manager.hpp>
namespace stateObservation::odometry
{

///////////////////////////////////////////////////////////////////////
/// -------------------------Legged Odometry---------------------------
///////////////////////////////////////////////////////////////////////

void LeggedOdometryManager::init(const Configuration & odomConfig, const Vector7 & initPose)

{
  odometryType_ = odomConfig.odometryType_;
  withYawEstimation_ = odomConfig.withYaw_;
  correctContacts_ = odomConfig.correctContacts_;

  fbKine_.position = initPose.segment(0, 3);
  fbKine_.orientation.fromVector4(initPose.segment(3, 4));
}

void LeggedOdometryManager::run(KineParams & kineParams)
{
  BOOST_ASSERT_MSG(k_data_ != k_est_, "Please call initLoop before this function");
  BOOST_ASSERT_MSG(kineParams.tiltOrAttitudeMeas != nullptr, "Need a full orientation or at least a tilt estimate.");

  // updates the contacts and the resulting floating base kinematics
  updateFbAndContacts(kineParams);

  // updates the floating base kinematics in the observer
  updateFbKinematicsPvt(*kineParams.kineToUpdate);

  k_est_ = k_iter_;
}

void LeggedOdometryManager::updateFbAndContacts(const KineParams & params)
{
  // If the position and orientation of the floating base can be updated using contacts (that were already set on the
  // previous iteration), they are updated, else we keep the previous estimation. Then we estimate the pose of new
  // contacts using the obtained pose of the floating base.

  double sumLambdas_orientation = 0.0;

  // indicates if the orientation can be updated from the current contacts or not
  bool oriUpdatable = false;

  // if the given orientation is only a tilt, we compute the yaw using the one of the contacts
  if(!params.oriIsAttitude)
  {
    const stateObservation::Matrix3 & tilt = *(params.tiltOrAttitudeMeas);
    // selects the contacts to use for the yaw odometry. We cannot call it in the onMaintainedContact function as it is
    // looping over all the maintained contact and not used on each contact separately
    selectForOrientationOdometry(oriUpdatable, sumLambdas_orientation);

    // we update the orientation of the floating base first
    if(oriUpdatable)
    {
      // the orientation can be updated using contacts, it will use at most the two most suitable contacts.
      // We merge the obtained yaw with the tilt estimated by the previous observers
      if(contactsManager_.oriOdometryContacts_.size() == 1)
      {
        // the orientation can be updated using 1 contact
        fbKine_.orientation = stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
            tilt, contactsManager_.oriOdometryContacts_.begin()->get().worldFbKineFromRef_.orientation);
      }
      if(contactsManager_.oriOdometryContacts_.size() == 2) // the orientation can be updated using 2 contacts
      {
        const auto & contact1 = (*contactsManager_.oriOdometryContacts_.begin()).get();
        const auto & contact2 = (*std::next(contactsManager_.oriOdometryContacts_.begin(), 1)).get();

        const auto & R1 = contact1.worldFbKineFromRef_.orientation.toMatrix3();
        const auto & R2 = contact2.worldFbKineFromRef_.orientation.toMatrix3();

        // double u = contact2.forceRatio() / sumForceRatios_orientation;
        double u;
        u = contact2.lambda() / sumLambdas_orientation;

        /*
        \tilde{\boldsymbol{R}} = \boldsymbol{R}^{T}_{\mathcal{I}, 1} \boldsymbol{R}_{\mathcal{I}, 2}
        \boldsymbol{R}_{\mathcal{I}, \text{avg}} =
         \boldsymbol{R}_{\mathcal{I}, 1} \text{exp} \left( \lambda_{2} \text{vec}\left(\text{log} \left(
        \tilde{\boldsymbol{R}}\right)\right)  \right)
        */
        stateObservation::Matrix3 diffRot = R1.transpose() * R2;

        stateObservation::Vector3 diffRotVector =
            stateObservation::kine::skewSymmetricToRotationVector(diffRot - diffRot.transpose());

        stateObservation::Matrix3 meanOri =
            R1 * rotationVectorToRotationMatrix(u / 2.0 * diffRotVector); // = R1 * exp( (1 - u) * log(R1^T R2) )

        fbKine_.orientation = stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(tilt, meanOri);
      }
    }
    else
    {
      // If no contact is detected, the yaw will not be updated but the tilt will.
      fbKine_.orientation =
          stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(tilt, fbKine_.orientation.toMatrix3());
    }
  }
  else
  {
    const stateObservation::Matrix3 & attitude = *(params.tiltOrAttitudeMeas);
    fbKine_.orientation = attitude;
  }

  /*   Update of the position of the floating base    */
  if(params.worldPosMeas != nullptr)
  {
    /* If exceptionally the position in the world is given, we use it directly */
    fbKine_.position = *(params.worldPosMeas);
  }
  else
  {
    /*   if we can update the position, we compute the weighted average of the position obtained from the contacts    */
    if(!maintainedContacts_.empty())
    {
      fbKine_.position = getWorldFbPosFromAnchor();
    }
  }

  // we correct the reference position of the contacts in the world
  if(correctContacts_)
  {
    correctContactsRef();
  }

  // computation of the reference kinematics of the newly set contacts in the world. We cannot use the onNewContacts
  // function as it is used at the beginning of the iteration and we need to compute this at the end
  for(auto * nContact : newContacts_)
  {
    setNewContact(*nContact);
  }
}

void LeggedOdometryManager::updateFbVelocity(const Vector3 & linVel, Vector3 * angVel)
{
  fbKine_.linVel = linVel;
  if(angVel)
  {
    fbKine_.angVel = angVel;
  }
}

void LeggedOdometryManager::selectForOrientationOdometry(bool & oriUpdatable, double & sumLambdas)
{
  // we cannot update the orientation if no contact was set on last iteration
  if(!posUpdatable_ || !withYawEstimation_)
  {
    return;
  }

  contactsManager_.oriOdometryContacts_.clear();
  for(auto * mContact : maintainedContacts_)
  {
    if(mContact->isSet() && mContact->wasAlreadySet())
    {
      mContact->useForOrientation_ = true;
      contactsManager_.oriOdometryContacts_.insert(*mContact);
    }
  }

  // contacts are sorted from the lowest force to the highest force
  while(contactsManager_.oriOdometryContacts_.size() > 2)
  {
    (*contactsManager_.oriOdometryContacts_.begin()).get().useForOrientation_ = false;
    contactsManager_.oriOdometryContacts_.erase(contactsManager_.oriOdometryContacts_.begin());
  }

  // the position of the floating base in the world can be obtained by a weighted average of the estimations for each
  // contact
  for(LoContact & oriOdomContact : contactsManager_.oriOdometryContacts_)
  {
    // the orientation can be computed using contacts
    oriUpdatable = true;

    sumLambdas += oriOdomContact.lambda();

    oriOdomContact.worldFbKineFromRef_.orientation = Matrix3(oriOdomContact.worldRefKine_.orientation.toMatrix3()
                                                             * oriOdomContact.contactFbKine_.orientation.toMatrix3());
  }
}

Vector3 LeggedOdometryManager::getWorldFbPosFromAnchor()
{
  /* For each maintained contact, we compute the position of the floating base in the contact frame, we then compute the
   * weighted average wrt to the measured forces at the contact and obtain the estimated translation from the anchor
   * point to the floating base.  We apply this translation to the reference position of the anchor frame in the world
   * to obtain the new position of the floating base in the word. */

  fbAnchorPos_.setZero();
  Vector3 worldFbPosFromAnchor;

  for(auto * mContact : maintainedContacts_)
  {
    fbAnchorPos_ += mContact->contactFbKine_.getInverse().position() * mContact->lambda();
  }
  worldFbPosFromAnchor = getWorldRefAnchorPos() - fbKine_.orientation * fbAnchorPos_;

  return worldFbPosFromAnchor;
}

void LeggedOdometryManager::updateFbKinematicsPvt(Kinematics & pose)
{
  pose.position = fbKine_.position;
  pose.orientation = fbKine_.orientation;
}

void LeggedOdometryManager::setNewContact(LoContact & contact)
{
  contact.resetLifeTime();

  contact.worldRefKine_ = getContactKinematics(contact);
  contact.worldRefKineBeforeCorrection_ = contact.worldRefKine_;
  contact.newIncomingWorldRefKine_ = contact.worldRefKine_;

  if(odometryType_ == measurements::OdometryType::Flat)
  {
    contact.worldRefKine_.position()(2) = 0.0;
  }
}

const Kinematics & LeggedOdometryManager::getContactKinematics(LoContact & contact)
{
  // if the kinematics of the contact in the floating base have not been updated yet (k_est_ = k_iter_ - 1), we cannot
  // use them.
  BOOST_ASSERT_MSG(k_data_ == k_iter_, "Please update the contacts first.");

  // if the kinematics of the contact in the floating base have already been updated but the kinematics of the robot in
  // the world still have not changed, we don't need to recompute the kinematics of the contact in the world.
  if(k_data_ != k_est_)
  {
    contact.currentWorldKine_ = fbKine_ * contact.contactFbKine_.getInverse();
  };

  return contact.currentWorldKine_;
}

void LeggedOdometryManager::correctContactsRef()
{
  for(auto * mContact : maintainedContacts_)
  {
    // we store the pose of the contact before it is corrected
    mContact->worldRefKineBeforeCorrection_ = mContact->worldRefKine_;

    mContact->newIncomingWorldRefKine_ = getContactKinematics(*mContact);

    // double tau = ctl_dt_ / (kappa_ * mContact->lifeTime());
    mContact->correctionWeightingCoeff((1 - lambdaInf_) * exp(-kappa_ * mContact->lifeTime()) + lambdaInf_);

    Orientation Rtilde(Matrix3(mContact->worldRefKineBeforeCorrection_.orientation.toMatrix3().transpose()
                               * mContact->newIncomingWorldRefKine_.orientation.toMatrix3()));

    Vector3 logRtilde = skewSymmetricToRotationVector(Rtilde.toMatrix3() - Rtilde.toMatrix3().transpose());
    mContact->worldRefKine_.orientation =
        Matrix3(mContact->worldRefKineBeforeCorrection_.orientation.toMatrix3()
                * rotationVectorToRotationMatrix(mContact->correctionWeightingCoeff() / 2.0 * logRtilde));

    mContact->worldRefKine_.position =
        mContact->worldRefKine_.position()
        + mContact->correctionWeightingCoeff()
              * (mContact->newIncomingWorldRefKine_.position() - mContact->worldRefKine_.position());
    if(odometryType_ == measurements::OdometryType::Flat)
    {
      mContact->worldRefKine_.position()(2) = 0.0;
    }
  }

  k_correct_ = k_data_;
}

Kinematics LeggedOdometryManager::getContactKineIn(LoContact & contact, Kinematics & worldTargetKine)
{
  Kinematics targetContactKine = worldTargetKine.getInverse() * getContactKinematics(contact);
  return targetContactKine;
}

Kinematics LeggedOdometryManager::getAnchorKineIn(Kinematics & worldTargetKine)
{
  BOOST_ASSERT_MSG(k_data_ != k_est_, "Please call initLoop before this function");

  Kinematics targetAnchorKine;
  targetAnchorKine.position.set().setZero();

  if(worldTargetKine.linVel.isSet())
  {
    BOOST_ASSERT_MSG(fbKine_.linVel.isSet(),
                     "The velocity of the anchor frame cannot be computed without the velocity of the "
                     "floating base. Please make sure to use updateFbVelocity().");
    targetAnchorKine.linVel.set().setZero();
  }

  for(auto * mContact : maintainedContacts_)
  {
    Kinematics targetContactKine = getContactKineIn(*mContact, worldTargetKine);
    targetAnchorKine.position() += targetContactKine.position() * mContact->lambda();
    if(targetContactKine.linVel.isSet())
    {
      targetAnchorKine.linVel() += targetContactKine.linVel() * mContact->lambda();
    }
  }

  return targetAnchorKine;
}

void LeggedOdometryManager::replaceRobotPose(const Vector7 & newPose)
{
  Kinematics prevPoseKine = fbKine_;
  Kinematics & newPoseKine = fbKine_;

  newPoseKine.position = newPose.segment(0, 3);
  newPoseKine.orientation.fromVector4(newPose.segment(3, 4));

  for(auto & contact : maintainedContacts())
  {
    // compute the pose of the reference frame of the contact in the frame of the floating base.
    Kinematics fbWorldRefKine = prevPoseKine.getInverse() * contact->worldRefKine_;
    Kinematics fbWorldRefKineBeforeCorrection = prevPoseKine.getInverse() * contact->worldRefKineBeforeCorrection_;

    // compute the new pose of the reference frame of the contact in the frame of the world from the new floating base
    // pose.
    contact->worldRefKine_ = newPoseKine * fbWorldRefKine;
    contact->worldRefKineBeforeCorrection_ = newPoseKine * fbWorldRefKineBeforeCorrection;

    if(odometryType_ == measurements::OdometryType::Flat)
    {
      contact->worldRefKineBeforeCorrection_.position()(2) = 0.0;
      contact->worldRefKine_.position()(2) = 0.0;
    }
  }

  // we impose the re-computation of the anchor point as the contact references changed.
  k_anchor_ = k_data_ - 1;
}

const Vector3 & LeggedOdometryManager::getWorldRefAnchorPos()
{
  // if true, the contacts were not corrected since the last anchor computation, the anchor remains the same.
  bool contactsUnchanged = (k_anchor_ != k_correct_);
  // if true, the anchor has been computed since the beginning of the new iteration
  bool anchorComputed = (k_anchor_ == k_data_);

  // If the anchor point cannot be updated, we return the previously computed value.
  // We also return it if the anchor point has already been computed and the contacts have not been corrected yet (the
  // anchor therefore has not changed yet).
  if(!posUpdatable_ || (anchorComputed && contactsUnchanged))
  {
    return worldRefAnchorPosition_;
  }

  // weighted sum of the estimated floating base positions
  worldRefAnchorPosition_.setZero();

  // checks that the position and orientation can be updated from the currently set contacts and computes the pose of
  // the floating base obtained from each contact
  for(auto * mContact : maintainedContacts_)
  {
    const Kinematics & worldContactRefKine = mContact->worldRefKine_;
    // force weighted sum of the estimated floating base positions
    worldRefAnchorPosition_ += worldContactRefKine.position() * mContact->lambda();
  }

  k_anchor_ = k_data_;

  return worldRefAnchorPosition_;
}

void LeggedOdometryManager::setOdometryType(OdometryType newOdometryType)
{
  odometryType_ = newOdometryType;
}

} // namespace stateObservation::odometry
