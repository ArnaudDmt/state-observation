/**
 * \file     imu-fixed-contact-dynamical-system.hpp
 * \author   Mehdi Benallegue
 * \date     2013
 * \brief    Definitions of the dynamical system of a robot flexibility with an IMU sensor.
 *
 * \details
 *
 *
 */

#ifndef FIXED_CONTACTS_IMU_DYNAMICS_FUNCTOR_HPP
#define FIXED_CONTACTS_IMU_DYNAMICS_FUNCTOR_HPP

#include <vector>

#include <state-observation/dynamical-system/dynamical-system-functor-base.hpp>
#include <state-observation/noise/noise-base.hpp>
#include <state-observation/sensors-simulation/accelerometer-gyrometer.hpp>
#include <state-observation/dynamical-system/algorithm/rigid-body-kinematics.hpp>

namespace stateObservation
{
namespace flexibilityEstimation
{
    /**
    * \class  IMUFixedContactDynamicalSystem
    * \brief  This class describes the dynamics of a robot's flexibility
    *         this dynamics is the simplest possible system, the flexibility
    *         is expressed as a rotation against the contact positions with no
    *         other hypothesis than that the contact points are at constant position
    *
    */
    class IMUFixedContactDynamicalSystem :
        public stateObservation::DynamicalSystemFunctorBase,
        private stateObservation::algorithm::RigidBodyKinematics
    {
    public:
        ///constructor
        explicit IMUFixedContactDynamicalSystem(double dt);

        ///virtual destructor
        virtual ~IMUFixedContactDynamicalSystem();

        ///Description of the state dynamics
        virtual stateObservation::Vector stateDynamics
        (const stateObservation::Vector& x, const stateObservation::Vector& u,
            unsigned k);

        ///Description of the sensor's dynamics
        virtual stateObservation::Vector measureDynamics
        (const stateObservation::Vector& x, const stateObservation::Vector& u,
            unsigned k);

        ///Sets a noise which disturbs the state dynamics
        virtual void setProcessNoise( stateObservation::NoiseBase * );

        ///Removes the process noise
        virtual void resetProcessNoise();

        ///Gets the process noise
        virtual stateObservation::NoiseBase * getProcessNoise() const;

        ///Sets a noise which disturbs the measurements
        virtual void setMeasurementNoise( stateObservation::NoiseBase * );

        ///Removes the measurement noise
        virtual void resetMeasurementNoise();

        ///Gets a pointer on the measurement noise
        virtual stateObservation::NoiseBase * getMeasurementNoise() const;

        ///Set the period of the time discretization
        virtual void setSamplingPeriod(double dt);

        ///Gets the state size
        virtual unsigned getStateSize() const;
        ///Gets the input size
        virtual unsigned getInputSize() const;
        ///Gets the measurement size
        virtual unsigned getMeasurementSize() const;

        ///Sets the number of contacts
        virtual void setContactsNumber(unsigned);

        ///Sets the position of the contact number i
        virtual void setContactPosition(unsigned i, const Vector3 & position);

    protected:

        stateObservation::AccelerometerGyrometer sensor_;

        stateObservation::NoiseBase * processNoise_;

        double dt_;

        Vector3Unaligned orientationVector_;
        QuaternionUnaligned quaternion_;

        Quaternion computeQuaternion_(const Vector3 & x);

        static const unsigned stateSize_=18;
        static const unsigned inputSize_=15;
        static const unsigned measurementSizeBase_=6;

        unsigned measurementSize_;

        std::vector <Vector3,Eigen::aligned_allocator<Vector3> >
            contactPositions_;

    private:

    public:
    };
}
}

#endif // FIXED-CONTACTS-IMU-DYNAMICS-FUNCTOR_HPP
