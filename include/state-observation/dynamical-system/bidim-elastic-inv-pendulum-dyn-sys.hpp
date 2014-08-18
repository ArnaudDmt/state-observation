/**
 * \file      imu-dynamical-system.hpp
 * \author    Mehdi Benallegue
 * \date       2013
 * \brief   The file describes the dynamical system defined by an inertial
 *          measurement unit (IMU) fixed on a rigid body.
 *
 * \details
 *
 *
 */

#ifndef BIDIM_ELASTIC_INV_PENDULUM
#define BIDIM_ELASTIC_INV_PENDULUM

#include <state-observation/dynamical-system/dynamical-system-functor-base.hpp>
#include <state-observation/noise/noise-base.hpp>

namespace stateObservation
{

     /**
    * \class  BidimElasticInvPendulum
    * \brief  The class is an implementation of the dynamical system defined by
    *         a 2D inverted pendulum with an elastic joint. The input is the
    *         horizontal acceleration.
    *
    */
    class BidimElasticInvPendulum : public DynamicalSystemFunctorBase
    {
    public:
        ///The constructor
        BidimElasticInvPendulum();

        ///The virtual destructor
        virtual ~BidimElasticInvPendulum();

        ///Description of the state dynamics
        virtual Vector stateDynamics
        (const Vector& x, const Vector& u, unsigned k);

        ///Description of the sensor's dynamics
        virtual Vector measureDynamics
        (const Vector& x, const Vector& u, unsigned k);

        ///Sets a noise which disturbs the state dynamics
        virtual void setProcessNoise( NoiseBase * );
        ///Removes the process noise
        virtual void resetProcessNoise();
        ///Gets the process noise
        virtual NoiseBase * getProcessNoise() const;

        ///Sets a noise which disturbs the measurements
        virtual void setMeasurementNoise( NoiseBase * );
        ///Removes the measurement noise
        virtual void resetMeasurementNoise();
        ///Gets a pointer on the measurement noise
        virtual NoiseBase * getMeasurementNoise() const;

        ///Set the period of the time discretization
        virtual void setSamplingPeriod(double dt);

        ///Gets the state size
        virtual unsigned getStateSize();
        ///Gets the input size
        virtual unsigned getInputSize();
        ///Gets the measurement size
        virtual unsigned getMeasurementSize();

        /// set the height of the com of the pendulum
        void setHeight(const double & h);
        /// set the mass of the pendulum
        void setMass(const double &m);
        /// set the elasticity of the pendulum
        void setElasticity(const double &k);

    protected:

        double k_;
        double m_;
        double h_;

        NoiseBase * processNoise_;

        double dt_;

        static const unsigned stateSize_=4;
        static const unsigned inputSize_=1;
        static const unsigned measurementSize_=0;
    };
}
#endif // BIDIM_ELASTIC_INV_PENDULUM
