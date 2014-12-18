#include <state-observation/flexibility-estimation/stable-imu-fixed-contact-dynamical-system.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

namespace stateObservation
{
namespace flexibilityEstimation
{
    using namespace stateObservation;



    StableIMUFixedContactDynamicalSystem::
    	StableIMUFixedContactDynamicalSystem(double dt):
        processNoise_(0x0), dt_(dt),orientationVector_(Vector3::Zero()),
        quaternion_(Quaternion::Identity()),
        measurementSize_(measurementSizeBase_)
    {
#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
//        std::cout<<std::endl<<"IMUFixedContactDynamicalSystem Constructor"<<std::endl;
#endif //STATEOBSERVATION_VERBOUS_CONSTRUCTOR
    }

    StableIMUFixedContactDynamicalSystem::
                    ~StableIMUFixedContactDynamicalSystem()
    {
        //dtor
    }

    Vector3 StableIMUFixedContactDynamicalSystem::stabilizeAccelerationLinear
    	(Vector3 x, Vector3 xdot)
    {

        const Vector3 WLinear(5,5,5);
        const Vector3 ELinear(0.1,0.1,0.1);
        return -WLinear.cwiseProduct(WLinear.cwiseProduct(x))-2*ELinear.cwiseProduct(WLinear.cwiseProduct(xdot));

    }

    Vector3 StableIMUFixedContactDynamicalSystem::stabilizeAccelerationAngular
    	(Vector3 x, Vector3 xdot)
    {

        const Vector3 WAngular(5,5,5);
        const Vector3 EAngular(0.1,0.1,0.1);
        return -WAngular.cwiseProduct(WAngular.cwiseProduct(x))-2*EAngular.cwiseProduct(WAngular.cwiseProduct(xdot));

    }


    Vector StableIMUFixedContactDynamicalSystem::stateDynamics
        (const Vector& x, const Vector& , unsigned )
    {
        assertStateVector_(x);

        Vector3 positionFlex(x.segment(kine::pos,3));
        Vector3 velocityFlex(x.segment(kine::linVel,3));
        //Vector3 accelerationFlex(x.segment(kine::linAcc,3));
        Vector3 accelerationFlex(stabilizeAccelerationLinear(positionFlex, velocityFlex));

        Vector3 orientationFlexV(x.segment(kine::ori,3));
        Vector3 angularVelocityFlex(x.segment(kine::angVel,3));
        //Vector3 angularAccelerationFlex(x.segment(kine::angAcc,3));
        Vector3 angularAccelerationFlex(stabilizeAccelerationAngular(orientationFlexV, angularVelocityFlex));

        Quaternion orientationFlex(computeQuaternion_(orientationFlexV));

        integrateKinematics
                (positionFlex, velocityFlex, accelerationFlex, orientationFlex,
                 angularVelocityFlex, angularAccelerationFlex, dt_);

        //x_{k+1}
        Vector xk1(x);

        xk1.segment(kine::pos,3) = positionFlex;
        xk1.segment(kine::linVel,3) = velocityFlex;
        xk1.segment(kine::linAcc,3) = accelerationFlex;

        AngleAxis orientationAA(orientationFlex);
        orientationFlexV=orientationAA.angle()*orientationAA.axis();

        xk1.segment(kine::ori,3) =  orientationFlexV;
        xk1.segment(kine::angVel,3) = angularVelocityFlex;
        xk1.segment(kine::angAcc,3) = angularAccelerationFlex;

         if (processNoise_!=0x0)
            return processNoise_->addNoise(xk1);
        else
            return xk1;
    }

    Quaternion StableIMUFixedContactDynamicalSystem::computeQuaternion_
                (const Vector3 & x)
    {
        if (orientationVector_!=x)
        {
            orientationVector_ = x;
            quaternion_ = kine::rotationVectorToAngleAxis(x);
        }

        return quaternion_;
    }

    Vector StableIMUFixedContactDynamicalSystem::measureDynamics
                (const Vector& x, const Vector& u, unsigned k)
    {
        assertStateVector_(x);

        Vector3 positionFlex(x.segment(kine::pos,3));
        Vector3 velocityFlex(x.segment(kine::linVel,3));
        Vector3 accelerationFlex(x.segment(kine::linAcc,3));

        Vector3 orientationFlexV(x.segment(kine::ori,3));
        Vector3 angularVelocityFlex(x.segment(kine::angVel,3));
        Vector3 angularAccelerationFlex(x.segment(kine::angAcc,3));

        Quaternion qFlex (computeQuaternion_(orientationFlexV));
        Matrix3 rFlex (qFlex.toRotationMatrix());


        assertInputVector_(u);

        Vector3 positionControl(u.segment(kine::pos,3));
        Vector3 velocityControl(u.segment(kine::linVel,3));
        Vector3 accelerationControl(u.segment(kine::linAcc,3));

        Vector3 orientationControlV(u.segment(kine::ori,3));
        Vector3 angularVelocityControl(u.segment(kine::angVel,3));

        Quaternion qControl(computeQuaternion_(orientationControlV));

        Quaternion q = qFlex * qControl;

        Vector3 acceleration (
         (kine::skewSymmetric(angularAccelerationFlex)
              + tools::square(kine::skewSymmetric(angularVelocityFlex)))
                  * rFlex * positionControl
         + 2*kine::skewSymmetric(angularVelocityFlex) * rFlex * velocityControl
         + accelerationFlex + rFlex * accelerationControl);

        Vector3 angularVelocity( angularVelocityFlex +
                                    rFlex * angularVelocityControl);

        Vector v(Vector::Zero(10,1));

        v[0]=q.w();
        v[1]=q.x();
        v[2]=q.y();
        v[3]=q.z();

        v.segment(4,3)=acceleration;
        v.tail(3)=angularVelocity;

        sensor_.setState(v,k);

        Vector y (Matrix::Zero(measurementSize_,1));

        y.head(sensor_.getMeasurementSize()) = sensor_.getMeasurements();

        for (unsigned i=0; i<contactPositions_.size();++i)
        {
            y.segment(sensor_.getMeasurementSize()+i*3,3)=
                rFlex * contactPositions_[i] + positionFlex - contactPositions_[i];
        }

        return y;
    }

    void StableIMUFixedContactDynamicalSystem::setProcessNoise(NoiseBase * n)
    {
        processNoise_=n;
    }

    void StableIMUFixedContactDynamicalSystem::resetProcessNoise()
    {
        processNoise_=0x0;
    }

    void StableIMUFixedContactDynamicalSystem::setMeasurementNoise
                ( NoiseBase * n)
    {
        sensor_.setNoise(n);
    }

    void StableIMUFixedContactDynamicalSystem::resetMeasurementNoise()
    {
        sensor_.resetNoise();
    }

    void StableIMUFixedContactDynamicalSystem::setSamplingPeriod(double dt)
    {
        dt_=dt;
    }

    unsigned StableIMUFixedContactDynamicalSystem::getStateSize()
    {
        return stateSize_;
    }

    unsigned StableIMUFixedContactDynamicalSystem::getInputSize()
    {
        return inputSize_;
    }

    unsigned StableIMUFixedContactDynamicalSystem::getMeasurementSize()
    {
        return measurementSize_;
    }

    NoiseBase * StableIMUFixedContactDynamicalSystem::getProcessNoise() const
    {
        return processNoise_;
    }

    NoiseBase * StableIMUFixedContactDynamicalSystem::
                                                    getMeasurementNoise() const
    {
        return sensor_.getNoise();
    }

    void StableIMUFixedContactDynamicalSystem::setContactsNumber(unsigned i)
    {
        measurementSize_ = measurementSizeBase_ + 3 * i;
        contactPositions_.resize(i, Vector3::Zero());
    }

    void StableIMUFixedContactDynamicalSystem::setContactPosition
                                        (unsigned i, const Vector3 & position)
    {
        BOOST_ASSERT( i< contactPositions_.size() &&
                    "ERROR: The index of contact is out of range.");

        contactPositions_[i] = position;
    }
}
}
