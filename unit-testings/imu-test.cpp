#include <fstream>
#include <iostream>

#include <state-observation/examples/imu-attitude-trajectory-reconstruction.hpp>
#include <state-observation/noise/gaussian-white-noise.hpp>

using namespace stateObservation;

typedef kine::indexes<kine::rotationVector> indexes;

int test(bool withGyroBias)
{
  /// The number of samples
  const Index kmax = 10000;

  /// sampling period
  const double dt = 1e-3;

  /// Sizes of the states for the state, the measurement, and the input vector
  unsigned stateSize;
  if(withGyroBias)
  {
    stateSize = 21;
  }
  else
  {
    stateSize = 18;
  }

  const unsigned measurementSize = 6;
  // const unsigned inputSize=6;

  /// The array containing all the states, the measurements and the inputs
  IndexedVectorArray x;
  IndexedVectorArray y;
  IndexedVectorArray u;
  Vector3 gyroBias = Vector3::Zero();

  /// The covariance matrix of the process noise and the measurement noise
  Matrix q;
  Matrix r;

  {
    /// simulation of the signal
    /// the IMU dynamical system functor
    /// (we set it to the wrong value to test the setter function)
    IMUDynamicalSystem imu(!withGyroBias);
    IMUDynamicalSystem imuSimulation;

    /// set it to the correct version
    imu.setWithGyroBias(withGyroBias);

    /// The process noise initialization
    Matrix q1Simu = Matrix::Identity(imuSimulation.getStateSize(), imuSimulation.getStateSize()) * 0.001;
    Matrix q1 = Matrix::Identity(stateSize, stateSize) * 0.001;
    GaussianWhiteNoise processNoise(imuSimulation.getStateSize());
    processNoise.setStandardDeviation(q1Simu);
    imuSimulation.setProcessNoise(&processNoise);
    q = q1 * q1.transpose();

    /// The measurement noise initialization
    Matrix r1 = Matrix::Identity(measurementSize, measurementSize) * 0.01;
    GaussianWhiteNoise MeasurementNoise(imuSimulation.getMeasurementSize());
    MeasurementNoise.setStandardDeviation(r1);
    imuSimulation.setMeasurementNoise(&MeasurementNoise);
    r = r1 * r1.transpose();

    /// the simulator initalization
    DynamicalSystemSimulator sim;
    sim.setDynamicsFunctor(&imuSimulation);

    /// initialization of the state vector
    Vector x0 = Vector::Zero(imuSimulation.getStateSize(), 1);
    sim.setState(x0, 0);

    /// construction of the input
    /// the input is constant over 10 time samples
    for(Index i = 0; i < kmax / 10; ++i)
    {
      Vector uk = Vector::Zero(imuSimulation.getInputSize(), 1);
      double id = double(i);

      uk[0] = 0.4 * sin(M_PI / 10 * id);
      uk[1] = 0.6 * sin(M_PI / 12 * id);
      uk[2] = 0.2 * sin(M_PI / 5 * id);

      uk[3] = 10 * sin(M_PI / 12 * id);
      uk[4] = 0.07 * sin(M_PI / 15 * id);
      uk[5] = 0.05 * sin(M_PI / 5 * id);

      /// filling the 10 time samples of the constant input
      for(int j = 0; j < 10; ++j)
      {
        u.setValue(uk, i * 10 + j);
      }

      /// give the input to the simulator
      /// we only need to give one value and the
      /// simulator takes automatically the appropriate value
      sim.setInput(uk, 10 * i);
    }

    /// set the sampling perdiod to the functor
    imu.setSamplingPeriod(dt);
    imuSimulation.setSamplingPeriod(dt);

    /// launched the simulation to the time kmax+1
    sim.simulateDynamicsTo(kmax + 1);

    /// extract the array of measurements and states
    y = sim.getMeasurementArray(1, kmax);

    if(withGyroBias)
    {

      gyroBias = (Vector3::Random() - Vector3::Constant(0.5)) * 3; /// random (time-constant) value of bias
      for(Index i = y.getFirstIndex(); i < y.getNextIndex(); ++i)
      {
        y[i].tail<3>() += gyroBias;
      }
    }

    x = sim.getStateArray(1, kmax);
  }

  /// initialization of the extended Kalman filter
  ExtendedKalmanFilter filter(stateSize, measurementSize, false, true);

  /// the initalization of a random estimation of the initial state
  Vector xh0 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector>(stateSize) * 3.14;
  xh0[indexes::ori] = 3.14;

  /// computation and initialization of the covariance matrix of the initial state
  Matrix p = Matrix::Zero(stateSize, stateSize);
  for(Index i = 0; i < filter.getStateSize(); ++i)
  {
    p(i, i) = xh0[i];
  }
  p = p * p.transpose();

  tools::SimplestStopwatch timer;
  timer.start();

  IndexedVectorArray xh = examples::imuAttitudeTrajectoryReconstruction(y, u, xh0, p, q, r, dt, withGyroBias);

  double duration = timer.stop();

#ifdef OUTPUT_TRAJECTORY_FILE
  /// file of output
  std::ofstream f;
  f.open("trajectory.dat");
#endif

  double dx = 0;

  for(TimeIndex i = y.getFirstIndex(); i < y.getNextIndex(); ++i)
  {
    /// display part, useless
    Vector3 g;
    {
      Matrix3 R;
      Vector3 orientationV = Vector(x[i]).segment(indexes::ori, 3);
      double angle = orientationV.norm();
      if(angle > cst::epsilonAngle)
        R = AngleAxis(angle, orientationV / angle).toRotationMatrix();
      else
        R = Matrix3::Identity();
      g = R.transpose() * Vector3::UnitZ();
      g.normalize();
    }

    Vector3 gh;
    {
      Matrix3 Rh;
      Vector3 orientationV = Vector(xh[i]).segment(indexes::ori, 3);
      double angle = orientationV.norm();
      if(angle > cst::epsilonAngle)
        Rh = AngleAxis(angle, orientationV / angle).toRotationMatrix();
      else
        Rh = Matrix3::Identity();
      gh = Rh.transpose() * Vector3::UnitZ();
      gh.normalize();
    }

    dx = acos(double(g.transpose() * gh));

#ifdef OUTPUT_TRAJECTORY_FILE
    f << i << " \t " << dx * 180 / M_PI << " \t\t\t " << g.transpose() << " \t\t\t " << gh.transpose() << std::endl;
#endif
  }

  if(withGyroBias)
  {
    std::cout << "Bias Estimation " << xh.back().tail<3>().transpose() << "real value  " << gyroBias.transpose()
              << "\n";
  }

  std::cout << "computation time: " << duration / kmax << ". ";
  std::cout << "Verticality estimation error (degrees): " << dx * 180 / M_PI;

  if(dx * 180 / M_PI < 1)
  {
    std::cout << " Test succeeded" << std::endl;
    return 0;
  }
  else
  {
    std::cout << " Test failed" << std::endl;
    return 1;
  }
}

int main()
{
  return test(false) || test(true);
}
