IndexedVectorArray imuMultiplicativeAttitudeReconstruction(const IndexedVectorArray & y,
                                                           const IndexedInputVectorArray & u,
                                                           const Vector & xh0,
                                                           const Matrix & p,
                                                           const Matrix & q,
                                                           const Matrix & r,
                                                           double dt)
{
  /// Sizes of the states for the state, the measurement, and the input vector
  const Index stateSize = 19;
  const Index measurementSize = 6;
  const Index inputSize = 6;

  typedef kine::indexes<kine::rotationVector> indexesTangent;

  /// initialization of the extended Kalman filter
  ExtendedKalmanFilter filter(stateSize, indexesTangent::size, measurementSize, measurementSize, false, true,
                              std::make_shared<stateObservation::IndexedInputVectorArray>());

  /// initalization of the functor
  IMUMltpctiveDynamicalSystem imuFunctor;
  imuFunctor.setSamplingPeriod(dt);
  filter.setFunctor(&imuFunctor);
  filter.setStateArithmetics(&imuFunctor);

  /// the initalization of the estimation of the initial state
  filter.setState(xh0, y.getFirstIndex() - 1);

  /// computation and initialization of the covariance matrix of the initial state
  filter.setStateCovariance(p);

  /// set initial input
  filter.setInput(u[y.getFirstIndex() - 1], y.getFirstIndex() - 1);

  /// The covariance matrix of the process noise and the measurement noise
  ///  for the extended Kalman filter
  filter.setR(r);
  filter.setQ(q);

  /// the array of the state estimations over time
  IndexedVectorArray xh;
  xh.setValue(xh0, y.getFirstIndex() - 1);

  Vector xkp = xh0; // previous xk

  /// the reconstruction of the state
  for(TimeIndex i = y.getFirstIndex(); i < y.getNextIndex(); ++i)
  {
    /// introduction of the measurement
    filter.setMeasurement(y[i], i);

    /// introduction of the input
    if(i < y.getLastIndex()) filter.setInput(u[i], i);

    Matrix a = imuFunctor.getAMatrix(xkp);

    filter.updateStatePrediction();
    Matrix c = imuFunctor.getCMatrix(filter.getLastPrediction());

    // std::cout<<"a" << std::endl << a <<std::endl;
    // std::cout<<"c" << std::endl << c <<std::endl;

    filter.setA(a);
    filter.setC(c);

    /// get the estimation and give it to the array
    Vector xhk = filter.getEstimatedState(i);

    // std::cout<<"xh"<< xhk.transpose() <<std::endl;

    /// give the new value of the state to the kalman filter.
    /// This step is usually unnecessary, unless we modify the
    /// value of the state esimation which is the case here.
    filter.setState(xhk, i);

    xh.setValue(xhk, i);

    xkp = xhk;
  }

  return xh;
}

IndexedVectorArray imuMultiplicativeAttitudeReconstruction(const IndexedVectorArray & y,
                                                           const Vector & xh0,
                                                           const Matrix & p,
                                                           const Matrix & q,
                                                           const Matrix & r,
                                                           double dt)
{
  const Index inputSize = 6;

  /// initialization of a zero input
  stateObservation::IndexedInputVectorArray u;
  for(TimeIndex k = y.getFirstIndex() - 1; k < y.getNextIndex(); ++k)
  {
    VectorInput uk = VectorInput::Zero(inputSize, 1);
    u.setValue(uk, k);
  }

  return imuMultiplicativeAttitudeReconstruction(y, u, xh0, p, q, r, dt);
}
