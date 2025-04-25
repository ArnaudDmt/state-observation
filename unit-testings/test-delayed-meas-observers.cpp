// test avec delayed meas et delayed inputs.verifier indexes

//     test ou async sont ajoutes pour l'iteration suivant comme les inputs normaux.
#include <iostream>
#include <state-observation/observer/delayed-measurements-observer.hpp>

using namespace stateObservation;
int testWithoutInputMeas(int errorcode)
{
  struct TestObserver : DelayedMeasurementObserver
  {
    TestObserver(double dt,
                 Index n,
                 Index m,
                 unsigned long bufferCapacity,
                 const std::shared_ptr<IndexedInputArrayInterface> input = nullptr,
                 const std::shared_ptr<AsynchronousDataMapBase> async_input = nullptr,
                 const std::shared_ptr<AsynchronousDataMapBase> async_meas = nullptr)
    : DelayedMeasurementObserver(dt, n, m, bufferCapacity, input, async_input, async_meas)
    {
    }
    StateVector oneStepEstimation_(StateIterator it) override
    {
      (*it)().x() += 1.0;
      return (*it)();
    }
    void startNewIteration_() override {}
  };

  TestObserver obs(1.0, 3, 0, 100);

  obs.initEstimator(Vector3::Zero());
  for(int i = 0; i < 10; i++)
  {
    if(std::abs(obs.getEstimatedState(i + 1).x() - i - 1) > 1e-15) return errorcode;
  }

  return 0;
}

int testWithInput(int errorcode)
{
  struct TestObserverInput : public InputBase, public Vector1
  {
    TestObserverInput()
    {
      x() = 1;
    }
  };
  struct TestObserver : DelayedMeasurementObserver
  {
    TestObserver(double dt,
                 Index n,
                 Index m,
                 unsigned long bufferCapacity,
                 const std::shared_ptr<IndexedInputArrayInterface> input,
                 const std::shared_ptr<AsynchronousDataMapBase> async_input = nullptr,
                 const std::shared_ptr<AsynchronousDataMapBase> async_meas = nullptr)
    : DelayedMeasurementObserver(dt, n, m, bufferCapacity, input, async_input, async_meas)
    {
    }
    StateVector oneStepEstimation_(StateIterator it) override
    {
      TimeIndex k = getCurrentTime();
      const TestObserverInput & u = convert_input<TestObserverInput>((*u_)[k - 1]);
      (*it)().x() += u[0];
      return (*it)();
    }
    void startNewIteration_() override {}
  };

  TestObserver obs(1.0, 3, 0, 100, std::make_shared<IndexedInputArrayT<TestObserverInput>>());

  obs.initEstimator(Vector3::Zero());

  for(int i = 0; i < 10; i++)
  {
    obs.setInput(TestObserverInput(), i);
    if(std::abs(obs.getEstimatedState(i + 1).x() - i - 1) > 1e-15) return errorcode;
  }

  // now we test the use of a buffer of inputs
  obs.setCurrentState(Vector3::Zero());

  TimeIndex nextTime = obs.getCurrentTime() + 10;
  for(int i = obs.getCurrentTime(); i < nextTime; i++)
  {
    obs.setInput(TestObserverInput(), i);
  }
  if(std::abs(obs.getEstimatedState(nextTime).x() - 10) > 1e-15) return errorcode;

  // now we replay the previously played iterations, using the stored state and inputs
  for(int i = 0; i < 10; i++)
  {
    Vector state = obs.getEstimatedState(i + 1);
  }
  if(std::abs(obs.getCurrentEstimatedState().x() - 10) > 1e-15) return errorcode;

  return 0;
}

int testWithDelayedInput(int errorcode)
{
  struct AsynchronousTestInput : public AsynchronousDataBase
  {
  public:
    AsynchronousTestInput()
    {
      inputs_.push_back(1);
    }
    ~AsynchronousTestInput() {}
    inline void merge(const AsynchronousDataBase & input2) override
    {
      const AsynchronousTestInput & async_input2 = static_cast<const AsynchronousTestInput &>(input2);
      inputs_.insert(inputs_.end(), async_input2.inputs_.begin(), async_input2.inputs_.end());
    }

    std::vector<int> inputs_;
  };

  struct TestObserver : DelayedMeasurementObserver
  {
    TestObserver(double dt,
                 Index n,
                 Index m,
                 unsigned long bufferCapacity,
                 const std::shared_ptr<IndexedInputArrayInterface> input = nullptr,
                 const std::shared_ptr<AsynchronousDataMapBase> async_input = nullptr,
                 const std::shared_ptr<AsynchronousDataMapBase> async_meas = nullptr)
    : DelayedMeasurementObserver(dt, n, m, bufferCapacity, input, async_input, async_meas)
    {
    }
    StateVector oneStepEstimation_(StateIterator it) override
    {
      TimeIndex k = it->getTime();
      if(u_asynchronous_->checkIndex(k - 1))
      {
        AsynchronousTestInput & input = convert_async_data<AsynchronousTestInput>(u_asynchronous_->getElement(k - 1));
        for(auto & u : input.inputs_)
        {
          (*it)().x() += u;
        }
      }

      return (*it)();
    }
    void startNewIteration_() override {}
  };

  TestObserver obs(1.0, 3, 0, 50, nullptr, std::make_shared<AsynchronousDataMapT<AsynchronousTestInput>>());

  obs.initEstimator(Vector3::Zero());

  for(int i = 0; i < 10; i++)
  {
    obs.pushAsyncInput(AsynchronousTestInput(), i);
    Vector state = obs.getEstimatedState(i + 1);

    if(std::abs(state.x() - i - 1) > 1e-15) return errorcode;
  }

  // now we replay the previously played iterations, using the stored state and inputs
  for(int i = 0; i < 10; i++)
  {
    Vector state = obs.getEstimatedState(i + 1);
  }
  if(std::abs(obs.getCurrentEstimatedState().x() - 10) > 1e-15) return errorcode;

  obs.initEstimator(Vector3::Zero());
  obs.clearDelayedInputs();
  for(int i = 0; i < 10; i++)
  {
    obs.pushAsyncInput(AsynchronousTestInput(), 0);
  }
  if(std::abs(obs.getEstimatedState(1).x() - 10) > 1e-15) return errorcode;

  return 0;
}

int testWithDelayedInputAndMeas(int errorcode)
{
  struct TestData : public AsynchronousDataBase, public InputBase
  {
  public:
    TestData() : data_(0.25) {}
    ~TestData() {}
    inline void merge(const AsynchronousDataBase &) override {}

    double data_;
  };

  struct TestObserver : DelayedMeasurementObserver
  {
    TestObserver(double dt,
                 Index n,
                 Index m,
                 unsigned long bufferCapacity,
                 const std::shared_ptr<IndexedInputArrayInterface> input = nullptr,
                 const std::shared_ptr<AsynchronousDataMapBase> async_input = nullptr,
                 const std::shared_ptr<AsynchronousDataMapBase> async_meas = nullptr)
    : DelayedMeasurementObserver(dt, n, m, bufferCapacity, input, async_input, async_meas)
    {
    }
    StateVector oneStepEstimation_(StateIterator it) override
    {
      TimeIndex k = it->getTime();
      Vector meas = getMeasurement(k);
      TestData & input = convert_input<TestData>(u_->front());

      TestData & delayed_input = convert_async_data<TestData>(u_asynchronous_->getElement(k - 1));
      TestData & delayed_meas = convert_async_data<TestData>(y_asynchronous_->getElement(k));

      (*it)().x() += delayed_input.data_;
      (*it)().x() += delayed_meas.data_;
      (*it)().x() += meas[0];
      (*it)().x() += input.data_;

      return (*it)();
    }
    void startNewIteration_() override {}
  };

  TestObserver obs(1.0, 3, 1, 50, std::make_shared<IndexedInputArrayT<TestData>>(),
                   std::make_shared<AsynchronousDataMapT<TestData>>(),
                   std::make_shared<AsynchronousDataMapT<TestData>>());

  obs.initEstimator(Vector3::Zero());

  for(int i = 0; i < 10; i++)
  {
    obs.pushAsyncInput(TestData(), i);
    obs.setInput(TestData(), i);
    obs.setMeasurement(Vector1::Identity() * 0.25, i + 1);
    obs.pushAsyncMeasurement(TestData(), i + 1);
    Vector state = obs.getEstimatedState(i + 1);

    if(std::abs(state.x() - i - 1) > 1e-15) return errorcode;
  }

  return 0;
}

int testWithIntermittentInputAndMeas(int errorcode)
{
  struct TestData : public AsynchronousDataBase
  {
  public:
    TestData() : data_(0.5) {}
    ~TestData() {}
    inline void merge(const AsynchronousDataBase &) override {}

    double data_;
  };

  struct TestObserver : DelayedMeasurementObserver
  {
    TestObserver(double dt,
                 Index n,
                 Index m,
                 unsigned long bufferCapacity,
                 const std::shared_ptr<IndexedInputArrayInterface> input = nullptr,
                 const std::shared_ptr<AsynchronousDataMapBase> async_input = nullptr,
                 const std::shared_ptr<AsynchronousDataMapBase> async_meas = nullptr)
    : DelayedMeasurementObserver(dt, n, m, bufferCapacity, input, async_input, async_meas)
    {
    }
    StateVector oneStepEstimation_(StateIterator it) override
    {
      TimeIndex k = it->getTime();

      if(u_asynchronous_->checkIndex(k - 1) && y_asynchronous_->checkIndex(k))
      {
        TestData & delayed_input = convert_async_data<TestData>(u_asynchronous_->getElement(k - 1));
        TestData & delayed_meas = convert_async_data<TestData>(y_asynchronous_->getElement(k));

        (*it)().x() += delayed_input.data_;
        (*it)().x() += delayed_meas.data_;
      }

      return (*it)();
    }
    void startNewIteration_() override {}
  };

  TestObserver obs(1.0, 3, 1, 50, nullptr, std::make_shared<AsynchronousDataMapT<TestData>>(),
                   std::make_shared<AsynchronousDataMapT<TestData>>());

  obs.initEstimator(Vector3::Zero());

  for(int i = 0; i < 10; i++)
  {
    if(i % 2 == 0)
    {
      obs.pushAsyncInput(TestData(), i);
      obs.pushAsyncMeasurement(TestData(), i + 1);
    }
  }
  if(std::abs(obs.getEstimatedState(10).x() - 5) > 1e-15) return errorcode;

  return 0;
}

int main()
{
  int returnVal;
  int errorcode = 1;

  std::cout << "Starting test Delayed Measurement Observers." << std::endl;

  std::cout << "Starting testWithoutInputMeas." << std::endl;
  if((returnVal = testWithoutInputMeas(errorcode)))
  {
    std::cout << "testWithoutInputMeas failed!" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithoutInputMeas succeeded." << std::endl;
  }
  errorcode++;

  std::cout << "Starting testWithInput." << std::endl;
  if((returnVal = testWithInput(errorcode)))
  {
    std::cout << "testWithInput failed!" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithInput succeeded." << std::endl;
  }
  errorcode++;

  std::cout << "Starting testWithDelayedInput." << std::endl;
  if((returnVal = testWithDelayedInput(errorcode)))
  {
    std::cout << "testWithDelayedInput failed!" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithDelayedInput succeeded." << std::endl;
  }
  errorcode++;

  std::cout << "Starting testWithDelayedInputAndMeas." << std::endl;
  if((returnVal = testWithDelayedInputAndMeas(errorcode)))
  {
    std::cout << "testWithDelayedInputAndMeas failed!" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithDelayedInputAndMeas succeeded." << std::endl;
  }

  errorcode++;

  std::cout << "Starting testWithIntermittentInputAndMeas." << std::endl;
  if((returnVal = testWithIntermittentInputAndMeas(errorcode)))
  {
    std::cout << "testWithIntermittentInputAndMeas failed!" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testWithIntermittentInputAndMeas succeeded." << std::endl;
  }

  std::cout << "Test Delayed Measurement Observers succeeded." << std::endl;
  return 0;
}
