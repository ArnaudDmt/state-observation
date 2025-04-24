// test avec delayed meas et delayed inputs.verifier indexes

//     test ou async sont ajoutes pour l'iteration suivant comme les inputs normaux.
#include <iostream>
#include <state-observation/observer/delayed-measurements-observer.hpp>

int testWithoutInputMeas(int errorcode)
{
  struct TestObserver : stateObservation::DelayedMeasurementObserver
  {
    TestObserver(double dt,
                 stateObservation::Index n,
                 stateObservation::Index m,
                 unsigned long bufferCapacity,
                 const std::shared_ptr<stateObservation::IndexedInputArrayInterface> input = nullptr,
                 const std::shared_ptr<stateObservation::AsynchronousDataMapBase> async_input = nullptr,
                 const std::shared_ptr<stateObservation::AsynchronousDataMapBase> async_meas = nullptr)
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

  obs.initEstimator(stateObservation::Vector3::Zero());
  for(int i = 0; i < 10; i++)
  {
    if(std::abs(obs.getEstimatedState(i + 1).x() - i - 1) > 1e-15) return errorcode;
  }

  return 0;
}

int testWithInput(int errorcode)
{
  struct TestObserverInput : public stateObservation::InputBase, public stateObservation::Vector1
  {
    TestObserverInput()
    {
      x() = 1;
    }
  };
  struct TestObserver : stateObservation::DelayedMeasurementObserver
  {
    TestObserver(double dt,
                 stateObservation::Index n,
                 stateObservation::Index m,
                 unsigned long bufferCapacity,
                 const std::shared_ptr<stateObservation::IndexedInputArrayInterface> input,
                 const std::shared_ptr<stateObservation::AsynchronousDataMapBase> async_input = nullptr,
                 const std::shared_ptr<stateObservation::AsynchronousDataMapBase> async_meas = nullptr)
    : DelayedMeasurementObserver(dt, n, m, bufferCapacity, input, async_input, async_meas)
    {
    }
    StateVector oneStepEstimation_(StateIterator it) override
    {
      stateObservation::TimeIndex k = getCurrentTime();
      const TestObserverInput & u = stateObservation::convert_input<TestObserverInput>((*u_)[k - 1]);
      (*it)().x() += u[0];
      return (*it)();
    }
    void startNewIteration_() override {}
  };

  TestObserver obs(1.0, 3, 0, 100, std::make_shared<stateObservation::IndexedInputArrayT<TestObserverInput>>());

  obs.initEstimator(stateObservation::Vector3::Zero());

  for(int i = 0; i < 10; i++)
  {
    obs.setInput(TestObserverInput(), i);
    if(std::abs(obs.getEstimatedState(i + 1).x() - i - 1) > 1e-15) return errorcode;
  }

  // now we test the use of a buffer of inputs
  obs.setCurrentState(stateObservation::Vector3::Zero());

  stateObservation::TimeIndex nextTime = obs.getCurrentTime() + 10;
  for(int i = obs.getCurrentTime(); i < nextTime; i++)
  {
    obs.setInput(TestObserverInput(), i);
  }
  if(std::abs(obs.getEstimatedState(nextTime).x() - 10) > 1e-15) return errorcode;

  // now we replay the previously played iterations, using the stored state and inputs
  for(int i = 0; i < 10; i++)
  {
    stateObservation::Vector state = obs.getEstimatedState(i + 1);
  }
  if(std::abs(obs.getCurrentEstimatedState().x() - 10) > 1e-15) return errorcode;

  return 0;
}

int testWithDelayedInput(int errorcode)
{
  struct AsynchronousTestInput : public stateObservation::AsynchronousDataBase
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

  struct TestObserver : stateObservation::DelayedMeasurementObserver
  {
    TestObserver(double dt,
                 stateObservation::Index n,
                 stateObservation::Index m,
                 unsigned long bufferCapacity,
                 const std::shared_ptr<stateObservation::IndexedInputArrayInterface> input = nullptr,
                 const std::shared_ptr<stateObservation::AsynchronousDataMapBase> async_input = nullptr,
                 const std::shared_ptr<stateObservation::AsynchronousDataMapBase> async_meas = nullptr)
    : DelayedMeasurementObserver(dt, n, m, bufferCapacity, input, async_input, async_meas)
    {
    }
    StateVector oneStepEstimation_(StateIterator it) override
    {
      stateObservation::TimeIndex k = it->getTime();
      if(u_asynchronous_->checkIndex(k - 1))
      {
        AsynchronousTestInput & input =
            stateObservation::convert_async_data<AsynchronousTestInput>(u_asynchronous_->getElement(k - 1));
        for(auto & u : input.inputs_)
        {
          (*it)().x() += u;
        }
      }

      return (*it)();
    }
    void startNewIteration_() override {}
  };

  TestObserver obs(1.0, 3, 0, 50, nullptr,
                   std::make_shared<stateObservation::AsynchronousDataMapT<AsynchronousTestInput>>());

  obs.initEstimator(stateObservation::Vector3::Zero());

  for(int i = 0; i < 10; i++)
  {
    obs.pushAsyncInput(AsynchronousTestInput(), i);
    stateObservation::Vector state = obs.getEstimatedState(i + 1);

    if(std::abs(state.x() - i - 1) > 1e-15) return errorcode;
  }

  // now we replay the previously played iterations, using the stored state and inputs
  for(int i = 0; i < 10; i++)
  {
    stateObservation::Vector state = obs.getEstimatedState(i + 1);
  }
  if(std::abs(obs.getCurrentEstimatedState().x() - 10) > 1e-15) return errorcode;

  return 0;
}

int main()
{
  int returnVal;
  int errorcode = 1;

  std::cout << "Starting test Delayed Measurement Observers." << std::endl;

  //   std::cout << "Starting testWithoutInputMeas." << std::endl;
  //   if((returnVal = testWithoutInputMeas(errorcode)))
  //   {
  //     std::cout << "testWithoutInputMeas failed!" << std::endl;
  //     return returnVal;
  //   }
  //   else
  //   {
  //     std::cout << "testWithoutInputMeas succeeded." << std::endl;
  //   }
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

  std::cout << "Test Delayed Measurement Observers succeeded." << std::endl;
  return 0;
}
