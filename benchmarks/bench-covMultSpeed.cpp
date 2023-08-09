#include <bitset>
#include <iostream>
#include <state-observation/tools/definitions.hpp>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

#include <chrono>

#include <benchmark/benchmark.h>

using namespace stateObservation::kine;

static void Method1(benchmark::State & state)
{
  // Perform setup here
  Eigen::MatrixXd P =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd Q =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd A =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  P = P * P.transpose(); // symmetric matrix
  Q = Q * Q.transpose(); // symmetric matrix
  Eigen::MatrixXd B1;
  B1.resize(72, 72);
  for(auto _ : state)
  {
    // This code gets timed

    // Method 1

    B1.noalias() = Q + A * P * A.transpose();
  }
  // std::cout << std::endl << "Method1: " << B1.sum() << std::endl;
}

static void Method2(benchmark::State & state)
{
  // Perform setup here
  Eigen::MatrixXd P =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd Q =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd A =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  P = P * P.transpose(); // symmetric matrix
  Q = Q * Q.transpose(); // symmetric matrix
  Eigen::MatrixXd B2;
  B2.resize(72, 72);
  for(auto _ : state)
  {
    // This code gets timed

    // Method 2

    B2.triangularView<Eigen::Upper>() = A * P * A.transpose();
    Eigen::MatrixXd C(B2.selfadjointView<Eigen::Upper>());
  }
}

static void Method3(benchmark::State & state)
{
  // Perform setup here
  Eigen::MatrixXd P =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd Q =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd A =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  P = P * P.transpose(); // symmetric matrix
  Q = Q * Q.transpose(); // symmetric matrix
  Eigen::MatrixXd B3;
  B3.resize(72, 72);
  for(auto _ : state)
  {
    // This code gets timed

    // Method 3
    B3.triangularView<Eigen::Upper>() = A * P.selfadjointView<Eigen::Upper>() * A.transpose();
    Eigen::MatrixXd D(B3.selfadjointView<Eigen::Upper>());
  }
}

static void Method3bis(benchmark::State & state)
{
  // Perform setup here
  Eigen::MatrixXd P =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd Q =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd A =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  P = P * P.transpose(); // symmetric matrix
  Q = Q * Q.transpose(); // symmetric matrix
  Eigen::MatrixXd B3;
  B3.resize(72, 72);
  for(auto _ : state)
  {
    // This code gets timed

    // Method 3
    B3.triangularView<Eigen::Upper>() = Q;
    B3.triangularView<Eigen::Upper>() += A * P.selfadjointView<Eigen::Upper>() * A.transpose();
  }
}

static void Method5(benchmark::State & state)
{
  // Perform setup here
  Eigen::MatrixXd P =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd Q =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd A =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  P = P * P.transpose(); // symmetric matrix
  Q = Q * Q.transpose(); // symmetric matrix
  Eigen::MatrixXd B5;
  Eigen::MatrixXd B5t(72, 72);
  B5.resize(72, 72);
  for(auto _ : state)
  {
    // This code gets timed

    // Method 5

    B5t.noalias() = A * P.selfadjointView<Eigen::Upper>();
    B5.triangularView<Eigen::Upper>() = B5t * A.transpose();
    B5.triangularView<Eigen::Upper>() += Q;
  }
}

static void Method7(benchmark::State & state)
{
  // Perform setup here
  Eigen::MatrixXd P =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd Q =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  Eigen::MatrixXd A =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, 72, 72>>() / 10;
  P = P * P.transpose(); // symmetric matrix
  Q = Q * Q.transpose(); // symmetric matrix
  Eigen::MatrixXd B7;
  B7.resize(72, 72);
  Eigen::MatrixXd B7temp;
  B7temp.resize(72, 72);
  Eigen::LLT<Eigen::MatrixXd> llt(72 * 72);
  for(auto _ : state)
  {
    // This code gets timed

    // Method 7
    B7.triangularView<Eigen::Upper>() = Q;
    llt.compute(P);
    B7temp.noalias() = A * llt.matrixL();
    B7.selfadjointView<Eigen::Upper>().rankUpdate(B7temp);
  }
}

// Run the benchmark

// BENCHMARK_MAIN();

int main(int argc, char ** argv)
{
  // Register the function as a benchmark
  BENCHMARK(Method1);
  BENCHMARK(Method2);
  BENCHMARK(Method3);
  BENCHMARK(Method3bis);
  BENCHMARK(Method5);
  BENCHMARK(Method7);

  ::benchmark::Initialize(&argc, argv);
  if(::benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;
  ::benchmark::RunSpecifiedBenchmarks();
}
