#include <iostream>
#include <state-observation/tools/definitions.hpp>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

#include <chrono>

#include <benchmark/benchmark.h>

using namespace stateObservation::kine;

class MyFixture : public benchmark::Fixture
{
public:
  void SetUp(const ::benchmark::State & state)
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
    Eigen::MatrixXd B;
    B.resize(72, 72);
  }
};

// Method 1: "1_Adj_Upper_APAt"
static void meth1_Adj_Upper_APAt(benchmark::State & state)
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
  Eigen::MatrixXd B;
  B.resize(72, 72);

  for(auto _ : state)
  {
    // This code gets timed

    B.triangularView<Eigen::Upper>() = A * P * A.transpose();
    Eigen::MatrixXd C(B.selfadjointView<Eigen::Upper>());
  }
}

// Method 2: "2_Adj_Upper_AUpperPAt"
static void meth2_Adj_Upper_AUpperPAt(benchmark::State & state)
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
  Eigen::MatrixXd B;
  B.resize(72, 72);
  for(auto _ : state)
  {
    // This code gets timed

    B.triangularView<Eigen::Upper>() = A * P.selfadjointView<Eigen::Upper>() * A.transpose();
    Eigen::MatrixXd D(B.selfadjointView<Eigen::Upper>());
  }
}

// Method 3: "3_QAPAt". Basic method, no optimization.
static void meth3_QAPAt(benchmark::State & state)
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
  Eigen::MatrixXd B;
  B.resize(72, 72);
  for(auto _ : state)
  {
    // This code gets timed

    B.noalias() = Q + A * P * A.transpose();
  }
  // std::cout << std::endl << "Method1: " << B1.sum() << std::endl;
}

// Method 4: "sum_2_Q". Adding Q to the method 2.
static void meth4_sum_meth2_Q(benchmark::State & state)
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
  Eigen::MatrixXd B;
  B.resize(72, 72);
  for(auto _ : state)
  {
    // This code gets timed

    B.triangularView<Eigen::Upper>() = Q;
    B.triangularView<Eigen::Upper>() += A * P.selfadjointView<Eigen::Upper>() * A.transpose();
  }
}

// Method 5: "sum_UpperQUpper_AAdjP_At"
static void meth5_sum_UpperQUpper_AAdjP_At(benchmark::State & state)
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
  Eigen::MatrixXd B;
  Eigen::MatrixXd Bt(72, 72);
  B.resize(72, 72);
  for(auto _ : state)
  {
    // This code gets timed

    Bt.noalias() = A * P.selfadjointView<Eigen::Upper>();
    B.triangularView<Eigen::Upper>() = Bt * A.transpose();
    B.triangularView<Eigen::Upper>() += Q;
  }
}

// Method 6: "6_llt_rankUpdate"
static void meth6_llt_rankUpdate(benchmark::State & state)
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
  Eigen::MatrixXd B;
  B.resize(72, 72);
  Eigen::MatrixXd Btemp;
  Btemp.resize(72, 72);
  Eigen::LLT<Eigen::MatrixXd> llt(72 * 72);
  for(auto _ : state)
  {
    // This code gets timed

    B.triangularView<Eigen::Upper>() = Q;
    llt.compute(P);
    Btemp.noalias() = A * llt.matrixL();
    B.selfadjointView<Eigen::Upper>().rankUpdate(Btemp);
  }
}

// Run the benchmark

// BENCHMARK_MAIN();

int main(int argc, char ** argv)
{
  // Register the function as a benchmark. The name of functions corresponds to the operations tested in the function.

  /* Tests for the symmetric matrix multiplications */
  BENCHMARK(meth1_Adj_Upper_APAt);
  BENCHMARK(meth2_Adj_Upper_AUpperPAt);
  /* Adding the sum with the Q symmetric matrix */
  BENCHMARK(meth3_QAPAt);
  BENCHMARK(meth4_sum_meth2_Q);
  BENCHMARK(meth5_sum_UpperQUpper_AAdjP_At);
  BENCHMARK(meth6_llt_rankUpdate);

  ::benchmark::Initialize(&argc, argv);
  if(::benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;
  ::benchmark::RunSpecifiedBenchmarks();
}
