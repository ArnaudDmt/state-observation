#include <iostream>
#include <state-observation/tools/definitions.hpp>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

#include <chrono>

#include <benchmark/benchmark.h>

using namespace stateObservation::kine;

/*
This benchmark tests the computation speed of several methods for the multiplication of matrices with symmetry
properties. More specifically, here we test the multiplication A*P*A.transpose() involved in the Kalman Filter because
it appeared that this operation is extremely computationally expensive even for average-sized matrices (like 50 rows x
columns).
*/

const int sizeState = 100;
const int sizeMeas = 10;

class MyFixture : public benchmark::Fixture
{
public:
  void SetUp(const ::benchmark::State & state)
  {
    // Perform setup here

    P = P * P.transpose(); // symmetric matrix
    Q = Q * Q.transpose(); // symmetric matrix

    B.resize(sizeState, sizeState);
  }

public:
  Eigen::MatrixXd P =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, sizeState, sizeState>>()
      / 10;
  Eigen::MatrixXd Q =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, sizeState, sizeState>>()
      / 10;
  Eigen::MatrixXd A =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, sizeState, sizeState>>()
      / 10;
  Eigen::MatrixXd K =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, sizeState, sizeMeas>>()
      / 10;
  Eigen::MatrixXd C =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, sizeMeas, sizeState>>()
      / 10;
  Eigen::MatrixXd R =
      stateObservation::tools::ProbabilityLawSimulation::getUniformMatrix<Eigen::Matrix<double, sizeMeas, sizeMeas>>()
      / 10;
  Eigen::MatrixXd B;
};

BENCHMARK_F(MyFixture, josephFormula)(benchmark::State & st)
{
  Eigen::MatrixXd P_prime = P.triangularView<Eigen::Upper>();
  // (I-KC) has the same size than Q so we use it
  for(auto _ : st)
  {
    // This code gets timed
    P.triangularView<Eigen::Upper>() =
        Q * P_prime.selfadjointView<Eigen::Upper>() * Q.transpose() + K * R * K.transpose();
  }
}

BENCHMARK_F(MyFixture, otherP_Formula)(benchmark::State & st)
{
  Eigen::MatrixXd P_prime = P.triangularView<Eigen::Upper>();
  // (I-KC) has the same size than Q so we use it
  for(auto _ : st)
  {
    // This code gets timed
    P.triangularView<Eigen::Upper>() = (Q * P_prime.selfadjointView<Eigen::Upper>()).eval();
  }
}

BENCHMARK_F(MyFixture, meth1_Adj_Upper_APAt)(benchmark::State & st)
{
  for(auto _ : st)
  {
    // This code gets timed

    B.triangularView<Eigen::Upper>() = A * P * A.transpose();
    Eigen::MatrixXd C(B.selfadjointView<Eigen::Upper>());
  }
}

BENCHMARK_F(MyFixture, 2_Adj_Upper_AUpperPAt)(benchmark::State & st)
{
  for(auto _ : st)
  {
    // This code gets timed

    B.triangularView<Eigen::Upper>() = A * P.selfadjointView<Eigen::Upper>() * A.transpose();
    Eigen::MatrixXd C(B.selfadjointView<Eigen::Upper>());
  }
}

BENCHMARK_F(MyFixture, meth2_Adj_Upper_AUpperPAt)(benchmark::State & st)
{
  for(auto _ : st)
  {
    // This code gets timed

    B.triangularView<Eigen::Upper>() = A * P.selfadjointView<Eigen::Upper>() * A.transpose();
    Eigen::MatrixXd C(B.selfadjointView<Eigen::Upper>());
  }
}

BENCHMARK_F(MyFixture, meth3_QAPAt)(benchmark::State & st)
{
  for(auto _ : st)
  {
    // This code gets timed

    B.noalias() = Q + A * P * A.transpose();
  }
}

BENCHMARK_F(MyFixture, meth4_sum_meth2_Q)(benchmark::State & st)
{
  for(auto _ : st)
  {
    // This code gets timed

    B.triangularView<Eigen::Upper>() = Q;
    B.triangularView<Eigen::Upper>() += A * P.selfadjointView<Eigen::Upper>() * A.transpose();

    Eigen::MatrixXd C(B.selfadjointView<Eigen::Upper>());
  }
}

BENCHMARK_F(MyFixture, meth5_sum_UpperQUpper_AAdjP_At)(benchmark::State & st)
{
  for(auto _ : st)
  {
    // This code gets timed
    Eigen::MatrixXd Bt(sizeState, sizeState);

    Bt.noalias() = A * P.selfadjointView<Eigen::Upper>();
    B.triangularView<Eigen::Upper>() = Bt * A.transpose();
    B.triangularView<Eigen::Upper>() += Q;
  }
}

BENCHMARK_F(MyFixture, meth6_llt_rankUpdate)(benchmark::State & st)
{
  for(auto _ : st)
  {
    // This code gets timed
    Eigen::MatrixXd Bt(sizeState, sizeState);
    Eigen::LLT<Eigen::MatrixXd> llt(sizeState * sizeState);

    B.triangularView<Eigen::Upper>() = Q;
    llt.compute(P);
    Bt.noalias() = A * llt.matrixL();
    B.selfadjointView<Eigen::Upper>().rankUpdate(Bt);
  }
}

// Run the benchmark

BENCHMARK_MAIN();
