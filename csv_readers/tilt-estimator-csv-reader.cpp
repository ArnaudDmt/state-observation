#include <fstream>
#include <iostream>
#include <sstream>
#include <state-observation/observer/tilt-estimator.hpp>

struct Iter
{
  double timestamp;
  double alpha;
  double beta;
  double gamma;
  Eigen::Vector3d yv; // Velocity measurement
  Eigen::Vector3d ya; // Acceleration measurement
  Eigen::Vector3d yg; // Gyroscope measurement
};

// Function to read parameters and measurements from the CSV
std::vector<Iter> readCSV(const std::string & filename,
                          Eigen::Vector3d & x1,
                          Eigen::Vector3d & x2_prime,
                          Eigen::Vector3d & x2)
{
  std::vector<Iter> iterations_;
  std::ifstream file(filename);
  std::string line;

  if(!file.is_open())
  {
    throw std::runtime_error("Failed to open the CSV file.");
  }

  int lineNumber = 0;

  while(std::getline(file, line))
  {
    std::stringstream ss(line);
    std::string cell;

    if(lineNumber == 0)
    { // Read initial state
      for(int i = 0; i < 3; ++i)
      {
        std::getline(ss, cell, ',');
        x1[i] = std::stod(cell);
      }

      for(int i = 0; i < 3; ++i)
      {
        std::getline(ss, cell, ',');
        x2_prime[i] = std::stod(cell);
      }

      for(int i = 0; i < 3; ++i)
      {
        std::getline(ss, cell, ',');
        x2[i] = std::stod(cell);
      }
    }
    if(lineNumber > 0)
    { // Read timestamped iterations
      Iter iter;

      std::getline(ss, cell, ',');
      iter.timestamp = std::stod(cell);

      std::getline(ss, cell, ',');
      iter.alpha = std::stod(cell);
      std::getline(ss, cell, ',');
      iter.beta = std::stod(cell);
      std::getline(ss, cell, ',');
      iter.gamma = std::stod(cell);

      for(int i = 0; i < 3; ++i)
      {
        std::getline(ss, cell, ',');
        iter.yv[i] = std::stod(cell);
      }

      for(int i = 0; i < 3; ++i)
      {
        std::getline(ss, cell, ',');
        iter.ya[i] = std::stod(cell);
      }

      for(int i = 0; i < 3; ++i)
      {
        std::getline(ss, cell, ',');
        iter.yg[i] = std::stod(cell);
      }

      iterations_.push_back(iter);
    }

    ++lineNumber;
  }

  std::cout << "Retrieved data with " << iterations_.size() << " iterations." << std::endl;

  return iterations_;
}

void writeEstimatedStateToCSV(const std::string & filename,
                              const std::vector<double> & timestamps,
                              const std::vector<Eigen::Vector3d> & x1_states,
                              const std::vector<Eigen::Vector3d> & x2_states,
                              const std::vector<Eigen::Vector3d> & x2prime_states)
{
  std::ofstream file(filename);
  Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

  // Write headers
  file << "t,x1_x,x1_y,x1_z,x2_x,x2_y,x2_z,x2prime_x,x2prime_y,x2prime_z\n";

  // Write data
  for(size_t i = 0; i < timestamps.size(); ++i)
  {
    file << timestamps[i];
    file << "," << x1_states[i].transpose().format(CSVFormat); // Write x1
    file << "," << x2_states[i].transpose().format(CSVFormat); // Write x2
    file << "," << x2prime_states[i].transpose().format(CSVFormat); // Write x2prime
    file << "\n";
  }

  file.close();
}

int main()
{
  std::cout << "Starting the estimation" << std::endl;
  std::string filename = "data.csv";

  // Parameters and initial states
  Eigen::Vector3d x1, x2_prime, x2;

  std::vector<double> timestamps;
  std::vector<Eigen::Vector3d> x1_states, x2_states, x2prime_states;

  // Read CSV data
  std::vector<Iter> iterations_;
  try
  {
    iterations_ = readCSV(filename, x1, x2_prime, x2);
  }
  catch(const std::exception & e)
  {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  if(iterations_.empty())
  {
    std::cerr << "No measurements found in the CSV file." << std::endl;
    return -1;
  }

  // Initialize the TiltEstimator
  stateObservation::TiltEstimator estimator(iterations_[0].alpha, iterations_[0].beta, iterations_[0].gamma,
                                            0.01); // Temporary dt
  estimator.initEstimator(x1, x2_prime, x2);

  // Iterate through the measurements
  for(size_t k = 0; k < iterations_.size() - 1; ++k)
  {
    double dt = iterations_[k + 1].timestamp - iterations_[k].timestamp;

    estimator.setSamplingTime(dt);

    if(dt <= 0.0)
    {
      std::cerr << "Invalid timestamp difference at index " << k << ": " << dt << "." << std::endl;
      continue;
    }
    estimator.setAlpha(iterations_[k].alpha);
    estimator.setBeta(iterations_[k].beta);
    estimator.setGamma(iterations_[k].gamma);
    // Update measurements
    estimator.setMeasurement(iterations_[k].yv, iterations_[k].ya, iterations_[k].yg, k + 1);

    // Run one step of estimation
    stateObservation::ObserverBase::StateVector x_hat = estimator.getEstimatedState(k + 1);

    // Output the current estimated state
    std::cout << "Estimated state at time " << iterations_[k + 1].timestamp << ": " << x_hat.transpose() << std::endl;

    // Extract components
    Eigen::Vector3d x1_hat = x_hat.segment<3>(0);
    Eigen::Vector3d x2prime_hat = x_hat.segment<3>(3);
    Eigen::Vector3d x2_hat = x_hat.segment<3>(6);

    // Store results
    timestamps.push_back(iterations_[k + 1].timestamp);
    x1_states.push_back(x1_hat);
    x2_states.push_back(x2_hat);
    x2prime_states.push_back(x2prime_hat);
  }
  writeEstimatedStateToCSV("estimationResults.csv", timestamps, x1_states, x2_states, x2prime_states);

  return 0;
}
