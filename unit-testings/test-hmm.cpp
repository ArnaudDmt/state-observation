#include <iostream>
#include <state-observation/observer/hmm-contact.hpp>

#include <functional>
#include <map>
#include <string>
#include <vector>

int main()
{

  std::cout << "Starting Hmm test" << std::endl;

  ////////////// Hmm Contact ////////////

  std::cout << "Test Hmm Contact estimator" << std::endl;

  int n = 3;
  int p = 3;
  stateObservation::Hmm_contact_force Hmm(n, p);
  std::cout << "Test contruction passed !" << std::endl;

  stateObservation::Vector2 F = {1, 2};
  stateObservation::Vector3 v = {1, 2, 3};
  stateObservation::Vector3 v_p = {1, 2, 3};

  Hmm.setMeasurement(F, v, v_p, 3);
  std::cout << "SetMeasurment test passed !" << std::endl;

  std::vector<std::string> labels = {"A", "B", "C"};
  stateObservation::Hmm_contact_force::Emission emission;
  emission["A"] = [](stateObservation::Hmm_contact_force::MeasureVector O_t) { return 2 * O_t(1); };
  emission["B"] = [](stateObservation::Hmm_contact_force::MeasureVector O_t) { return 2 * O_t(1); };
  emission["C"] = [](stateObservation::Hmm_contact_force::MeasureVector O_t) { return 2 * O_t(1); };

  stateObservation::Hmm_contact_force::Transition transition;
  transition["A"]["B"] = [](stateObservation::Hmm_contact_force::MeasureVector O_t) { return 2 * O_t(1); };
  transition["A"]["C"] = [](stateObservation::Hmm_contact_force::MeasureVector O_t) { return 2 * O_t(1); };
  transition["A"]["A"] = [&transition](stateObservation::Hmm_contact_force::MeasureVector O_t)
  { return 1 - transition["A"]["B"](O_t) - transition["A"]["C"](O_t); };

  transition["B"]["C"] = [](stateObservation::Hmm_contact_force::MeasureVector O_t) { return 2 * O_t(1); };
  transition["B"]["A"] = [](stateObservation::Hmm_contact_force::MeasureVector O_t) { return 2 * O_t(1); };
  transition["B"]["B"] = [&transition](stateObservation::Hmm_contact_force::MeasureVector O_t)
  { return 1 - transition["B"]["C"](O_t) - transition["B"]["A"](O_t); };

  transition["C"]["A"] = [](stateObservation::Hmm_contact_force::MeasureVector O_t) { return 2 * O_t(1); };
  transition["C"]["B"] = [](stateObservation::Hmm_contact_force::MeasureVector O_t) { return 2 * O_t(1); };
  transition["C"]["C"] = [&transition](stateObservation::Hmm_contact_force::MeasureVector O_t)
  { return 1 - transition["C"]["A"](O_t) - transition["C"]["B"](O_t); };

  Hmm.setEmTr(labels, emission, transition);

  std::cout << "SetEmTr test passed !" << std::endl;
  return 0;
}
