#ifndef HMMCONTACTESTIMATOR
#define HMMCONTACTESTIMATOR

#include "state-observation/observer/hmm.hpp"

#pragma once

#include <string>
#include <vector>

using namespace std;

namespace stateObservation
{

class STATE_OBSERVATION_DLLAPI Hmm_contact_force : public Hmm
{

public:
  Hmm_contact_force(const int n,
                    const int p,
                    const std::vector<std::string> & labels,
                    const Hmm::Emission & emission,
                    const Hmm::Transition & transition);

  Hmm_contact_force(int n, int p);

  void setMeasurement(Vector2 F, Vector3 v, Vector3 v_p, TimeIndex k);
};

} // namespace stateObservation
#endif
