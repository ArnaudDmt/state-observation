using namespace std;

#include "state-observation/observer/hmm-contact.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>
namespace stateObservation
{

Hmm_contact_force::Hmm_contact_force(const int n,
                                     const int p,
                                     const std::vector<std::string> & labels,
                                     const Emission & emission,
                                     const Transition & transition)
: Hmm{n, 8, p, labels, emission, transition}
{
}

Hmm_contact_force::Hmm_contact_force(int n, int p) : Hmm{n, 8, p} {}

void Hmm_contact_force::setMeasurement(Vector2 F, Vector3 v, Vector3 v_p, TimeIndex k)
{
  ObserverBase::MeasureVector y(8);
  y << F, v, v_p;
  ZeroDelayObserver::setMeasurement(y, k);
}

} // namespace stateObservation
