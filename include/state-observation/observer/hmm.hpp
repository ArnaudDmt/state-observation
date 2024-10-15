#ifndef HmmESTIMATORHPP
#define HmmESTIMATORHPP

#include <functional>
#include <map>
#include <string>

#include "state-observation/api.h"
#include "state-observation/observer/zero-delay-observer.hpp"
#include "state-observation/tools/state-vector-arithmetics.hpp"

namespace stateObservation
{
class Hmm_state
{
public:
  Hmm_state(const std::string label,
            const std::function<float(Vector)> & emission,
            const std::map<std::string, std::function<float(Matrix)>> & transition);
  std::string get_label()
  {
    return label;
  };
  float get_emission(const Vector & O_t) const
  {
    return emission(O_t);
  }; // represent P(O_t | X_t = label)
  float get_transition(const std::string label_i, const Matrix & O_p_t) const
  {
    return transition.at(label_i)(O_p_t);
  }; // represent P(X_t+1 = label_i | X_t = label, O_p;t)
  // O_p;t is a matrix of size dim(O_t)*p with p the order to look at in the past for the transition

private:
  std::string label;
  std::function<float(Vector)> emission;
  std::map<std::string, std::function<float(Matrix)>> transition;
};

///////////////////////////////////////////// Hmm ////////////////////////////////////////

class STATE_OBSERVATION_DLLAPI Hmm : public ZeroDelayObserver
{
public:
  typedef std::map<std::string, std::function<float(Vector)>> Emission;
  typedef std::map<std::string, std::map<std::string, std::function<float(Matrix)>>> Transition;

  Hmm(const int n, const int m, const int p);
  Hmm(const int n,
      const int m,
      const int p,
      const std::vector<std::string> & labels,
      const Emission & emission,
      const Transition & transition);

  void setEmTr(const std::vector<std::string> & labels, const Emission & emission, const Transition & transition);

protected:
  ObserverBase::StateVector oneStepEstimation_() override;

  size_t n_state;

private:
  void calc_next_alpha();
  void calc_P();

  int n_obs;
  int m_obs;

  std::map<std::string, float> alpha_t;
  std::map<std::string, float> alpha_t_m_1;
  std::map<std::string, float> P;

  std::vector<Hmm_state> states;

  ObserverBase::StateVector X;

  Vector O_t;
  Matrix O_p_t;
};
} // namespace stateObservation
#endif
