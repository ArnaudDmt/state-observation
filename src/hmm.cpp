#include"state-observation/observer/hmm.hpp"

using namespace std;


namespace stateObservation
{
hmm_state::hmm_state(string label, const function<float(Vector)>& emission, const map<std::string, function<float(Matrix)>>& transition){
    this->label = label;
    this->emission = emission;
    this->transition = transition;
}


///////////////////////////////////////////// HMM ////////////////////////////////////////

hmm::hmm(int n, int m, int p, std::vector<std::string> labels, const Emission emission, const Transition transition):
   ZeroDelayObserver{n, m}{

    // TODO test if size(labels)=size(emission)=size(transition)
    
    n_state = n; 
    n_obs  = m;
    m_obs = p;

    O_t = new Vector(m, 1);
    O_p_t = new Matrix(m, p);
    
    for (size_t i=0; i<n_state; i++) {
        hmm_state state(labels.at(i), emission.at(labels.at(i)), transition.at(labels.at(i)));
        states.push_back(state);
    }
}

void hmm::calc_next_alpha(){
    for (size_t i=0; i<n_state; i++){
        hmm_state state_i = states[i];
        string name = state_i.get_label();
        
        alpha_t[name] = state_i.get_emission(*O_t);
        float sum=0;
        for (size_t j=0; j<n_state; j++){
            string name_j = states[j].get_label();
            sum += state_i.get_transition(name_j, *O_p_t)*alpha_t_m_1[name_j];
        }
        alpha_t[name] *= sum;  
    }
}


void hmm::calc_P(){
    for (size_t i=0; i<n_state; i++){
        string label = states[i].get_label();
        P[label] = alpha_t[label];
       
        float sum=0;
        for (const auto [_, second]:alpha_t){
                sum += second;
        }
        P[label] /= sum;
    }
}


ObserverBase::StateVector hmm::oneStepEstimation_(){
    
    TimeIndex k = x_.getTime(); 

    *O_t = getMeasurement(k+1);
    O_p_t->block(0, 0, n_obs, m_obs-1) = O_p_t->block(0, 1, n_obs, m_obs); //m_obs-1 is the lengh of the sequence who started at index 1
    O_p_t->col(m_obs-1) = *O_t; //here m_obs-1 is the index of the last column

    calc_next_alpha();
    calc_P();

    for (size_t i = 0; i<n_state; i++){
        X(i, 0) = P[states[i].get_label()];
    }

    return X;
}



}
