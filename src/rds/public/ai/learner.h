#pragma once
#include "ai/util.h"
#include "ai/policy/policy.h"
#include "ai/policy/boltzmannPolicy.h"
#include "ai/fa/fa.h"
#include "ai/fa/linearFA.h"
#include "coremin.h"

/*
*	Policy gradient learner with Q-value function approximation
*/

class Learner {

	public:

		/*
		*	[POLICY] policy_class: "Boltzmann"
		*	[Q-FN APPROX] fa_class: "Linear", ["NeuralNetwork" - not implemented]
		*/
		Learner(PolicyClass pc = PolicyClass::BOLTZMANN, FAClass fc = FAClass::LINEAR, float gamma=0.99);
		~Learner();

		int drawAction(float* stateFeatures);
		void updateQFA(Array<AgentEpisode> &episodes);
		void updatePolicy(Array<AgentEpisode> &episodes);
		void learningStep(Array<AgentEpisode> &episodes);
	
	private:
		
		float gamma;
		Policy *policy;
		FA *fa;
};