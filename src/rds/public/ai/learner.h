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
		*	[Q-FN APPROX] fa_class: ["Linear" - not implemented], ["NeuralNetwork" - not implemented]
		*/
		Learner(String policy_class="Boltzmann", String fa_class="Linear", float gamma=0.99);
		~Learner();

		int drawAction(float* stateFeatures);
		void updatePolicy(Array<AgentEpisode> data);
	
	private:
		
		float gamma;
		Policy *policy;
		FA *fa;
};