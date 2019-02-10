#include "ai/learner.h"

Learner::Learner(String policy_class, String fa_class, float gamma): gamma(gamma) {
	
	if(policy_class=="Boltzmann"){
		printf("Learner::Learner() - BoltzmannPolicy set.\n");
		policy = new BoltzmannPolicy(NSF,NA);
	}
	else{
		printf("Learner::Learner() - Error: unknown policy class.\n");
		exit(0);
	}

	
	if(fa_class=="Linear"){
		printf("Learner::Learner() - LinearFA set.\n");
		fa = new LinearFA(NSF*NA,ALPHA,GAMMA,LAMBDA);
	}
	else if(fa_class=="NeuralNetwork"){
		printf("Learner::Learner() - Error: NeuralNetwork function approximation not implemented.\n");
		exit(0);
	}
	else{
		printf("Learner::Learner() - Error: unknown function approximation class.\n");
		exit(0);
	}

}

Learner::~Learner(){
	
	delete policy;
	delete fa;
}

int Learner::drawAction(float* stateFeatures){

	return policy->drawAction(stateFeatures);
}

void Learner::updatePolicy(Array<AgentEpisode> episodes){

	printf("Learner::updatePolicy - Error: not implemented.\n");
}
