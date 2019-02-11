#include "ai/learner.h"

Learner::Learner(PolicyClass pc, FAClass fc, float gamma) : gamma(gamma) {
	
	switch(pc){
		case PolicyClass::BOLTZMANN:
			printf("Learner::Learner() - BoltzmannPolicy set.\n");
			policy = new BoltzmannPolicy(NSF,NA);
			break;
		default:
			printf("Learner::Learner() - Error: unknown policy class.\n");
			exit(EXIT_SUCCESS);
			break;
	}
	
	switch(fc){
		case FAClass::LINEAR:
			printf("Learner::Learner() - LinearFA set.\n");
			fa = new LinearFA(NSF*NA,ALPHA,GAMMA,LAMBDA);
			break;

		case FAClass::NN:
			printf("Learner::Learner() - Error: NeuralNetwork function approximation not implemented.\n");
			exit(EXIT_SUCCESS);
			break;

		default:
			printf("Learner::Learner() - Error: unknown function approximation class.\n");
			exit(EXIT_SUCCESS);
			break;
	}

}

Learner::~Learner(){
	
	delete policy;
	delete fa;
}

int Learner::drawAction(float* stateFeatures){

	return policy->drawAction(stateFeatures);
}

void Learner::updateQFA(Array<AgentEpisode> &episodes){

	int numParams = policy->getNumParams();
	int N = episodes.getCount();

	for(int i=0; i<N; i++){

		AgentEpisode ep = episodes[i];
		int T = ep.getCount();
		for (int t=0; t<T-1; t++){

			AgentStep step1 = ep[t];
			AgentStep step2 = ep[t+1];

			float* f1 = (float*) malloc(numParams*sizeof(float));
			float* f2 = (float*) malloc(numParams*sizeof(float));
			buildLinearFeatures(step1.s,step1.a,f1);
			buildLinearFeatures(step2.s,step2.a,f2);

			fa->updateParams(f1,step1.r,f2,t==T-2);

			free(f1);
			free(f2);
		}
	}
}

void Learner::updatePolicy(Array<AgentEpisode> &episodes){

	int numParams = policy->getNumParams();
	float gradient[numParams];

	int N = episodes.getCount();
	for(int i=0; i<N; i++){

		AgentEpisode ep = episodes[i];
		float grad[numParams];
		for(int p=0; p<numParams; p++) grad[p] = 0;

		int T = ep.getCount();
		for(int t=0; t<T; t++){

			AgentStep step = ep[t];

			// compute state-action Q(s,a) [approximated] value
			float* features = (float*) malloc(numParams*sizeof(float));
			buildLinearFeatures(step.s,step.a,features);
			float q_value = fa->getValue(features);
			free(features);

			// compute log gradient of the policy in (s,a)
			policy->computeLogGradient(step.s,step.a);

			for(int p=0; p<numParams; p++) grad[p] += policy->logGradient[p]*q_value;
		}

		for(int p=0; p<numParams; p++) gradient[p] += grad[p]/(float)N;
	}
	
	/*
	*	Temporary solution, TODO: Adam optimizer
	*/
	for(int p=0; p<numParams; p++){
		policy->params[p] = policy->params[p] + 0.003 * gradient[p];
	}
}

void Learner::learningStep(Array<AgentEpisode> &episodes){

	updateQFA(episodes);
	updatePolicy(episodes);
}
