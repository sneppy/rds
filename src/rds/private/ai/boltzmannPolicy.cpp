#include "ai/boltzmannPolicy.h"
#include "ai/util.h"

BoltzmannPolicy::BoltzmannPolicy(int nSF, int nA){

	nStateFeatures = nSF;
	nActions = nA;
	nFeatures = nSF * nA;
	params = (float*)malloc(nFeatures*sizeof(float));
	policy = (float*)malloc(nActions*sizeof(float));
	logGradient = (float*)malloc(nFeatures*sizeof(float));

	initRandom(-0.1,0.1);
};

void BoltzmannPolicy::initRandom(float minVal, float maxVal){

	for(int i=0; i<nFeatures; i++){
		params[i] = minVal + ((float)rand()/(float)(RAND_MAX))*(maxVal-minVal);
	}
};

void BoltzmannPolicy::computePolicy(float *stateFeatures){

	for(int a=0; a<nActions; a++){
		policy[a] = 0.0f;
	}

	for(int s=0; s<nStateFeatures; s++){
		for(int a=0; a<nActions; a++){
			policy[a] += getParam(s,a)*stateFeatures[s];
		}
	}

	float sum = 0.0f;
	for(int a=0; a<nActions; a++){
		policy[a] = exp(policy[a]);
		sum += policy[a];
	}

	for(int a=0; a<nActions; a++){
		policy[a] = policy[a]/sum;
	}
};

void BoltzmannPolicy::computeLogGradient(float *stateFeatures, int action){

	if(action < 0 || action >= nActions){
		printf("Error: invalid action in BoltzmannPolicy::computeLogGradient.\n");
	}

	float *features[nActions], terms[nActions], sumTerms=0.0f;
	for(int i=0; i<nActions; i++){
		features[i] = (float*)malloc(nFeatures*sizeof(float));
		terms[i] = 0;
		buildFeatures(stateFeatures,i,features[i]);
		for(int j=0; j<nFeatures; j++){
			terms[i] += params[j]*features[i][j];
		}
		terms[i] = exp(terms[i]);
		sumTerms += terms[i];
	}

	for(int f=0; f<nFeatures; f++){
		logGradient[f] = 0.0f;
		for(int a=0; a<nActions; a++){
			logGradient[f] -= terms[a]*features[a][f];
		}
		logGradient[f] /= sumTerms;
		logGradient[f] += features[action][f];
	}

	for(int a=0; a<nActions; a++){
		free(features[a]);
	}
};

void BoltzmannPolicy::buildFeatures(float *stateFeatures, int action, float* ftPointer){

	if(action < 0 || action >= nActions){
		printf("Error: invalid action in BoltzmannPolicy::buildFeatures.\n");
	}

	for(int s=0; s<nStateFeatures; s++){
		for(int a=0; a<nActions; a++){
			ftPointer[f(s,a)] = stateFeatures[s]*(action==a);
		}
	}
}

int BoltzmannPolicy::drawAction(float* stateFeatures){

	computePolicy(stateFeatures);
	return probSample(policy,nActions);
}

int BoltzmannPolicy::f(int sf, int a){
	return a*nStateFeatures + sf;
}

void BoltzmannPolicy::setParam(int sf, int a, float val){

	params[a*nStateFeatures + sf] = val;
};

float BoltzmannPolicy::getParam(int sf, int a){

	return params[a*nStateFeatures + sf];
};

void BoltzmannPolicy::printPolicy(){
	for(int a=0; a<nActions; a++){
		printf("%.3f ",policy[a]);
	}
	printf("\n");
}