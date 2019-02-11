#include "ai/policy/boltzmannPolicy.h"
#include "ai/util.h"

BoltzmannPolicy::BoltzmannPolicy(int nSF, int nA){

	nStateFeatures = nSF;
	nActions = nA;
	nParams = nSF * nA;
	params = (float*)malloc(nParams*sizeof(float));
	policy = (float*)malloc(nActions*sizeof(float));
	logGradient = (float*)malloc(nParams*sizeof(float));

	initRandom(-0.1,0.1);
};

BoltzmannPolicy::~BoltzmannPolicy(){

	free(params);
	free(policy);
	free(logGradient);
}

void BoltzmannPolicy::initRandom(float minVal, float maxVal){

	for(int i=0; i<nParams; i++){
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
		features[i] = (float*)malloc(nParams*sizeof(float));
		terms[i] = 0;
		buildLinearFeatures(stateFeatures,i,features[i]);
		for(int j=0; j<nParams; j++){
			terms[i] += params[j]*features[i][j];
		}
		terms[i] = exp(terms[i]);
		sumTerms += terms[i];
	}

	for(int f=0; f<nParams; f++){
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

int BoltzmannPolicy::drawAction(float* stateFeatures){

	computePolicy(stateFeatures);
	return probSample(policy,nActions);
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

int BoltzmannPolicy::getNumParams(){
	return nParams;
}