#include "ai/fa/linearFA.h"

LinearFA::LinearFA(int nf, float alpha, float gamma, float lambda) : nf(nf), alpha(alpha), gamma(gamma), lambda(lambda){

	theta = (float*) malloc(nf*sizeof(float));
	zeta = (float*) malloc(nf*sizeof(float));

	initRandom(-0.1,0.1);
}

LinearFA::~LinearFA(){

	free(theta);
	free(zeta);
}

void LinearFA::initRandom(float minVal, float maxVal){

	for(int i=0; i<nf; i++){
		theta[i] = minVal + ((float)rand()/(float)(RAND_MAX))*(maxVal-minVal);
		zeta[i] = 0;
	}
};

float LinearFA::getValue(float* features){

	float v = 0;
	for(int i=0; i<nf; i++){
		v += theta[i]*features[i];
	}
	return v;
}

void LinearFA::resetTrace(){

	for(int i=0; i<nf; i++){
		zeta[i] = 0;
	}
}

void LinearFA::updateParams(float* f_sa1, float r, float* f_sa2, int endOfEpisode){

	float delta = r + gamma*getValue(f_sa2) - (endOfEpisode ? 0 : getValue(f_sa1));
	
	for(int i=0; i<nf; i++){

		zeta[i] = f_sa1[i] + gamma*lambda*zeta[i];

		if(zeta[i] > z_threshold) zeta[i] = z_threshold;
		else if(zeta[i] < -z_threshold) zeta[i] = -z_threshold;
		
		theta[i] += alpha*delta*zeta[i];
	}

	if(endOfEpisode) resetTrace();
}