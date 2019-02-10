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