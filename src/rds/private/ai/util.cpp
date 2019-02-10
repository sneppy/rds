#include "ai/util.h"

/*
*	Sample from a probability distribution {dist} with {n} terms
*/
int probSample(float* dist, int n){
	float cumul[n];
	float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	for(int i=0; i<n; i++){
		cumul[i] = (i==0 ? dist[i] : cumul[i-1]+dist[i]);
		if(r<cumul[i]) return i;
	}
	printf("Error in probSample: distribution doesn't sum up to 1.\n");
}

int getLinearFeaturesIndex(int sf, int a){
	return a*NSF + sf;
}

void buildLinearFeatures(float *stateFeatures, int action, float* ftPointer){

	if(action < 0 || action >= NA){
		printf("Error: invalid action in BoltzmannPolicy::buildFeatures.\n");
	}

	for(int s=0; s<NSF; s++){
		for(int a=0; a<NA; a++){
			ftPointer[getLinearFeaturesIndex(s,a)] = stateFeatures[s]*(action==a);
		}
	}
}