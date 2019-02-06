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