#pragma once
#include "coremin.h"

#define NSF 10
#define NA 4

#define ALPHA 0.003
#define GAMMA 0.999
#define LAMBDA 0.98

enum class PolicyClass {
	BOLTZMANN
};

enum class FAClass{
	LINEAR,
	NN
};

template<uint32 nSF> struct AgentStep {

	float s[nSF];
	int a;
	int r;
};

using AgentEpisode = Array<AgentStep<NSF>>;

int probSample(float* distribution, int n);

int getLinearFeaturesIndex(int sf, int a);

void buildLinearFeatures(float *stateFeatures, int action, float* ftPointer);