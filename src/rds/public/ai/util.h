#pragma once
#include "coremin.h"

#define NSF 10
#define NA 4

template<uint32 nSF> struct AgentStep {

	float s[nSF];
	int a;
	int r;
};

using AgentEpisode = Array<AgentStep<NSF>>;

int probSample(float* distribution, int n);