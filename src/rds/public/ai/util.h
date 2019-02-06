#pragma once
#include "coremin.h"


template<uint32 nSF> struct AgentStep {

	float s[nSF];
	int a;
	int r;
};

int probSample(float* distribution, int n);