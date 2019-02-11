#pragma once
#include "coremin.h"

/*
*	SARSA(λ) Function Approximator (abstract) class
*/
class FA {

	public:
		virtual ~FA() { /* ... */ }
		virtual void initRandom(float minVal, float maxVal) = 0;
		virtual float getValue(float* features) = 0;
		virtual void resetTrace() = 0;
		virtual void updateParams(float* sa1, float r, float* sa2, int endOfEpisode) = 0;

};