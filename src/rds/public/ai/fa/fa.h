#pragma once
#include "coremin.h"

/*
*	SARSA(λ) Function Approximator (abstract) class
*/
class FA {

	public:
		virtual ~FA() { /* ... */ }
		virtual void initRandom(float minVal, float maxVal) = 0;

};