#pragma once
#include "coremin.h"

class Policy {
	public:
		virtual ~Policy() { /* ... */ }
		virtual void initRandom(float minVal, float maxVal) = 0;

		virtual void computePolicy(float *stateFeatures) = 0;
		virtual void computeLogGradient(float *stateFeatures, int action) = 0;
		virtual int drawAction(float* stateFeatures) = 0;

		virtual void printPolicy() = 0;
		virtual int getNumParams() = 0;

		float *policy; 		// nActions
		float *logGradient; // nParams
		float *params; 		// nParams
};