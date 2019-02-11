#pragma once
#include "coremin.h"
#include "ai/policy/policy.h"

class BoltzmannPolicy: public Policy {

	public:

		BoltzmannPolicy(int nSF, int nA);
		~BoltzmannPolicy();
		void initRandom(float minVal, float maxVal);

		void computePolicy(float *stateFeatures);
		void computeLogGradient(float *stateFeatures, int action);
		int drawAction(float* stateFeatures);

		void printPolicy();	
		int getNumParams();

		// from Policy class:
		//	float *policy; 		// nActions
		//	float *logGradient; // nParams
		//	float *params; 		// nParams
	
	private:

		void setParam(int sf, int a, float val);
		float getParam(int sf, int a);

		int nStateFeatures;
		int nActions;
		int nParams;
};