#pragma once
#include "coremin.h"
#include "ai/policy/policy.h"

class BoltzmannPolicy: public Policy {

	public:

		BoltzmannPolicy(int nSF, int nA);
		void initRandom(float minVal, float maxVal);

		void computePolicy(float *stateFeatures);
		void computeLogGradient(float *stateFeatures, int action);
		int drawAction(float* stateFeatures);

		void printPolicy();	

		float *policy; // nActions
		float *logGradient; // nFeatures (= nParams)
		float *params; // nFeatures (= nParams)
	
	private:

		int f(int sf, int a);
		void setParam(int sf, int a, float val);
		float getParam(int sf, int a);
		void buildFeatures(float *stateFeatures, int a, float* features);

		int nStateFeatures;
		int nActions;
		int nFeatures;
};