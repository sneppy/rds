#pragma once
#include "coremin.h"

class BoltzmannPolicy{

	public:

		BoltzmannPolicy(int nSF, int nA);
		void initRandom(float minVal, float maxVal);

		void computePolicy(float *stateFeatures);
		void computeLogGradient(float *stateFeatures, int action);
		int drawAction(float* stateFeatures);
		
		void printPolicy();
		int f(int sf, int a);
		void setParam(int sf, int a, float val);
		float getParam(int sf, int a);
		void buildFeatures(float *stateFeatures, int a, float* features);
	
	private:

		int nStateFeatures;
		int nActions;

		int nFeatures;
		float *params; // nFeatures

		float *policy; // nActions
		float *logGradient; // nFeatures
};