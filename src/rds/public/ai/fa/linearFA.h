#pragma once
#include "coremin.h"
#include "ai/fa/fa.h"

class LinearFA: public FA {

	public:

		LinearFA(int nf, float alpha, float gamma, float lambda);
		~LinearFA();

		void initRandom(float minVal, float maxVal);
		float getValue(float* features);
		void resetTrace();
		void updateParams(float* sa1, float r, float* sa2, int endOfEpisode);
	
	private:

		int nf;
		float gamma;
		float lambda;
		float alpha;
		float z_threshold = 1.0f;

		float *theta;
		float *zeta;
};