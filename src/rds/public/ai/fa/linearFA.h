#pragma once
#include "coremin.h"
#include "ai/fa/fa.h"

class LinearFA: public FA {

	public:

		LinearFA(int nf, float alpha, float gamma, float lambda);
		~LinearFA();

		void initRandom(float minVal, float maxVal);
	
	private:

		int nf;
		float gamma;
		float lambda;
		float alpha;

		float *theta;
		float *zeta;
};