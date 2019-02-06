#include "coremin.h"
#include "ai/learner.h"

Malloc* gMalloc = nullptr;

int main(){

	Memory::createGMalloc();
	srand(time(nullptr));

	BoltzmannPolicy policy = BoltzmannPolicy(10,4);

	float sf[10] = {1, 0, 1, 0, 0, 1, 0, 1, 1, 0};
	policy.computePolicy(sf);
	policy.printPolicy();

	policy.computeLogGradient(sf,3);
	int a = policy.drawAction(sf);
	printf("%d\n",a);

	AgentEpisode ep;
	AgentStep<NSF> step = {{1,0,0,0,1,0,0,0,1,1},2,1};
	for(int i=0; i<10000000; i++) ep.push(step);
	printf("%d\n",ep.getCount());
}