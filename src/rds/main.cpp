#include "math/quat.h"

int main()
{
	Quat q1(vec3(0.f, 1.f, 0.f), 3.14f / 2.f), q2(vec3(0.f, 1.f, 0.f), 6.28f);
	(q1 * vec3::backward).print();
	q1.toForward().print();
	return 0;
}