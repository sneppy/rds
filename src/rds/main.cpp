#include "math/mat4.h"

int main()
{
	vec3 v(1.f), r(2.f);
	vec2 v2(1.f);
	printf("%f\n", v & r);
	mat4 m(
		2.f, 1.f, 0.f, 1.f,
		3.f, 1.f, 1.f, 1.f,
		2.f, 0.f, 3.f, 1.f,
		0.f, 0.f, 0.f, 1.f
	);
	//m = mat4::translation(vec3(4.f));
	m.print();
	(!m).print();
	auto start = clock();
	for (uint64 i = 0; i < 1e5; ++i)
		//m.getDeterminant();
		v2.getNormal();
	printf("%f\n", float(clock() - start) / CLOCKS_PER_SEC);
	return 0;
}