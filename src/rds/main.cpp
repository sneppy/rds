#include "math/vec3.h"

int main()
{
	Vec3<> v1(1.f, 0.f, 0.f), v2(0.f, 1.f, 0.f), v3, v4;
	Vec3<int32> v5(1, 0, 0), v6(0, 1, 0), v7, v8;
	v3 = v1 ^ v2;
	v7 = v5 ^ v6;
	v1.print();
	v2.print();
	v3.print();
	return 0;
}