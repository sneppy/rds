#include "math/vec4.h"

int main()
{
	Vec3<> v1(1.f, 0.f, 0.f), v2(0.f, 1.f, 0.f), v3(2.f), v4;
	Vec3<int32> v5(1, 0, 0), v6(0, 1, 0), v7(1), v8(1);
	Vec4<float32> vv(6.f, 8.f, 0.f, 0.f);
	v3 = v1 ^ v2;
	v7 = v5 ^ v6;
	v5.print();
	v6.print();
	v7.print();
	v8.print();
	vv.getNormal().print();
	return 0;
}