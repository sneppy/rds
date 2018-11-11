#include "math/vec3.h"

int main()
{
	Vec3<float32> v1(1.f, 2.f, 3.f);
	Vec3<int32> v2(1, 2, 3);
	v1 = v1.normalize() + v2;
	printf("%f\n", v1.y);
	printf("%f\n", v2.getSquaredSize());
	return 0;
}