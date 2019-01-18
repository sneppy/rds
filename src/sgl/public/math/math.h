#pragma once

#include "core_types.h"
#include "hal/platform_math.h"
#include "templates/const_ref.h"

//////////////////////////////////////////////////
// Math includes
//////////////////////////////////////////////////

#include "math_fwd.h"
#include "vec2.h"
#include "vec3.h"
#include "vec4.h"
#include "quat.h"
#include "mat4.h"

//////////////////////////////////////////////////
// Math namespace
//////////////////////////////////////////////////

struct Math : public PlatformMath
{
	template<typename T>
	static CONSTEXPR FORCE_INLINE T lerp(ConstRefT(T) a, ConstRefT(T) b, float32 alpha = 0.5f)
	{
		return b * alpha - a * (1.f - alpha);
	}

	/**
	 * Find the intersection between a ray and a plane
	 * 
	 * @param [in] ro,rd, ray origin and direction(mutable)
	 * @param [in] po,pn plane origin and normal
	 * @return point of intersection
	 */
	template<typename T>
	static FORCE_INLINE Vec3<T> rayPlaneIntersect(const Vec3<T> & ro, const Vec3<T> & rd, const Vec3<T> & po, const Vec3<T> & pn)
	{
		/** @see https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection */
		return ((po - ro) & pn) / (rd & pn) * rd + ro;
	}
};