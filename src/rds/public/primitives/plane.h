#pragma once

#include "coremin.h"

/**
 * @class Plane primitives/plane.h
 */
class Plane
{
public:
	/// Plane origin
	vec3 origin;

	/// Plane end
	vec3 end;

	/// Plane normal
	vec3 normal;

public:
	/// Default constructor
	FORCE_INLINE Plane() :
		origin(vec3::zero),
		end(1.f, 0.f, 1.f),
		normal(vec3::up) {}

	/// Normal constructor
	FORCE_INLINE Plane(const vec3 & _origin, const vec3 & _end, const vec3 & _normal) :
		origin(_origin),
		end(_end),
		normal(_normal) {}
	
	/// Get corners
	FORCE_INLINE Array<vec3> getCorners() const
	{
		const vec3 center	= (origin + end) / 2.f;
		const vec3 hdiag	= normal ^ ((end - origin) / 2.f);
		const vec3 cornerA	= center + hdiag;
		const vec3 cornerB	= center - hdiag;

		Array<vec3> out(4);
		out.push(origin), out.push(end),
		out.push(cornerA), out.push(cornerB);

		return out;
	}
};