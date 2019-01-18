#pragma once

#include "math/math.h"

struct Box
{
public:
	/// Origin of the box
	vec3 min;

	/// End of the box
	vec3 max;

public:
	/// Default constructor, constructs a cube
	FORCE_INLINE Box() : min(0.f, 0.f, 0.f), max(1.f, 1.f, 1.f) {}

	/// Initialization constructor
	FORCE_INLINE Box(const vec3 & origin, const vec3 & extent) : min(origin), max(origin + extent) {}

	/// Test if point is inside box
	FORCE_INLINE bool contains(const vec3 & p) const
	{
		return p >= min & p <= max;
	}

	/**
	 * Find intersection with ray
	 * 
	 * @param [in] start,end start and end position of the ray
	 * @param [out] hit hit location
	 * @return @c true if hit
	 */
	FORCE_INLINE bool intersect(const vec3 & start, const vec3 & end, vec3 & hit)
	{
		const vec3 rd = (end - start).normalize();
		vec3 hitX, hitY, hitZ;
		float32 minDist = (end - start).getSquaredSize();
		bool bIntersect = false;
		
		if (start.x < min.x)
			hitX = Math::rayPlaneIntersect(start, rd, min, vec3::left);
		else
			hitX = Math::rayPlaneIntersect(start, rd, max, vec3::right);

		if (start.y < min.y)
			hitY = Math::rayPlaneIntersect(start, rd, min, vec3::down);
		else
			hitY = Math::rayPlaneIntersect(start, rd, max, vec3::up);
		
		if (start.z < min.z)
			hitZ = Math::rayPlaneIntersect(start, rd, min, vec3::backward);
		else
			hitZ = Math::rayPlaneIntersect(start, rd, max, vec3::forward);
		
		if (contains(hitX) & (hitX - start).getSquaredSize() < minDist)
		{
			hit = hitX;
			minDist = (hitX - start).getSquaredSize();
			bIntersect = true;
		}

		if (contains(hitY) & (hitY - start).getSquaredSize() < minDist)
		{
			hit = hitY;
			minDist = (hitY - start).getSquaredSize();
			bIntersect = true;
		}

		if (contains(hitZ) & (hitZ - start).getSquaredSize() < minDist)
		{
			hit = hitZ;
			minDist = (hitZ - start).getSquaredSize();
			bIntersect = true;
		}

		return bIntersect;
	}
};