#include "Geometry.h"
#include "TextureLoader.h"

Vector2f ClosestPoint(const Plane& plane, const Vector2f& point)
{
	const float dot = plane.normal.dot(point);
	const float distance = dot - plane.distance;
	return point - plane.normal * distance;
}

//Vector2f ClosestPoint(const AABB& aabb, const Vector2f& point)
//{
//	Vector2f result = point;
//	Vector2f min = GetMin(aabb);
//	Vector2f max = GetMax(aabb);
//
//	result.x = (result.x < min.x) ? min.x : result.x;
//	result.y = (result.y < min.x) ? min.y : result.y;
//
//	result.x = (result.x > max.x) ? max.x : result.x;
//	result.y = (result.y > max.x) ? max.y : result.y;
//
//	return result;
//}

float SqDistPointAABB(const Vector2f& p, const AABB& b)
{
	Vector2f min = GetMin(b);
	Vector2f max = GetMax(b);

	float sqDist = 0.0f;
	
    // For each axis count any excess distance outside box extents
	float v = p.x;
	if (v < min.x) sqDist += (min.x - v) * (min.x - v);
	if (v > max.x) sqDist += (v - max.x) * (v - max.x);

	v = p.y;
	if (v < min.y) sqDist += (min.y - v) * (min.y - v);
	if (v > max.y) sqDist += (v - max.y) * (v - max.y);
	
	return sqDist;
}

// Given point p, return the point q on or in AABB b that is closest to p
void ClosestPtPointAABB(const Vector2f& p, const AABB& b, Vector2f& q)
{
	Vector2f min = GetMin(b);
	Vector2f max = GetMax(b);

	// For each coordinate axis, if the point coordinate value is
	// outside box, clamp it to the box, else keep it as is	
	float v = p.x;
	if (v < min.x) v = min.x;
	if (v > max.x) v = max.x;
	q.x = v;

	v = p.y;
	if (v < min.y) v = min.y;
	if (v > max.y) v = max.y;
	q.y = v;
}

bool PointInAABB(const Vector2f& point, const AABB& aabb)
{
	Vector2f min = GetMin(aabb);
	Vector2f max = GetMax(aabb);

	if (point.x < min.x || point.y < min.y)
		return false;

	if (point.x > max.x || point.y > max.y)
		return false;

	return true;
}

bool PointOnPlane(const Vector2f& point, const Plane& plane)
{
	const float dot = point.dot(plane.normal);
	return CMP(dot - plane.distance, 0.0f);
}

Vector2f GetMin(const AABB& aabb)
{
	Vector2f p1 = aabb.position + aabb.size;
	Vector2f p2 = aabb.position - aabb.size;

	return Vector2f(
		fminf(p1.x, p2.x), 
		fminf(p1.y, p2.y)
	);
}

Vector2f GetMax(const AABB& aabb)
{
	Vector2f p1 = aabb.position + aabb.size;
	Vector2f p2 = aabb.position - aabb.size;

	return Vector2f(
		fmaxf(p1.x, p2.x),
		fmaxf(p1.y, p2.y)
	);
}

//AABB FromMinMax(const Vector2f& min, const Vector2f& max)
//{
//	return AABB( (min + max) * 0.5f, (max - min) * 0.5f);
//}

float PlaneEquation(const Vector2f& point, const Plane& plane)
{
	return point.dot(plane.normal) - plane.distance;
}

bool SpherePlane(const Sphere& s, const Plane& p)
{
	Vector2f closestPoint = ClosestPoint(p, s.position);
	const float distSq = (s.position - closestPoint).lengthSq();
	const float radiusSq = s.radius * s.radius;
	return distSq < radiusSq;
}

bool SphereAABB(const Sphere& sphere, const AABB& aabb)
{
	const float distSq = SqDistPointAABB(sphere.position, aabb);
	//Vector2f closestPoint = ClosestPoint(aabb, sphere.position);
	//const float distSq = (sphere.position - closestPoint).lengthSq();
	const float radiusSq = sphere.radius * sphere.radius;
	return distSq < radiusSq;
}

//void SphereCollisionWithPlane(const Sphere& sphere, const Plane& plane, ContactManifold* contactManifold)
//{
//	if (SpherePlane(sphere, plane))
//	{
//		const float BallDist = plane.normal.dot(sphere.position) - sphere.radius - plane.distance;
//
//		if (BallDist >= 0.0f) return;
//
//		ManifoldPoint point;
//		point.contactID1 = this;
//		point.contactID2_p = plane;
//		point.contactNormal = plane.normal;
//		point.depth = -BallDist;
//
//		Vector2f contact = sphere.position - plane.normal * (BallDist + sphere.radius);
//		point.contacts.push_back(contact);
//
//		contactManifold->Add(point);
//	}
//}
//
//void SphereCollisionWithTruePlane(const Sphere& sphere, const Plane& plane, ContactManifold* contactManifold)
//{
//	if (SpherePlane(sphere, plane))
//	{
//		const float centerDist = plane.normal.dot(sphere.position) - plane.distance;
//
//		if (centerDist * centerDist > sphere.radius * sphere.radius) return;
//
//		Vector2f normal = plane.normal;
//		float depth = -centerDist;
//		if (centerDist < 0)
//		{
//			normal = normal * -1;
//			depth = -depth;
//		}
//		depth += sphere.radius;
//
//		ManifoldPoint point;
//		point.contactID1 = this;
//		point.contactID2_p = plane;
//		point.contactNormal = normal;
//		point.depth = depth;
//
//		Vector2f contact = sphere.position - plane.normal * centerDist;
//		point.contacts.push_back(contact);
//
//		contactManifold->Add(point);
//	}
//}
