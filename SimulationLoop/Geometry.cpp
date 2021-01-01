#include "Geometry.h"
#include "TextureLoader.h"

math::Vector3D ClosestPoint(const Plane& plane, const math::Vector3D& point)
{
	const float dot = plane.normal.dot(point);
	const float distance = dot - plane.distance;
	return point - plane.normal * distance;
}

//math::Vector3D ClosestPoint(const AABB& aabb, const math::Vector3D& point)
//{
//	math::Vector3D result = point;
//	math::Vector3D min = GetMin(aabb);
//	math::Vector3D max = GetMax(aabb);
//
//	result.x = (result.x < min.x) ? min.x : result.x;
//	result.y = (result.y < min.x) ? min.y : result.y;
//
//	result.x = (result.x > max.x) ? max.x : result.x;
//	result.y = (result.y > max.x) ? max.y : result.y;
//
//	return result;
//}

float SqDistPointAABB(const math::Vector3D& p, const AABB& b)
{
	math::Vector3D min = GetMin(b);
	math::Vector3D max = GetMax(b);

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
void ClosestPtPointAABB(const math::Vector3D& p, const AABB& b, math::Vector3D& q)
{
	math::Vector3D min = GetMin(b);
	math::Vector3D max = GetMax(b);

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

bool PointInAABB(const math::Vector3D& point, const AABB& aabb)
{
	math::Vector3D min = GetMin(aabb);
	math::Vector3D max = GetMax(aabb);

	if (point.x < min.x || point.y < min.y)
		return false;

	if (point.x > max.x || point.y > max.y)
		return false;

	return true;
}

bool PointOnPlane(const math::Vector3D& point, const Plane& plane)
{
	const float dot = point.dot(plane.normal);
	return CMP(dot - plane.distance, 0.0f);
}

bool PointInOBB(const math::Vector3D& point, const OBB& obb)
{
	math::Vector3D dir = point - obb.position;
	
	// X-AXIS
	math::Vector3D Xaxis(obb.orientation.a(), obb.orientation.b(), obb.orientation.c());
	float distance = dir.dot(Xaxis);
	if (distance > obb.size.x) 
		return false;
	if (distance < -obb.size.x)
		return false;

	// Y-AXIS
	math::Vector3D Yaxis(obb.orientation.d(), obb.orientation.e(), obb.orientation.f());
	distance = dir.dot(Yaxis);
	if (distance > obb.size.y)
		return false;
	if (distance < -obb.size.y)
		return false;

	// Z-AXIS
	math::Vector3D Zaxis(obb.orientation.g(), obb.orientation.h(), obb.orientation.k());
	distance = dir.dot(Zaxis);
	if (distance > obb.size.z)
		return false;
	if (distance < -obb.size.z)
		return false;


	return true;
}

math::Vector3D ClosestPoint(const OBB& obb, const math::Vector3D& point)
{
	math::Vector3D result = obb.position;
	math::Vector3D dir = point - obb.position;

	// X-AXIS
	math::Vector3D Xaxis(obb.orientation.a(), obb.orientation.b(), obb.orientation.c());
	float distance = dir.dot(Xaxis);
	if (distance > obb.size.x)
		distance = obb.size.x;
	if (distance < -obb.size.x)
		distance = -obb.size.x;
	result += (Xaxis * distance);

	// Y-AXIS
	math::Vector3D Yaxis(obb.orientation.d(), obb.orientation.e(), obb.orientation.f());
	distance = dir.dot(Yaxis);
	if (distance > obb.size.y)
		distance = obb.size.y;
	if (distance < -obb.size.y)
		distance = -obb.size.y;
	result += (Yaxis * distance);

	// Z-AXIS
	math::Vector3D Zaxis(obb.orientation.g(), obb.orientation.h(), obb.orientation.k());
	distance = dir.dot(Zaxis);
	if (distance > obb.size.z)
		distance = obb.size.z;
	if (distance < -obb.size.z)
		distance = -obb.size.z;
	result += (Zaxis * distance);

	return result;
}

// Given point p, return point q on (or in) OBB b, closest to p
//void ClosestPtPointOBB(const math::Vector3D& p, const OBB& b, math::Vector3D& q)
//{
//	math::Vector3D d = p - b.position;
//	// Start result at center of box; make steps from there
//	q = b.position;
//	// For each OBB axis...
//	for (int i = 0; i < 3; i++) {
//		// ...project d onto that axis to get the distance
//		// along the axis of d from the box center
//		float dist = Dot(d, b.u[i]);
//		// If distance farther than the box extents, clamp to the box
//		if (dist > b.e[i]) dist = b.e[i];
//		if (dist < -b.e[i]) dist = -b.e[i];
//		// Step that distance along the axis to get world coordinate
//		q += dist * b.u[i];
//	}
//}

// Computes the square distance between point p and OBB b
//float SqDistPointOBB(Point p, OBB b)
//{
//	Point closest;
//	ClosestPtPointOBB(p, b, closest);
//	float sqDist = Dot(closest - p, closest - p);
//	return sqDist;
//}

// Computes the square distance between point p and OBB b
//float SqDistPointOBB(Point p, OBB b)
//{
//	Vector v = p - b.c;
//	float sqDist = 0.0f;
//	for (int i = 0; i < 3; i++) {
//		// Project vector from box center to p on each axis, getting the distance
//		// of p along that axis, and count any excess distance outside box extents
//		float d = Dot(v, b.u[i]), excess = 0.0f;
//		if (d < -b.e[i])
//			excess = d + b.e[i];
//		else if (d > b.e[i])
//			excess = d - b.e[i];
//		sqDist += excess * excess;
//	}
//	return sqDist;
//}

math::Vector3D GetMin(const AABB& aabb)
{
	math::Vector3D p1 = aabb.position + aabb.size;
	math::Vector3D p2 = aabb.position - aabb.size;

	return math::Vector3D(
		fminf(p1.x, p2.x), 
		fminf(p1.y, p2.y),
		0.f
	);
}

math::Vector3D GetMax(const AABB& aabb)
{
	math::Vector3D p1 = aabb.position + aabb.size;
	math::Vector3D p2 = aabb.position - aabb.size;

	return math::Vector3D(
		fmaxf(p1.x, p2.x),
		fmaxf(p1.y, p2.y),
		0.f
	);
}

//AABB FromMinMax(const math::Vector3D& min, const math::Vector3D& max)
//{
//	return AABB( (min + max) * 0.5f, (max - min) * 0.5f);
//}

float PlaneEquation(const math::Vector3D& point, const Plane& plane)
{
	return point.dot(plane.normal) - plane.distance;
}

bool SpherePlane(const Sphere& s, const Plane& p)
{
	math::Vector3D closestPoint = ClosestPoint(p, s.position);
	const float distSq = (s.position - closestPoint).sizeSqr();
	const float radiusSq = s.radius * s.radius;
	return distSq < radiusSq;
}

bool SphereAABB(const Sphere& sphere, const AABB& aabb)
{
	const float distSq = SqDistPointAABB(sphere.position, aabb);
	//math::Vector3D closestPoint = ClosestPoint(aabb, sphere.position);
	//const float distSq = (sphere.position - closestPoint).lengthSq();
	const float radiusSq = sphere.radius * sphere.radius;
	return distSq < radiusSq;
}

bool SphereOBB(const Sphere& sphere, const OBB& obb)
{
	math::Vector3D closestPoint = ClosestPoint(obb, sphere.position);

	float distSq = (sphere.position - closestPoint).sizeSqr();
	float radiusSq = sphere.radius * sphere.radius;

	return distSq < radiusSq;
}


// Returns true if sphere s intersects OBB b, false otherwise.
// The point p on the OBB closest to the sphere center is also returned
//int TestSphereOBB(Sphere s, OBB b, Point& p)
//{
//	// Find point p on OBB closest to sphere center
//	ClosestPtPointOBB(s.c, b, p);
//	// Sphere and OBB intersect if the (squared) distance from sphere
//	// center to point p is less than the (squared) sphere radius
//	Vector v = p - s.c;
//	return Dot(v, v) <= s.r * s.r;
//}


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
//		math::Vector3D contact = sphere.position - plane.normal * (BallDist + sphere.radius);
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
//		math::Vector3D normal = plane.normal;
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
//		math::Vector3D contact = sphere.position - plane.normal * centerDist;
//		point.contacts.push_back(contact);
//
//		contactManifold->Add(point);
//	}
//}
