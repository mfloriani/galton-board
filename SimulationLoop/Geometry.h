#pragma once

#include <vector>
#include "Math\Vector3D.h"

#define CMP(x, y) (fabsf(x - y) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

struct ManifoldPoint
{
	bool                        colliding{ false };
	math::Vector3D              normal;
	float                       depth{ 0.0f };
	std::vector<math::Vector3D> contacts;
};

struct Plane
{
	math::Vector3D normal;
	float          distance;

	Plane() : normal(1.f, 0.f, 0.f), distance(0.f) {}
	Plane(const math::Vector3D& n, float d) : normal(n), distance(d) {}
};

struct AABB
{
	math::Vector3D position;
	math::Vector3D size;

	AABB() : size(1.f, 1.f, 0.f){}
	AABB(const math::Vector3D& p, const math::Vector3D& s): position(p), size(s){}
};

struct OBB
{
	math::Vector3D position;
	math::Vector3D size;
};

struct Sphere
{
	math::Vector3D position;
	float          radius;

	Sphere() : radius(5) {}
	Sphere(const math::Vector3D& pos, float r) : position(pos), radius(r){}
};

math::Vector3D GetMin(const AABB& aabb);
math::Vector3D GetMax(const AABB& aabb);
//AABB FromMinMax(const math::Vector3D& min, const math::Vector3D& max);

bool PointInAABB(const math::Vector3D& point, const AABB& aabb);
bool PointOnPlane(const math::Vector3D& point, const Plane& plane);

//math::Vector3D ClosestPoint(const AABB& aabb, const math::Vector3D& point);
math::Vector3D ClosestPoint(const Plane& plane, const math::Vector3D& point);
float SqDistPointAABB(const math::Vector3D& p, const AABB& b);
void ClosestPtPointAABB(const math::Vector3D& p, const AABB& b, math::Vector3D& q);

float PlaneEquation(const math::Vector3D& point, const Plane& plane);

bool SpherePlane(const Sphere& s, const Plane& p);
#define PlaneSphere(plane, sphere) SpherePlane(sphere, plane)

bool SphereAABB(const Sphere& sphere, const AABB& aabb);
#define AABBSphere(aabb, sphere) SphereAABB(sphere, aabb)
