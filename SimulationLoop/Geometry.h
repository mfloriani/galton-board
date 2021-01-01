#pragma once

#include <vector>
#include "Vector2f.h"

#define CMP(x, y) (fabsf(x - y) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

struct ManifoldPoint
{
	bool                  colliding{ false };
	Vector2f              normal;
	float                 depth{ 0.0f };
	std::vector<Vector2f> contacts;
};

struct Plane
{
	Vector2f normal;
	float    distance;

	Plane() : normal(1, 0), distance(0) {}
	Plane(const Vector2f& n, float d) : normal(n), distance(d) {}
};

struct AABB
{
	Vector2f position;
	Vector2f size;

	AABB() : size(1.f, 1.f){}
	AABB(const Vector2f& p, const Vector2f& s): position(p), size(s){}
};

struct OBB
{
	Vector2f position;

};

struct Sphere
{
	Vector2f position;
	float    radius;

	Sphere() : radius(5) {}
	Sphere(const Vector2f& pos, float r) : position(pos), radius(r){}
};

Vector2f GetMin(const AABB& aabb);
Vector2f GetMax(const AABB& aabb);
//AABB FromMinMax(const Vector2f& min, const Vector2f& max);

bool PointInAABB(const Vector2f& point, const AABB& aabb);
bool PointOnPlane(const Vector2f& point, const Plane& plane);

//Vector2f ClosestPoint(const AABB& aabb, const Vector2f& point);
Vector2f ClosestPoint(const Plane& plane, const Vector2f& point);
float SqDistPointAABB(const Vector2f& p, const AABB& b);
void ClosestPtPointAABB(const Vector2f& p, const AABB& b, Vector2f& q);

float PlaneEquation(const Vector2f& point, const Plane& plane);

bool SpherePlane(const Sphere& s, const Plane& p);
#define PlaneSphere(plane, sphere) SpherePlane(sphere, plane)

bool SphereAABB(const Sphere& sphere, const AABB& aabb);
#define AABBSphere(aabb, sphere) SphereAABB(sphere, aabb)
