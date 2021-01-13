#pragma once

#include <vector>
#include "Math\Vector3D.h"
#include "Math\Matrix3.h"
#include "Utils.h"
#include "Geometry2D.h"

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
	float distance;

	inline Plane() 
		: normal(1, 0, 0) { }
	inline Plane(const math::Vector3D& n, float d) 
		: normal(n), distance(d) { }
};

struct AABB
{
	math::Vector3D position;
	math::Vector3D size;
	math::Vector3D color;
	Rectangle2D    bounds;

	AABB() 
		: size(1.f, 1.f, 1.f){}
	AABB(const math::Vector3D& p, const math::Vector3D& s) 
		: position(p), size(s)
	{
		FillRect2dFrom3d(bounds, p, s);
	}
	AABB(const math::Vector3D& p, const math::Vector3D& s, const math::Vector3D c)
		: position(p), size(s), color(c)
	{
		FillRect2dFrom3d(bounds, p, s);
	}
};

struct OBB
{
	math::Vector3D position;
	math::Vector3D size;
	math::Matrix3  orientation;
	math::Vector3D color;
	Rectangle2D    bounds;

	OBB() 
		: size(1.f, 1.f, 1.f) {}
	OBB(const math::Vector3D& p, const math::Vector3D& s) 
		: position(p), size(s)
	{
		 FillRect2dFrom3d(bounds, p, s);
	}
	OBB(const math::Vector3D& p, const math::Vector3D& s, const math::Matrix3& o) 
		: position(p), size(s), orientation(o)
	{
		FillRect2dFrom3d(bounds, p, s);
	}
	OBB(const math::Vector3D& p, const math::Vector3D& s, const math::Matrix3& o, const math::Vector3D c) 
		: position(p), size(s), orientation(o), color(c)
	{
		FillRect2dFrom3d(bounds, p, s);
	}
};

struct Sphere
{
	math::Vector3D position;
	float          radius;
	math::Vector3D color;
	Rectangle2D    bounds;

	Sphere() 
		: radius(5) {}
	Sphere(const math::Vector3D& p, float r)
		: position(p), radius(r)
	{
		FillRect2dFrom3d(bounds, p, r);
	}
	Sphere(const math::Vector3D& p, float r, const math::Vector3D c)
		: position(p), radius(r), color(c)
	{
		FillRect2dFrom3d(bounds, p, r);
	}
};

struct Interval 
{
	float min;
	float max;
};

struct Line 
{
	math::Vector3D start;
	math::Vector3D end;

	inline Line() {}
	inline Line(const math::Vector3D& s, const math::Vector3D& e) :
		start(s), end(e) { }
};

struct Ray 
{
	math::Vector3D origin;
	math::Vector3D direction;

	inline Ray() : direction(0.0f, 0.0f, 1.0f) {}
	inline Ray(const math::Vector3D& o, const math::Vector3D& d) :
		origin(o), direction(d) {
		NormalizeDirection();
	}
	inline void NormalizeDirection() {
		math::normalize(direction);
	}
};

struct RaycastResult 
{
	math::Vector3D point;
	math::Vector3D normal;
	float t;
	bool hit;
};

math::Vector3D GetMin(const AABB& aabb);
math::Vector3D GetMax(const AABB& aabb);
//AABB FromMinMax(const math::Vector3D& min, const math::Vector3D& max);

bool PointInAABB(const math::Vector3D& point, const AABB& aabb);
bool PointOnPlane(const math::Vector3D& point, const Plane& plane);
bool PointInOBB(const math::Vector3D& point, const OBB& obb);

math::Vector3D ClosestPoint(const OBB& obb, const math::Vector3D& point);
math::Vector3D ClosestPoint(const Plane& plane, const math::Vector3D& point);
float SqDistPointAABB(const math::Vector3D& p, const AABB& b);
void ClosestPtPointAABB(const math::Vector3D& p, const AABB& b, math::Vector3D& q);

float PlaneEquation(const math::Vector3D& point, const Plane& plane);

bool SphereSphere(const Sphere& s1, const Sphere& s2);

bool SpherePlane(const Sphere& s, const Plane& p);
#define PlaneSphere(plane, sphere) SpherePlane(sphere, plane)

bool SphereAABB(const Sphere& sphere, const AABB& aabb);
#define AABBSphere(aabb, sphere) SphereAABB(sphere, aabb)

bool SphereOBB(const Sphere& sphere, const OBB& obb);
#define OBBSphere(obb, sphere) SphereOBB(sphere, obb)


bool Linetest(const OBB& obb, const Line& line);

bool Raycast(const OBB& obb, const Ray& ray, RaycastResult* outResult);










#if 1

float PenetrationDepth(const OBB& o1, const OBB& o2, math::Vector3D& axis, bool* outShouldFlip);
Interval GetInterval(const OBB& obb, const math::Vector3D& axis);
std::vector<math::Vector3D> ClipEdgesToOBB(const std::vector<Line>& edges, const OBB& obb);
std::vector<math::Vector3D> GetVertices(const OBB& obb);
std::vector<Line> GetEdges(const OBB& obb);
bool ClipToPlane(const Plane& plane, const Line& line, math::Vector3D* outPoint);
std::vector<Plane> GetPlanes(const OBB& obb);

#endif
