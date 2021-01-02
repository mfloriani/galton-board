#pragma once

#include <vector>
#include "Math\Vector3D.h"
#include "Math\Matrix3.h"
#include "Utils.h"

//#define ENABLE_ANGULAR

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

	inline Plane() : normal(1, 0, 0) { }
	inline Plane(const math::Vector3D& n, float d) :
		normal(n), distance(d) { }
};

struct AABB
{
	math::Vector3D position;
	math::Vector3D size;

	AABB() : size(1.f, 1.f, 1.f){}
	AABB(const math::Vector3D& p, const math::Vector3D& s): position(p), size(s){}
};

struct OBB
{
	math::Vector3D position;
	math::Vector3D size;
	math::Matrix3  orientation;

	OBB() : size(1.f, 1.f, 1.f) {}
	OBB(const math::Vector3D& p, const math::Vector3D& s) : position(p), size(s){}
	OBB(const math::Vector3D& p, const math::Vector3D& s, const math::Matrix3& o) : position(p), size(s), orientation(o){}
};

struct Sphere
{
	math::Vector3D position;
	float          radius;

	Sphere() : radius(5) {}
	Sphere(const math::Vector3D& pos, float r) : position(pos), radius(r){}
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

#if 0

float PenetrationDepth(const OBB& o1, const OBB& o2, math::Vector3D& axis, bool* outShouldFlip);
Interval GetInterval(const OBB& obb, const math::Vector3D& axis);
std::vector<math::Vector3D> ClipEdgesToOBB(const std::vector<Line>& edges, const OBB& obb);
std::vector<math::Vector3D> GetVertices(const OBB& obb);
std::vector<Line> GetEdges(const OBB& obb);
bool ClipToPlane(const Plane& plane, const Line& line, math::Vector3D* outPoint);
std::vector<Plane> GetPlanes(const OBB& obb);

#endif
