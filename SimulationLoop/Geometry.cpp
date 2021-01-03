#include "Geometry.h"
#include "TextureLoader.h"



math::Vector3D ClosestPoint(const Plane& plane, const math::Vector3D& point)
{
	const float dot = plane.normal.dot(point);
	const float distance = dot - plane.distance;
	return point - plane.normal * distance;
}

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

bool SphereSphere(const Sphere& A, const Sphere& B)
{
	const float radiusSum = A.radius + B.radius;
	const float sqDistance = (B.position - A.position).sizeSqr();
	return sqDistance < radiusSum * radiusSum;
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
	const float radiusSq = sphere.radius * sphere.radius;
	return distSq < radiusSq;
}

bool SphereOBB(const Sphere& sphere, const OBB& obb)
{
	math::Vector3D closestPoint = ClosestPoint(obb, sphere.position);
	const float distSq = (sphere.position - closestPoint).sizeSqr();
	const float radiusSq = sphere.radius * sphere.radius;
	return distSq < radiusSq;
}

float LengthSq(const Line& line) 
{
	return (line.start - line.end).sizeSqr();
}

bool Linetest(const OBB& obb, const Line& line) 
{
	if ((line.end - line.start).sizeSqr() < 0.0000001f) 
		return PointInOBB(line.start, obb);

	Ray ray;
	ray.origin = line.start;
	ray.direction = math::normalize(line.end - line.start);
	RaycastResult result;
	if (!Raycast(obb, ray, &result))
		return false;

	float t = result.t;

	return t >= 0 && t * t <= LengthSq(line);
}

void ResetRaycastResult(RaycastResult* outResult) {
	if (outResult != 0) 
	{
		outResult->t = -1;
		outResult->hit = false;
		outResult->normal = math::Vector3D(0, 0, 1);
		outResult->point = math::Vector3D(0, 0, 0);
	}
}

bool Raycast(const OBB& obb, const Ray& ray, RaycastResult* outResult) 
{
	ResetRaycastResult(outResult);

	float o[] = {
		obb.orientation.a(), obb.orientation.b(), obb.orientation.c(),
		obb.orientation.d(), obb.orientation.e(), obb.orientation.f(),
		obb.orientation.g(), obb.orientation.h(), obb.orientation.k()
	};

	float size[] = { obb.size.x, obb.size.y, obb.size.z };

	math::Vector3D p = obb.position - ray.origin;

	math::Vector3D X(o[0], o[1], o[2]);
	math::Vector3D Y(o[3], o[4], o[5]);
	math::Vector3D Z(o[6], o[7], o[8]);

	math::Vector3D f(
		X.dot(ray.direction),
		Y.dot(ray.direction),
		Z.dot(ray.direction)
	);

	math::Vector3D e(
		X.dot(p),
		Y.dot(p),
		Z.dot(p)
	);

#if 1
	float t[6] = { 0, 0, 0, 0, 0, 0 };
	for (int i = 0; i < 3; ++i) {
		if (CMP(f[i], 0)) {
			if (-e[i] - size[i] > 0 || -e[i] + size[i] < 0) {
				return false;
			}
			f[i] = 0.00001f; // Avoid div by 0!
		}

		t[i * 2 + 0] = (e[i] + size[i]) / f[i]; // tmin[x, y, z]
		t[i * 2 + 1] = (e[i] - size[i]) / f[i]; // tmax[x, y, z]
	}

	float tmin = fmaxf(fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])), fminf(t[4], t[5]));
	float tmax = fminf(fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])), fmaxf(t[4], t[5]));
#else 
	// The above loop simplifies the below if statements
	// this is done to make sure the sample fits into the book
	if (CMP(f.x, 0)) {
		if (-e.x - obb.size.x > 0 || -e.x + obb.size.x < 0) {
			return -1;
		}
		f.x = 0.00001f; // Avoid div by 0!
	}
	else if (CMP(f.y, 0)) {
		if (-e.y - obb.size.y > 0 || -e.y + obb.size.y < 0) {
			return -1;
		}
		f.y = 0.00001f; // Avoid div by 0!
	}
	else if (CMP(f.z, 0)) {
		if (-e.z - obb.size.z > 0 || -e.z + obb.size.z < 0) {
			return -1;
		}
		f.z = 0.00001f; // Avoid div by 0!
	}

	float t1 = (e.x + obb.size.x) / f.x;
	float t2 = (e.x - obb.size.x) / f.x;
	float t3 = (e.y + obb.size.y) / f.y;
	float t4 = (e.y - obb.size.y) / f.y;
	float t5 = (e.z + obb.size.z) / f.z;
	float t6 = (e.z - obb.size.z) / f.z;

	float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
	float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));
#endif

	// if tmax < 0, ray is intersecting AABB
	// but entire AABB is behing it's origin
	if (tmax < 0) {
		return false;
	}

	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax) {
		return false;
	}

	// If tmin is < 0, tmax is closer
	float t_result = tmin;

	if (tmin < 0.0f) {
		t_result = tmax;
	}

	if (outResult != 0) {
		outResult->hit = true;
		outResult->t = t_result;
		outResult->point = ray.origin + ray.direction * t_result;

		math::Vector3D normals[] = {
			X,			// +x
			X * -1.0f,	// -x
			Y,			// +y
			Y * -1.0f,	// -y
			Z,			// +z
			Z * -1.0f	// -z
		};

		for (int i = 0; i < 6; ++i) {
			if (CMP(t_result, t[i])) {
				outResult->normal = math::normalize(normals[i]);
			}
		}
	}
	return true;
}

#if 0

Interval GetInterval(const OBB& obb, const math::Vector3D& axis) 
{
	math::Vector3D vertex[8];

	math::Vector3D C = obb.position;	// OBB Center
	math::Vector3D E = obb.size;		// OBB Extents
	const math::Matrix3& o = obb.orientation;
	math::Vector3D A[] = {			// OBB Axis
		math::Vector3D(o.a(), o.b(), o.c()),
		math::Vector3D(o.d(), o.e(), o.f()),
		math::Vector3D(o.g(), o.h(), o.k())
	};

	vertex[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	vertex[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	vertex[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	vertex[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	vertex[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	Interval result;
	result.min = result.max = axis.dot(vertex[0]);

	for (int i = 1; i < 8; ++i) {
		float projection = axis.dot(vertex[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

std::vector<math::Vector3D> ClipEdgesToOBB(const std::vector<Line>& edges, const OBB& obb) 
{
	std::vector<math::Vector3D> result;
	result.reserve(edges.size() * 3);
	math::Vector3D intersection;

	std::vector<Plane>& planes = GetPlanes(obb);

	for (unsigned int i = 0; i < planes.size(); ++i) {
		for (unsigned int j = 0; j < edges.size(); ++j) {
			if (ClipToPlane(planes[i], edges[j], &intersection)) {
				if (PointInOBB(intersection, obb)) {
					result.push_back(intersection);
				}
			}
		}
	}

	return result;
}

bool ClipToPlane(const Plane& plane, const Line& line, math::Vector3D* outPoint) {
	math::Vector3D ab = line.end - line.start;

	float nA = plane.normal.dot(line.start);
	float nAB = plane.normal.dot(ab);

	if (CMP(nAB, 0)) {
		return false;
	}

	float t = (plane.distance - nA) / nAB;
	if (t >= 0.0f && t <= 1.0f) {
		if (outPoint != 0) {
			*outPoint = line.start + ab * t;
		}
		return true;
	}

	return false;
}

float PenetrationDepth(const OBB& o1, const OBB& o2, math::Vector3D& axis, bool* outShouldFlip)
{
	Interval i1 = GetInterval(o1, math::normalize(axis));
	Interval i2 = GetInterval(o2, math::normalize(axis));

	if (!((i2.min <= i1.max) && (i1.min <= i2.max))) 
		return 0.0f; // No penerattion

	float len1 = i1.max - i1.min;
	float len2 = i2.max - i2.min;
	float min = fminf(i1.min, i2.min);
	float max = fmaxf(i1.max, i2.max);
	float length = max - min;

	if (outShouldFlip != 0) 
		*outShouldFlip = (i2.min < i1.min);

	return (len1 + len2) - length;
}

std::vector<Plane> GetPlanes(const OBB& obb) {
	math::Vector3D c = obb.position;	// OBB Center
	math::Vector3D e = obb.size;		// OBB Extents
	const math::Matrix3& o = obb.orientation;
	math::Vector3D a[] = {			// OBB Axis
		math::Vector3D(o.a(), o.b(), o.c()),
		math::Vector3D(o.d(), o.e(), o.f()),
		math::Vector3D(o.g(), o.h(), o.k()),
	};

	std::vector<Plane> result;
	result.resize(6);

	result[0] = Plane(a[0], (a[0].dot(c + a[0] * e.x)));
	result[1] = Plane(a[0] * -1.0f, -(a[0].dot(c - a[0] * e.x)));
	result[2] = Plane(a[1], (a[1].dot(c + a[1] * e.y)));
	result[3] = Plane(a[1] * -1.0f, -(a[1].dot(c - a[1] * e.y)));
	result[4] = Plane(a[2], (a[2].dot(c + a[2] * e.z)));
	result[5] = Plane(a[2] * -1.0f, -(a[2].dot(c - a[2] * e.z)));

	return result;
}

std::vector<Line> GetEdges(const OBB& obb) {
	std::vector<Line> result;
	result.reserve(12);
	std::vector<math::Vector3D> v = GetVertices(obb);

	int index[][2] = { // Indices of edges
		{ 6, 1 },{ 6, 3 },{ 6, 4 },{ 2, 7 },{ 2, 5 },{ 2, 0 },
		{ 0, 1 },{ 0, 3 },{ 7, 1 },{ 7, 4 },{ 4, 5 },{ 5, 3 }
	};

	for (int j = 0; j < 12; ++j) {
		result.push_back(Line(
			v[index[j][0]], v[index[j][1]]
		));
	}

	return result;
}

std::vector<math::Vector3D> GetVertices(const OBB& obb) {
	std::vector<math::Vector3D> v;
	v.resize(8);

	math::Vector3D C = obb.position;	// OBB Center
	math::Vector3D E = obb.size;		// OBB Extents
	const math::Matrix3& o = obb.orientation;
	math::Vector3D A[] = {			// OBB Axis
		math::Vector3D(o.a(), o.b(), o.c()),
		math::Vector3D(o.d(), o.e(), o.f()),
		math::Vector3D(o.g(), o.h(), o.k()),
	};

	v[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	v[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	v[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	v[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	v[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	v[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	v[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	v[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	return v;
}

void SphereCollisionWithPlane(const Sphere& sphere, const Plane& plane, ContactManifold* contactManifold)
{
	if (SpherePlane(sphere, plane))
	{
		const float BallDist = plane.normal.dot(sphere.position) - sphere.radius - plane.distance;

		if (BallDist >= 0.0f) return;

		ManifoldPoint point;
		point.contactID1 = this;
		point.contactID2_p = plane;
		point.contactNormal = plane.normal;
		point.depth = -BallDist;

		math::Vector3D contact = sphere.position - plane.normal * (BallDist + sphere.radius);
		point.contacts.push_back(contact);

		contactManifold->Add(point);
	}
}

void SphereCollisionWithTruePlane(const Sphere& sphere, const Plane& plane, ContactManifold* contactManifold)
{
	if (SpherePlane(sphere, plane))
	{
		const float centerDist = plane.normal.dot(sphere.position) - plane.distance;

		if (centerDist * centerDist > sphere.radius * sphere.radius) return;

		math::Vector3D normal = plane.normal;
		float depth = -centerDist;
		if (centerDist < 0)
		{
			normal = normal * -1;
			depth = -depth;
		}
		depth += sphere.radius;

		ManifoldPoint point;
		point.contactID1 = this;
		point.contactID2_p = plane;
		point.contactNormal = normal;
		point.depth = depth;

		math::Vector3D contact = sphere.position - plane.normal * centerDist;
		point.contacts.push_back(contact);

		contactManifold->Add(point);
	}
}

#endif