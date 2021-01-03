#include "PhysicsSystem.h"
//#include "Math\Vector3D.h"
#include "Geometry.h"



PhysicsSystem::PhysicsSystem()
{
	m_bodies.reserve(200);
	m_collidersA.reserve(200);
	m_collidersB.reserve(200);
	m_results.reserve(200);
}

PhysicsSystem::~PhysicsSystem()
{
	ClearRigidBodies();
}

void PhysicsSystem::Update(float dt)
{
	m_collidersA.clear();
	m_collidersB.clear();
	m_results.clear();

	for (int i = 0, size = m_bodies.size(); i < size; ++i)
	{
		for (int j = i+1; j < size; ++j)
		{
			ManifoldPoint result;

			RigidBody* rb1 = m_bodies[i];
			RigidBody* rb2 = m_bodies[j];

			result = CheckCollision(*rb1, *rb2);

			if (result.colliding)
			{
				m_collidersA.push_back(m_bodies[i]);
				m_collidersB.push_back(m_bodies[j]);
				m_results.push_back(result);
			}
		}
	}

	for (auto body : m_bodies)
		body->ApplyForces();

	for (int k = 0; k < m_impulseIteration; ++k)
	{
		for (unsigned int i = 0; i < m_results.size(); ++i)
		{
			const unsigned int jSize = m_results[i].contacts.size();
			for (unsigned int j = 0; j < jSize; ++j)
			{
				RigidBody* rb1 = m_collidersA[i];
				RigidBody* rb2 = m_collidersB[i];
				ApplyLinearImpulse(*rb1, *rb2, m_results[i], j);
			}
		}
	}

	for (auto body : m_bodies)
		body->Update(dt);

	// Linear projection to avoid sinking
	AvoidSinking();
	
	for (auto body : m_bodies)
		body->SolveConstraints(m_constraints);


}

void PhysicsSystem::AddRigidBody(RigidBody* body)
{
	body->SyncCollisionVolumes();
	m_bodies.push_back(body);
}

void PhysicsSystem::AddConstraint(OBB& constraint)
{
	m_constraints.push_back(constraint);
}

void PhysicsSystem::ClearRigidBodies()
{ 
	for (auto b : m_bodies)
		delete b;

	m_bodies.clear();
}

void PhysicsSystem::ClearConstraints()
{
	m_constraints.clear();
}

void PhysicsSystem::Reset()
{
	ClearRigidBodies();
	ClearConstraints();
}

void PhysicsSystem::UpdateBallSize(float ballSize)
{
	for (auto b : m_bodies)
		b->sphereVolume.radius = ballSize;
}

void PhysicsSystem::UpdateRestitution(float restitution)
{
	for (auto b : m_bodies)
		b->restitution = restitution;
}

void PhysicsSystem::UpdateFriction(float friction)
{
	for (auto b : m_bodies)
		b->friction = friction;
}

void PhysicsSystem::AvoidSinking()
{
	for (unsigned int i = 0; i < m_results.size(); ++i)
	{
		RigidBody* rb1 = m_collidersA[i];
		RigidBody* rb2 = m_collidersB[i];

		const float invMassSum = rb1->InverseMass() + rb2->InverseMass();

		if (invMassSum == 0.0f)
			continue;

		float depth = fmaxf(m_results[i].depth - m_penetrationSlack, 0.0f);
		float scalar = depth / invMassSum;
		math::Vector3D correction = m_results[i].normal * scalar * m_linearProjectionPercent;

		rb1->position = rb1->position - correction * rb1->InverseMass();
		rb2->position = rb2->position + correction * rb2->InverseMass();

		rb1->SyncCollisionVolumes();
		rb2->SyncCollisionVolumes();
	}
}

void PhysicsSystem::ApplyLinearImpulse(RigidBody& A, RigidBody& B, const ManifoldPoint& P, int c)
{
	const float invMassA = A.InverseMass();
	const float invMassB = B.InverseMass();
	const float invMassSum = invMassA + invMassB;

	if (invMassSum == 0.0f)
		return;

#ifdef ENABLE_ANGULAR
	math::Vector3D r1 = P.contacts[c] - A.position;
	math::Vector3D r2 = P.contacts[c] - B.position;
	math::Matrix4 i1 = A.InverseTensor();
	math::Matrix4 i2 = B.InverseTensor();
#endif

#ifdef ENABLE_ANGULAR
	math::Vector3D relativeVel = (B.velocity + math::cross(B.angularVel, r2)) - (A.velocity + math::cross(A.angularVel, r1));
#else
	math::Vector3D relativeVel = B.velocity - A.velocity;
#endif
	math::Vector3D relativeNormal = P.normal;
	relativeNormal = relativeNormal.normalize();

	const float relativeDir = relativeVel.dot(relativeNormal);

	// Moving away from each other?
	if (relativeDir > 0.0f)
		return;

	const float e = fminf(A.restitution, B.restitution);
	float numerator = -(1.0f + e) * relativeDir;
	float d1 = invMassSum;
#ifdef ENABLE_ANGULAR
	math::Vector3D d2 = math::cross(math::multiplyVector(math::cross(r1, relativeNormal), i1), r1);
	math::Vector3D d3 = math::cross(math::multiplyVector(math::cross(r2, relativeNormal), i2), r2);
	float denominator = d1 + relativeNormal.dot(d2 + d3);
#else
	float denominator = d1;
#endif

	float j = denominator == 0.f ? 0.f : numerator / denominator;
	if (P.contacts.size() > 0 && j != 0.0f)
		j /= (float)P.contacts.size();

	math::Vector3D impulse = relativeNormal * j;
	A.velocity = A.velocity - impulse * invMassA;
	B.velocity = B.velocity + impulse * invMassB;

#ifdef ENABLE_ANGULAR
	A.angularVel -= math::multiplyVector(math::cross(r1, impulse), i1);
	B.angularVel += math::multiplyVector(math::cross(r2, impulse), i2);
#endif
	
	//
	// Friction
	//

	math::Vector3D t = relativeVel - (relativeNormal * relativeDir);
	if (CMP(t.sizeSqr(), 0.0f))
		return;

	t = t.normalize();

	numerator = -relativeVel.dot(t);
	d1 = invMassSum;
#ifdef ENABLE_ANGULAR
	d2 = math::cross(math::multiplyVector(math::cross(r1, t), i1), r1);
	d3 = math::cross(math::multiplyVector(math::cross(r2, t), i2), r2);
	denominator = d1 + t.dot(d2 + d3);
#else
	denominator = d1;
#endif
	if (denominator == 0.f)
		return;

	float jt = numerator / denominator;

	if (P.contacts.size() > 0 && jt != 0.0f)
		jt /= (float)P.contacts.size();

	if (CMP(jt, 0.0f))
		return;

	const float friction = sqrtf(A.friction * B.friction);
	if (jt > j * friction)
		jt = j * friction;
	else if (jt < -j * friction)
		jt = -j * friction;

	math::Vector3D tangentImpulse = t * jt;
	A.velocity = A.velocity - tangentImpulse * invMassA;
	B.velocity = B.velocity + tangentImpulse * invMassB;
#ifdef ENABLE_ANGULAR
	A.angularVel -= math::multiplyVector(math::cross(r1, tangentImpulse), i1);
	B.angularVel -= math::multiplyVector(math::cross(r2, tangentImpulse), i2);
#endif
}

ManifoldPoint PhysicsSystem::CheckCollision(const RigidBody& A, const RigidBody& B)
{
	ManifoldPoint result;

	if (A.type == VolumeType::Sphere)
	{
		if (B.type == VolumeType::Sphere)
		{
			result = CheckCollision(A.sphereVolume, B.sphereVolume);
		}
		else if (B.type == VolumeType::AABB)
		{
			result = CheckCollision(B.aabbVolume, A.sphereVolume);
		}
		else if (B.type == VolumeType::OBB)
		{
			result = CheckCollision(B.obbVolume, A.sphereVolume);
			result.normal = result.normal * -1.0f;
		}
	}
	//else if (A.type == VolumeType::OBB)
	//{
	//	if (B.type == VolumeType::OBB)
	//	{
	//		result = CheckCollision(A.obbVolume, B.obbVolume);
	//	}
	//	else if (B.type == VolumeType::Sphere)
	//	{
	//		result = CheckCollision(A.obbVolume, B.sphereVolume);
	//	}
	//}

	return result;
}

ManifoldPoint PhysicsSystem::CheckCollision(const Sphere& A, const Sphere& B)
{
	if (SphereSphere(A, B))
	{
		ManifoldPoint result;

		const float r = A.radius + B.radius;
		math::Vector3D d = B.position - A.position;

		if (d.sizeSqr() - r * r > 0 || d.sizeSqr() == 0.0f)
			return result;

		d = d.normalize();

		const float depth = fabsf(d.size() - r) * 0.5f;

		result.colliding = true;
		result.normal = d;
		result.depth = depth;

		const float dtp = A.radius - depth;
		math::Vector3D contact = A.position + d * dtp;
		result.contacts.push_back(contact);

		return result;
	}
	return ManifoldPoint();
}

ManifoldPoint PhysicsSystem::CheckCollision(const AABB& A, const Sphere& B)
{
	if (AABBSphere(A, B))
	{
		math::Vector3D closestPoint;
		ClosestPtPointAABB(B.position, A, closestPoint);
		
		const float distSq = closestPoint.dot(closestPoint);

		math::Vector3D d = closestPoint - B.position;
		d = d.normalize();

		const float depth = fabsf((closestPoint - B.position).size() - B.radius);

		ManifoldPoint result;
		result.colliding = true;
		result.normal = d;
		result.depth = depth;

		const float dtp = B.radius - depth;
		math::Vector3D contact = B.position + d * dtp;
		result.contacts.push_back(contact);

		return result;
	}
	return ManifoldPoint();
}

ManifoldPoint PhysicsSystem::CheckCollision(const OBB& A, const Sphere& B)
{
	if (SphereOBB(B, A))
	{
		ManifoldPoint result;

		math::Vector3D closestPoint = ClosestPoint(A, B.position);

		const float distanceSq = (closestPoint - B.position).sizeSqr();
		if (distanceSq > B.radius * B.radius)
			return result;

		math::Vector3D normal;
		if (CMP(distanceSq, 0.0f)) 
		{
			if (CMP((closestPoint - A.position).sizeSqr(), 0.0f)) 
				return result;

			// Closest point is at the center of the sphere
			normal = math::normalize(closestPoint - A.position);
		}
		else 
		{
			normal = math::normalize(B.position - closestPoint);
		}

		math::Vector3D outsidePoint = B.position - normal * B.radius;

		const float distance = (closestPoint - outsidePoint).size();

		result.colliding = true;
		result.contacts.push_back(closestPoint + (outsidePoint - closestPoint) * 0.5f);
		result.normal = normal;
		result.depth = distance * 0.5f;

		return result;
	}
	return ManifoldPoint();
}

#if 0
ManifoldPoint PhysicsSystem::CheckCollision(const OBB& A, const OBB& B)
{
	ManifoldPoint result;

	Sphere s1(A.position, A.size.size());
	Sphere s2(B.position, B.size.size());

	if (!SphereSphere(s1, s2)) 
		return result;

	const math::Matrix3& o1 = A.orientation;
	const math::Matrix3& o2 = B.orientation;

	math::Vector3D test[15] = 
	{
		math::Vector3D(o1.a(), o1.b(), o1.c()),
		math::Vector3D(o1.d(), o1.e(), o1.f()),
		math::Vector3D(o1.g(), o1.h(), o1.k()),
		math::Vector3D(o2.a(), o2.b(), o2.c()),
		math::Vector3D(o2.d(), o2.e(), o2.f()),
		math::Vector3D(o2.g(), o2.h(), o2.k())
	};

	for (int i = 0; i < 3; ++i) 
	{
		test[6 + i * 3 + 0] = math::cross(test[i], test[0]);
		test[6 + i * 3 + 1] = math::cross(test[i], test[1]);
		test[6 + i * 3 + 2] = math::cross(test[i], test[2]);
	}

	math::Vector3D* hitNormal = 0;
	bool shouldFlip;

	for (int i = 0; i < 15; ++i) 
	{
		if (test[i].x < 0.000001f) test[i].x = 0.0f;
		if (test[i].y < 0.000001f) test[i].y = 0.0f;
		if (test[i].z < 0.000001f) test[i].z = 0.0f;
		if (test[i].sizeSqr() < 0.001f)
			continue;

		float depth = PenetrationDepth(A, B, test[i], &shouldFlip);
		if (depth <= 0.0f) {
			return result;
		}
		else if (depth < result.depth) {
			if (shouldFlip) {
				test[i] = test[i] * -1.0f;
			}
			result.depth = depth;
			hitNormal = &test[i];
		}
	}

	if (hitNormal == 0)
		return result;

	math::Vector3D axis = math::normalize((*hitNormal));

	std::vector<math::Vector3D> c1 = ClipEdgesToOBB(GetEdges(B), A);
	std::vector<math::Vector3D> c2 = ClipEdgesToOBB(GetEdges(A), B);
	result.contacts.reserve(c1.size() + c2.size());
	result.contacts.insert(result.contacts.end(), c1.begin(), c1.end());
	result.contacts.insert(result.contacts.end(), c2.begin(), c2.end());

	Interval i = GetInterval(A, axis);
	float distance = (i.max - i.min) * 0.5f - result.depth * 0.5f;
	math::Vector3D pointOnPlane = A.position + axis * distance;

	for (int i = result.contacts.size() - 1; i >= 0; --i) 
	{
		math::Vector3D contact = result.contacts[i];
		result.contacts[i] = contact + (axis * axis.dot(pointOnPlane - contact));

		// This bit is in the "There is more" section of the book
		for (int j = result.contacts.size() - 1; j > i; --j) {
			if ((result.contacts[j] - result.contacts[i]).sizeSqr() < 0.0001f) {
				result.contacts.erase(result.contacts.begin() + j);
				break;
			}
		}
	}

	result.colliding = true;
	result.normal = axis;

	return result;
}

#endif