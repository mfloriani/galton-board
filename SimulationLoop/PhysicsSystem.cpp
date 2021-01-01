#include "PhysicsSystem.h"


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
	
	// TODO: implement constraints chapter 14
	//for (auto body : m_bodies)
	//	body->SolveConstraints(m_constraints);
}

void PhysicsSystem::AddRigidBody(RigidBody* body)
{
	body->SyncCollisionVolumes();
	m_bodies.push_back(body);
}

void PhysicsSystem::ClearRigidBodies()
{ 
	for (auto b : m_bodies)
		delete b;

	m_bodies.clear();
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

	math::Vector3D relativeVel = B.velocity - A.velocity;
	math::Vector3D relativeNormal = P.normal;
	relativeNormal = relativeNormal.normalize();

	const float relativeDir = relativeVel.dot(relativeNormal);

	// Moving away from each other?
	if (relativeDir > 0.0f)
		return;

	const float e = fminf(A.restitution, B.restitution);
	float numerator = -(1.0f + e) * relativeDir;
	float j = numerator / invMassSum;
	if (P.contacts.size() > 0 && j != 0.0f)
		j /= (float)P.contacts.size();

	math::Vector3D impulse = relativeNormal * j;
	A.velocity = A.velocity - impulse * invMassA;
	B.velocity = B.velocity + impulse * invMassB;

	//
	// Friction
	//

	math::Vector3D t = relativeVel - (relativeNormal * relativeDir);
	if (CMP(t.sizeSqr(), 0.0f))
		return;

	t = t.normalize();

	numerator = -relativeVel.dot(t);
	float jt = numerator / invMassSum;

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
	//else if (A.type == VolumeType::AABB)
	//{
	//	if (B.type == VolumeType::AABB)
	//	{
	//		result = CheckCollision(A.boxVolume, B.boxVolume);
	//	}
	//	else if (B.type == VolumeType::Sphere)
	//	{
	//		result = CheckCollision(A.boxVolume, B.sphereVolume);
	//	}
	//}

	return result;
}

ManifoldPoint PhysicsSystem::CheckCollision(const Sphere& A, const Sphere& B)
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
			normal = (closestPoint - A.position).normalize();
		}
		else 
		{
			normal = (B.position - closestPoint).normalize();
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