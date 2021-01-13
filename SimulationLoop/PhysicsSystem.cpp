#include "PhysicsSystem.h"
#include "Geometry.h"
#include "Constants.h"

#include <cassert>

PhysicsSystem::PhysicsSystem()
{
	m_bodies.reserve(500);
	m_collidersA.reserve(500);
	m_collidersB.reserve(500);
	m_results.reserve(500);
	m_staticBodies.reserve(500);

	m_quadTree = std::make_unique<QuadTree>(
		Rectangle2D(math::Vector2D(0,0), math::Vector2D(WND_WIDTH, WND_HEIGHT))
	);
}

PhysicsSystem::~PhysicsSystem()
{
	ClearRigidBodies();
	ClearStaticRigidBodies();
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

#ifndef CONSTRAINT_BOARD
	for (auto dBody : m_bodies)
	{
		for (auto sBody : m_staticBodies)
		{
			ManifoldPoint result;
			result = CheckCollision(*dBody, *sBody);
			if (result.colliding)
			{
				m_collidersA.push_back(dBody);
				m_collidersB.push_back(sBody);
				m_results.push_back(result);
			}
		}
	}
#endif // !CONSTRAINT_BOARD

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
				RigidBody::ApplyImpulse(*rb1, *rb2, m_results[i], j);
			}
		}
	}

	for (auto body : m_bodies)
		body->Update(dt);

	// Linear projection to avoid sinking
	AvoidSinking();
	
#ifdef CONSTRAINT_BOARD
	for (auto body : m_bodies)
		body->SolveConstraints(m_constraints);
#endif

}

void PhysicsSystem::AddRigidBody(RigidBody* body)
{
	body->SyncCollisionVolumes();

	//InsertQuadTree(body); // insert the body into quad tree and bound it to the body

	//auto quatTreeData = InsertQuadTree(body);
	//body->quadTreeDataIndex = quatTreeData;

	// map the rigidbody id with quad tree data
	//m_rBodyQTD.insert(std::make_pair(body->id, quatTreeData));
	
	m_bodies.push_back(body);
}

void PhysicsSystem::AddStaticRigidBody(RigidBody* body)
{
	body->SyncCollisionVolumes();

	//InsertQuadTree(body); // insert the body into quad tree and bound it to the body

	//auto quatTreeData = InsertQuadTree(body);
	//body->quadTreeDataIndex = quatTreeData;

	// map the rigidbody id with quad tree data
	//m_rBodyQTD.insert(std::make_pair(body->id, quatTreeData));

	m_staticBodies.push_back(body);
}

#ifdef CONSTRAINT_BOARD
void PhysicsSystem::AddConstraint(OBB& constraint)
{
	m_constraints.push_back(constraint);
}
#endif

void PhysicsSystem::ClearRigidBodies()
{ 
	for (auto b : m_bodies)
		delete b;

	m_bodies.clear();
}

void PhysicsSystem::ClearStaticRigidBodies()
{
	for (auto b : m_staticBodies)
		delete b;

	m_staticBodies.clear();
}

#ifdef CONSTRAINT_BOARD
void PhysicsSystem::ClearConstraints()
{
	m_constraints.clear();
}
#endif

void PhysicsSystem::Reset()
{
	ClearRigidBodies();
	ClearStaticRigidBodies();

	m_collidersA.clear();
	m_collidersB.clear();
	m_results.clear();

#ifdef CONSTRAINT_BOARD
	ClearConstraints();
#endif
}

void PhysicsSystem::UpdateBallSize(float ballSize)
{
	for (auto b : m_bodies)
		b->sphereVolume.radius = ballSize;
}

void PhysicsSystem::UpdateRestitution(float restitution)
{
	for (auto db : m_bodies)
		db->restitution = restitution;

	for (auto sb : m_staticBodies)
		sb->restitution = restitution;
}

void PhysicsSystem::UpdateFriction(float friction)
{
	for (auto db : m_bodies)
		db->friction = friction;

	for (auto sb : m_staticBodies)
		sb->friction = friction;
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

size_t PhysicsSystem::InsertQuadTree(RigidBody* body)
{
	//get a index inside the vector to assign to the rigid body
	m_quadTreeData.push_back(QuadTreeData());
	auto qtdIndex = m_quadTreeData.size()-1;
	QuadTreeData* quadTreeData = &m_quadTreeData[qtdIndex];
	
	// bound qtd and body
	quadTreeData->object = body;
	body->quadTreeData = quadTreeData;

	if (body->type == VolumeType::Sphere)
	{
		quadTreeData->bounds = Rectangle2D(
			math::Vector2D(body->position.x, body->position.y),
			math::Vector2D(body->sphereVolume.radius, body->sphereVolume.radius));

		m_quadTree->Insert(*quadTreeData);
	}
	else if (body->type == VolumeType::AABB)
	{
		quadTreeData->bounds = Rectangle2D(
			math::Vector2D(body->position.x, body->position.y),
			math::Vector2D(body->aabbVolume.size.x, body->aabbVolume.size.y));

		m_quadTree->Insert(*quadTreeData);
	}
	else if (body->type == VolumeType::OBB)
	{
		quadTreeData->bounds = Rectangle2D(
			math::Vector2D(body->position.x, body->position.y),
			math::Vector2D(body->obbVolume.size.x, body->obbVolume.size.y));
		
		m_quadTree->Insert(*quadTreeData);
	}
	else
	{
		assert(true && "Invalid volume type PhysicsSystem::InsertQuadTree()");
	}

	return qtdIndex;
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
	else if (A.type == VolumeType::AABB)
	{
		if (B.type == VolumeType::Sphere)
		{
			result = CheckCollision(A.aabbVolume, B.sphereVolume);
			result.normal = result.normal * -1.0f;
		}
	}
	else if (A.type == VolumeType::OBB)
	{	
		if (B.type == VolumeType::Sphere)
		{
			result = CheckCollision(A.obbVolume, B.sphereVolume);
		}
		//else if (B.type == VolumeType::OBB)
		//{
		//	result = CheckCollision(A.obbVolume, B.obbVolume);
		//}
	}

	return result;
}

ManifoldPoint PhysicsSystem::CheckCollision(const Sphere& A, const Sphere& B)
{
	//if (SphereSphere(A, B))
	{
		ManifoldPoint result;

		const float r = A.radius + B.radius;
		math::Vector3D d = B.position - A.position;

		if (d.sizeSqr() - r * r > 0 || d.sizeSqr() == 0.0f)
			return result;


		const float depth = fabsf(d.size() - r) * 0.5f;
		d = math::normalize(d);

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