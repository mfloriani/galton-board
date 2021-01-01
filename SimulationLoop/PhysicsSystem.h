#pragma once

#include "RigidBody.h"

class PhysicsSystem
{
public:
	PhysicsSystem();
	~PhysicsSystem();

	void Update(float dt);

	void AddRigidBody(RigidBody* body);
	void ClearRigidBodies();

	const std::vector<RigidBody*>& Bodies() const { return m_bodies; }

private:
	std::vector<RigidBody*>    m_bodies;
	std::vector<RigidBody*>    m_collidersA;
	std::vector<RigidBody*>    m_collidersB;
	std::vector<ManifoldPoint> m_results;

	// Smaller = more accurate [0.01 to 0.1] 
	float m_penetrationSlack{ 0.01f };
	// Smaller = less jitter / more penetration [0.2 to 0.8]
	float m_linearProjectionPercent{ 0.2f };
	// More interations more accurate [1 to 20]
	int m_impulseIteration{ 8 };

	static ManifoldPoint CheckCollision(const RigidBody& A, const RigidBody& B);
	static ManifoldPoint CheckCollision(const Sphere& A, const Sphere& B);
	static ManifoldPoint CheckCollision(const AABB& A, const Sphere& B);
	static ManifoldPoint CheckCollision(const OBB& A, const Sphere& B);

private:
	void ApplyLinearImpulse(RigidBody& A, RigidBody& B, const ManifoldPoint& P, int c);
	void AvoidSinking();

};