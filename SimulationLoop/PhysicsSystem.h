#pragma once

#include "RigidBody.h"

class PhysicsSystem
{
public:
	PhysicsSystem();
	~PhysicsSystem();

	void Update(float dt);

	void AddRigidBody(RigidBody* body);
	void AddConstraint(OBB& constraint);
	
	void ClearRigidBodies();
	void ClearConstraints();

	void Reset();

	const std::vector<RigidBody*>& Bodies() const { return m_bodies; }
	const std::vector<OBB>& Constraints() const { return m_constraints; }

	void UpdateBallSize(float ballSize);
	void UpdateRestitution(float restitution);
	void UpdateFriction(float friction);

private:
	std::vector<RigidBody*>    m_bodies;
	std::vector<RigidBody*>    m_collidersA;
	std::vector<RigidBody*>    m_collidersB;
	std::vector<ManifoldPoint> m_results;
	std::vector<OBB>           m_constraints;

	// Smaller = more accurate [0.01 to 0.1] 
	float m_penetrationSlack{ 0.01f };
	// Smaller = less jitter / more penetration [0.2 to 0.8]
	float m_linearProjectionPercent{ 0.2f };
	// More interations more accurate [1 to 20]
	int m_impulseIteration{ 20 };

	static ManifoldPoint CheckCollision(const RigidBody& A, const RigidBody& B);
	static ManifoldPoint CheckCollision(const Sphere& A, const Sphere& B);
	static ManifoldPoint CheckCollision(const AABB& A, const Sphere& B);
	static ManifoldPoint CheckCollision(const OBB& A, const Sphere& B);
	static ManifoldPoint CheckCollision(const OBB& A, const OBB& B);

private:
	void ApplyImpulse(RigidBody& A, RigidBody& B, const ManifoldPoint& P, int c);
	void AvoidSinking();
	


};