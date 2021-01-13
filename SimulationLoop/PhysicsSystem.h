#pragma once

#include "RigidBody.h"
#include "QuadTree.h"

#include <memory>
#include <unordered_map>

class PhysicsSystem
{
public:
	PhysicsSystem();
	~PhysicsSystem();

	void Update(float dt);
	void Render();
	void RenderQuadTree(QuadTree& quadTree);

	void AddRigidBody(RigidBody* body);
	void AddStaticRigidBody(RigidBody* body);
	
	void ClearRigidBodies();
	void ClearStaticRigidBodies();

	void Reset();

	const std::vector<RigidBody*>& Bodies() const { return m_bodies; }
	const std::vector<RigidBody*>& StaticBodies() const { return m_staticBodies; }

	void UpdateBallSize(float ballSize);
	void UpdateRestitution(float restitution);
	void UpdateFriction(float friction);

private:
	std::unique_ptr<QuadTree>  m_quadTree;
	std::vector<QuadTreeData>  m_quadTreeData;
	
	std::vector<RigidBody*>    m_staticBodies;
	std::vector<RigidBody*>    m_bodies;
	std::vector<RigidBody*>    m_collidersA;
	std::vector<RigidBody*>    m_collidersB;
	std::vector<ManifoldPoint> m_results;
	
	float m_penetrationSlack{ 0.01f }; // Smaller = more accurate [0.01 to 0.1] 	
	float m_linearProjectionPercent{ 0.2f }; // Smaller = less jitter / more penetration [0.2 to 0.8]	
	int m_impulseIteration{ 20 };// More interations more accurate [1 to 20]

	static ManifoldPoint CheckCollision(const RigidBody& A, const RigidBody& B);
	static ManifoldPoint CheckCollision(const Sphere& A, const Sphere& B);
	static ManifoldPoint CheckCollision(const AABB& A, const Sphere& B);
	static ManifoldPoint CheckCollision(const OBB& A, const Sphere& B);
	static ManifoldPoint CheckCollision(const OBB& A, const OBB& B);

private:	
	void AvoidSinking();
	size_t InsertQuadTree(RigidBody* body);
};