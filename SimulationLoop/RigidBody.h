#pragma once

#include "Geometry.h"
#include "Math\Matrix4.h"
#include "Constants.h"

enum class VolumeType
{
	None,
	Sphere,
	AABB,
	OBB
};

class RigidBody
{
public:
	RigidBody();
	~RigidBody();
	
	void Update(float dt);
	void ApplyForces();
	void SyncCollisionVolumes();
	inline float InverseMass() { if (mass == 0.0f) return 0.0f; return 1.0f / mass;	}

#ifdef ANGULAR_VELOCITY
	math::Matrix4 InverseTensor();
	void AddRotationalImpulse(const math::Vector3D& point, const math::Vector3D& impulse);
#endif

#ifdef CONSTRAINT_BOARD
	void SolveConstraints(const std::vector<OBB>& constraints);
#endif
	static void ApplyImpulse(RigidBody& A, RigidBody& B, const ManifoldPoint& P, int c);

public:
	VolumeType     type{ VolumeType::None };
#ifdef EULER_INTEGRATION
	math::Vector3D velocity;
#endif
	math::Vector3D position;
	math::Vector3D oldPosition;
	
#ifdef ANGULAR_VELOCITY
	math::Vector3D orientation;
	math::Vector3D angularVel;
#endif // !ANGULAR_VELOCITY

	float          mass{ 1.0f };
	float          friction{ 0.0f };
	float          restitution{ 0.0f };

	Sphere         sphereVolume;
	AABB           aabbVolume;
	OBB            obbVolume;

private:
	math::Vector3D m_forces;
#ifdef ANGULAR_VELOCITY
	math::Vector3D m_torques;
#endif

};