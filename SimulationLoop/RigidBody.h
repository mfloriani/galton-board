#pragma once

#include "Geometry.h"
#include "Math\Matrix4.h"

#define EULER_INTEGRATION
#define ACCURATE_EULER_INTEGRATION

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
	
	inline float InverseMass() { if (mass == 0.0f) return 0.0f; return 1.0f / mass;	}
	void Update(float dt);
	void ApplyForces();
	void SyncCollisionVolumes();

	math::Matrix4 InverseTensor();
	void AddRotationalImpulse(const math::Vector3D& point, const math::Vector3D& impulse);

	void SolveConstraints(const std::vector<OBB>& constraints);

public:
	VolumeType     type{ VolumeType::None };
	math::Vector3D velocity;
	math::Vector3D position;
	math::Vector3D oldPosition;
	math::Vector3D orientation;
	math::Vector3D angularVel;
	float          mass{ 1.0f };
	float          friction{ 0.0f };
	float          restitution{ 0.0f };
	Sphere         sphereVolume;
	AABB           aabbVolume;
	OBB            obbVolume;

private:
	math::Vector3D m_forces;
	math::Vector3D m_torques;

};