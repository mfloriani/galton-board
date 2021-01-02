#pragma once

#include "Geometry.h"
#include "Math\Matrix4.h"

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
	
	inline float InverseMass();	
	void Update(float dt);
	void ApplyForces();
	void SyncCollisionVolumes();

	math::Matrix4 InverseTensor();
	void AddRotationalImpulse(const math::Vector3D& point, const math::Vector3D& impulse);

public:
	VolumeType     type{ VolumeType::None };
	math::Vector3D velocity;
	math::Vector3D position;
	math::Vector3D orientation;
	math::Vector3D angularVel;
	float          mass{ 1.0f };
	float          friction{ 0.6f };
	float          restitution{ 0.5f };
	Sphere         sphereVolume;
	AABB           aabbVolume;
	OBB            obbVolume;

private:
	math::Vector3D m_forces;
	math::Vector3D m_torques;

};