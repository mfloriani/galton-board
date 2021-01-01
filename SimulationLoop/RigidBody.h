#pragma once

#include "Geometry.h"

enum class VolumeType
{
	None,
	Sphere,
	AABB
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

public:
	VolumeType     type{ VolumeType::None };
	math::Vector3D velocity;
	math::Vector3D position;
	float          mass{ 1.0f };
	float          friction{ 0.6f };
	float          restitution{ 0.5f };
	Sphere         sphereVolume;
	AABB           boxVolume;

private:
	math::Vector3D m_forces;


};