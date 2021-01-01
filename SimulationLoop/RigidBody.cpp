#include "RigidBody.h"

RigidBody::RigidBody()
{	
}

RigidBody::~RigidBody()
{
}

inline float RigidBody::InverseMass()
{
	if (mass == 0.0f)
		return 0.0f;
	return 1.0f / mass;
}

void RigidBody::ApplyForces()
{
	m_forces = math::Vector3D(0.0f, -9.81f * mass, 0.f);
}

void RigidBody::Update(float dt)
{
	// air friction 
	constexpr float damping = 0.98f;

	// Calculate acceleration
	math::Vector3D accel = m_forces * InverseMass();

	// Integrate accel to get the velocity (using Euler)
	velocity = velocity + (accel * dt);
	velocity = velocity * damping;

	// Integrate old velocity to get the new position (using Euler)
	position = position + (velocity * dt);

	SyncCollisionVolumes();
}

void RigidBody::SyncCollisionVolumes()
{
	sphereVolume.position = position;
	aabbVolume.position = position;
	obbVolume.position = position;
}


