#include "RigidBody.h"

#define GRAVITY -9.81f
//#define DAMPING 0.98f // air friction 
#define DAMPING 1.f 

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
	m_forces = math::Vector3D(0.0f, GRAVITY * mass, 0.f);
}

void RigidBody::Update(float dt)
{
	
	
	// Calculate acceleration
	math::Vector3D accel = m_forces * InverseMass();

	// Integrate accel to get the velocity (using Euler)
	velocity = velocity + accel * dt;
	velocity = velocity * DAMPING;

	if (fabsf(velocity.x) < 0.001f)
		velocity.x = 0.0f;
	if (fabsf(velocity.y) < 0.001f) 
		velocity.y = 0.0f;
	if (fabsf(velocity.z) < 0.001f) 
		velocity.z = 0.0f;

#ifdef ENABLE_ANGULAR

	if (type == VolumeType::OBB)
	{
		math::Vector3D angAccel = math::multiplyVector(m_torques, InverseTensor());
		angularVel += angAccel * dt;
		angularVel *= damping;

		if (fabsf(angularVel.x) < 0.001f)
			angularVel.x = 0.0f;
		if (fabsf(angularVel.y) < 0.001f) 
			angularVel.y = 0.0f;
		if (fabsf(angularVel.z) < 0.001f)
			angularVel.z = 0.0f;
	}
#endif

	// Integrate old velocity to get the new position (using Euler)
	position = position + velocity * dt;

#ifdef ENABLE_ANGULAR
	if (type == VolumeType::OBB)
		orientation += angularVel * dt;
#endif

	SyncCollisionVolumes();
}

void RigidBody::SyncCollisionVolumes()
{
	sphereVolume.position = position;
	aabbVolume.position = position;
	obbVolume.position = position;

#ifdef ENABLE_ANGULAR
	obbVolume.orientation = math::rotation3x3(
		math::toDegrees(orientation.x), 
		math::toDegrees(orientation.y),
		math::toDegrees(orientation.z)
	);
#endif
}

math::Matrix4 RigidBody::InverseTensor()
{
	float ix = 0.f;
	float iy = 0.f;
	float iz = 0.f;
	float iw = 0.f;

	if (mass == 0.f)
		return math::Matrix4();

	if (type == VolumeType::Sphere)
	{
		float r2 = sphereVolume.radius * sphereVolume.radius;
		float fraction = 2.f / 5.f;
		ix = r2 * mass * fraction;
		iy = r2 * mass * fraction;
		iz = r2 * mass * fraction;
		iw = 1.f;
	}
	else if (type == VolumeType::OBB)
	{
		math::Vector3D size = obbVolume.size * 2.f;
		float fraction = 1.f / 12.f;
		float x2 = size.x * size.x;
		float y2 = size.y * size.y;
		float z2 = size.z * size.z;
		ix = (y2 + z2) * mass * fraction;
		iy = (x2 + z2) * mass * fraction;
		iz = (x2 + y2) * mass * fraction;
		iw = 1.f;
	}

	return math::inverse(
		math::Matrix4( ix, 0, 0, 0,
						0, iy, 0, 0,
						0, 0, iz, 0,
						0, 0, 0, iw )
	);
}

void RigidBody::AddRotationalImpulse(const math::Vector3D& point, const math::Vector3D& impulse)
{
	math::Vector3D centerOfMass = position;
	math::Vector3D torque = (point - centerOfMass).cross(impulse);
	math::Vector3D angAccel = math::multiplyVector(torque, InverseTensor());
	angularVel += angAccel;
}

