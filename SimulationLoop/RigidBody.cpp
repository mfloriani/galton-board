#include "RigidBody.h"
#include "Game.h"

#define GRAVITY -9.81f

RigidBody::RigidBody()
{	
}

RigidBody::~RigidBody()
{
}

void RigidBody::ApplyForces()
{
	m_forces = math::Vector3D(0.0f, GRAVITY * mass, 0.f);
}



void RigidBody::Update(float dt)
{	
#ifdef EULER_INTEGRATION
	oldPosition = position;	
	math::Vector3D accel = m_forces * InverseMass();

#ifdef ACCURATE_EULER_INTEGRATION // Velocity Verlet Integration
	math::Vector3D oldVelocity = velocity;
	velocity = velocity * (1.f - friction) + accel * dt;
	position = position + (oldVelocity + velocity) * 0.5f * dt;
#else
	velocity = velocity * DAMPING + accel * dt;
	position = position + velocity * dt;
#endif

#else 

	math::Vector3D vel = position - oldPosition;
	oldPosition = position;
	const float dtSq = dt * dt;
	position = position + (vel * DAMPING + m_forces * dtSq);

#endif

	//if (fabsf(velocity.x) < 0.001f)
	//	velocity.x = 0.0f;
	//if (fabsf(velocity.y) < 0.001f) 
	//	velocity.y = 0.0f;
	//if (fabsf(velocity.z) < 0.001f) 
	//	velocity.z = 0.0f;

//#ifdef ENABLE_ANGULAR
//
//	if (type == VolumeType::OBB)
//	{
//		math::Vector3D angAccel = math::multiplyVector(m_torques, InverseTensor());
//		angularVel += angAccel * dt;
//		angularVel *= damping;
//
//		if (fabsf(angularVel.x) < 0.001f)
//			angularVel.x = 0.0f;
//		if (fabsf(angularVel.y) < 0.001f) 
//			angularVel.y = 0.0f;
//		if (fabsf(angularVel.z) < 0.001f)
//			angularVel.z = 0.0f;
//	}
//#endif
//
//	
//
//#ifdef ENABLE_ANGULAR
//	if (type == VolumeType::OBB)
//		orientation += angularVel * dt;
//#endif

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

//math::Matrix4 RigidBody::InverseTensor()
//{
//	float ix = 0.f;
//	float iy = 0.f;
//	float iz = 0.f;
//	float iw = 0.f;
//
//	if (mass == 0.f)
//		return math::Matrix4();
//
//	if (type == VolumeType::Sphere)
//	{
//		float r2 = sphereVolume.radius * sphereVolume.radius;
//		float fraction = 2.f / 5.f;
//		ix = r2 * mass * fraction;
//		iy = r2 * mass * fraction;
//		iz = r2 * mass * fraction;
//		iw = 1.f;
//	}
//	else if (type == VolumeType::OBB)
//	{
//		math::Vector3D size = obbVolume.size * 2.f;
//		float fraction = 1.f / 12.f;
//		float x2 = size.x * size.x;
//		float y2 = size.y * size.y;
//		float z2 = size.z * size.z;
//		ix = (y2 + z2) * mass * fraction;
//		iy = (x2 + z2) * mass * fraction;
//		iz = (x2 + y2) * mass * fraction;
//		iw = 1.f;
//	}
//
//	return math::inverse(
//		math::Matrix4( ix, 0, 0, 0,
//						0, iy, 0, 0,
//						0, 0, iz, 0,
//						0, 0, 0, iw )
//	);
//}

//void RigidBody::AddRotationalImpulse(const math::Vector3D& point, const math::Vector3D& impulse)
//{
//	math::Vector3D centerOfMass = position;
//	math::Vector3D torque = math::cross((point - centerOfMass),impulse);
//	math::Vector3D angAccel = math::multiplyVector(torque, InverseTensor());
//	angularVel += angAccel;
//}


void RigidBody::SolveConstraints(const std::vector<OBB>& constraints)
{
	int size = constraints.size();
	for (int i = 0; i < size; ++i) 
	{
		Line traveled(oldPosition, position);
		//if (PointInOBB(position, constraints[i])) 
		if (Linetest(constraints[i], traveled)) 
		{
#ifndef EULER_INTEGRATION
			math::Vector3D velocity = position - oldPosition;
#endif
			math::Vector3D direction = math::normalize(velocity);
			Ray ray(oldPosition, direction);
			RaycastResult result;

			if (Raycast(constraints[i], ray, &result)) 
			{
				// Place object just a little above collision result
				position = result.point + result.normal * 0.003f;
				//position = result.point + result.normal * (sphereVolume.radius * 0.5f);

				math::Vector3D vn = result.normal * result.normal.dot(velocity);
				math::Vector3D vt = velocity - vn;

#ifdef EULER_INTEGRATION
				oldPosition = position;
				velocity = vt - vn * restitution;
#else
				oldPosition = position - (vt - vn * bounce);
#endif
				break;
			}
		}
	}
}

