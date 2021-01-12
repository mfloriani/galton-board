#include "RigidBody.h"
#include "Game.h"

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
	velocity = velocity * DAMPING + accel * dt;
	position = position + (oldVelocity + velocity) * 0.5f * dt;
#else
	velocity = velocity * DAMPING + accel * dt;
	position = position + velocity * dt;
#endif

#else 

	math::Vector3D velocity = position - oldPosition;
	oldPosition = position;
	float deltaSquare = dt * dt;
	position = position + (velocity * friction + m_forces * deltaSquare);

#endif

	//if (fabsf(velocity.x) < 0.001f)
	//	velocity.x = 0.0f;
	//if (fabsf(velocity.y) < 0.001f) 
	//	velocity.y = 0.0f;
	//if (fabsf(velocity.z) < 0.001f) 
	//	velocity.z = 0.0f;

#ifdef ANGULAR_VELOCITY
	//if (type == VolumeType::OBB)
	{
		math::Vector3D angAccel = math::multiplyVector(m_torques, InverseTensor());
		angularVel += angAccel * dt;
		angularVel *= DAMPING;

		//if (fabsf(angularVel.x) < 0.001f)
		//	angularVel.x = 0.0f;
		//if (fabsf(angularVel.y) < 0.001f) 
		//	angularVel.y = 0.0f;
		//if (fabsf(angularVel.z) < 0.001f)
		//	angularVel.z = 0.0f;
	}

	//if (type == VolumeType::OBB)
		orientation += angularVel * dt;
#endif
	SyncCollisionVolumes();
}

void RigidBody::SyncCollisionVolumes()
{
	sphereVolume.position = position;
	aabbVolume.position = position;
	obbVolume.position = position;

#ifdef ANGULAR_VELOCITY
	obbVolume.orientation = math::rotation3x3(
		math::toDegrees(orientation.x), 
		math::toDegrees(orientation.y),
		math::toDegrees(orientation.z)
	);
#endif
}

#ifdef ANGULAR_VELOCITY
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
	math::Vector3D torque = math::cross((point - centerOfMass),impulse);
	math::Vector3D angAccel = math::multiplyVector(torque, InverseTensor());
	angularVel += angAccel;
}
#endif

#ifdef CONSTRAINT_BOARD
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
				//position = result.point + result.normal * (0.003f + sphereVolume.radius);

				math::Vector3D vn = result.normal * result.normal.dot(velocity);
				math::Vector3D vt = velocity - vn;

#ifdef EULER_INTEGRATION
				oldPosition = position;
				velocity = vt - vn * restitution;
#else
				oldPosition = position - (vt - vn * restitution);
#endif
				break;
			}
		}
	}
}
#endif


void RigidBody::ApplyImpulse(RigidBody& A, RigidBody& B, const ManifoldPoint& P, int c)
{
	const float invMassA = A.InverseMass();
	const float invMassB = B.InverseMass();
	const float invMassSum = invMassA + invMassB;

	if (invMassSum == 0.0f)
		return;

#ifdef ANGULAR_VELOCITY
	math::Vector3D r1 = P.contacts[c] - A.position;
	math::Vector3D r2 = P.contacts[c] - B.position;
	math::Matrix4 i1 = A.InverseTensor();
	math::Matrix4 i2 = B.InverseTensor();
	
	math::Vector3D relativeVel = (B.velocity + math::cross(B.angularVel, r2)) - (A.velocity + math::cross(A.angularVel, r1));
#else
	const math::Vector3D relativeVel = B.velocity - A.velocity;
#endif

	math::Vector3D relativeNormal = P.normal;
	relativeNormal = relativeNormal.normalize();

	const float relativeDir = relativeVel.dot(relativeNormal);

	// Moving away from each other?
	if (relativeDir > 0.0f)
		return;

	const float e = fminf(A.restitution, B.restitution);
	float numerator = -(1.0f + e) * relativeDir;
	float d1 = invMassSum;
#ifdef ANGULAR_VELOCITY
	math::Vector3D d2 = math::cross(math::multiplyVector(math::cross(r1, relativeNormal), i1), r1);
	math::Vector3D d3 = math::cross(math::multiplyVector(math::cross(r2, relativeNormal), i2), r2);
	float denominator = d1 + relativeNormal.dot(d2 + d3);
#else
	float denominator = d1;
#endif
	
	float j = denominator == 0.f ? 0.f : numerator / denominator;
	if (P.contacts.size() > 0 && j != 0.0f)
		j /= static_cast<float>(P.contacts.size());

	math::Vector3D impulse = relativeNormal * j;
	A.velocity = A.velocity - impulse * invMassA;
	B.velocity = B.velocity + impulse * invMassB;

#ifdef ANGULAR_VELOCITY
	A.angularVel -= math::multiplyVector(math::cross(r1, impulse), i1);
	B.angularVel += math::multiplyVector(math::cross(r2, impulse), i2);
#endif

#if 1

	//
	// Friction
	//

	math::Vector3D t = relativeVel - (relativeNormal * relativeDir);
	if (CMP(t.sizeSqr(), 0.0f))
		return;

	t = t.normalize();

	numerator = -relativeVel.dot(t);
	d1 = invMassSum;
#ifdef ANGULAR_VELOCITY
	d2 = math::cross(math::multiplyVector(math::cross(r1, t), i1), r1);
	d3 = math::cross(math::multiplyVector(math::cross(r2, t), i2), r2);
	denominator = d1 + t.dot(d2 + d3);
#else
	denominator = d1;
#endif
	if (denominator == 0.f)
		return;

	float jt = numerator / denominator;

	if (P.contacts.size() > 0 && jt != 0.0f)
		jt /= static_cast<float>(P.contacts.size());

	if (CMP(jt, 0.0f))
		return;

	const float friction = sqrtf(A.friction * B.friction);
	if (jt > j * friction)
		jt = j * friction;
	else if (jt < -j * friction)
		jt = -j * friction;

	const math::Vector3D tangentImpulse = t * jt;
	A.velocity = A.velocity - tangentImpulse * invMassA;
	B.velocity = B.velocity + tangentImpulse * invMassB;
#ifdef ANGULAR_VELOCITY
	A.angularVel -= math::multiplyVector(math::cross(r1, tangentImpulse), i1);
	B.angularVel -= math::multiplyVector(math::cross(r2, tangentImpulse), i2);
#endif

#endif // enable friction? #if 1

}