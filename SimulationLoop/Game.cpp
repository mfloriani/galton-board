#include "Game.h"
#include "Renderer.h"
#include "Math\Matrix3.h"
#include <random>
#include<time.h>

#define BALL_SIZE  0.5f
#define BALL_TOTAL 50



Game::Game(HDC hdc) : m_hdc(hdc), m_previousTime(0)
{
	Renderer::Init();

	m_physicsSys = new PhysicsSystem();
	
	srand(time(0));

	for (int i = 0; i < BALL_TOTAL; ++i)
	{
		int x = rand() % 10;
		int y = rand() % 10;

		x = i % 2 == 0 ? x : -x;

		m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(x, 20 + y, 0), math::Vector3D(0, -20, 0), 100000.0f, BALL_SIZE));
	}

	//m_physicsSys->AddRigidBody(CreateAABB(math::Vector3D(0, 30.f, 0), math::Vector3D(0, 0, 0), 0.0f, math::Vector3D(15.f, 1.f, 15.f)));
	
	// left funnel
	m_physicsSys->AddRigidBody(
		CreateOBB(
			math::Vector3D(-9.0f, 13.f, 0.f),
			math::Vector3D(0, 0, 0),
			0.0f,
			math::Vector3D(8.0f, 1.0f, 1.0f),
			math::Vector3D(0.f, 0.f, -35.f)
		)
	);

	// right funnel
	m_physicsSys->AddRigidBody(
		CreateOBB(
			math::Vector3D(9.0f, 13.f, 0.f),
			math::Vector3D(0, 0, 0),
			0.0f,
			math::Vector3D(8.0f, 1.0f, 1.0f),
			math::Vector3D(0.f, 0.f, 35.f)
		)
	);
	
	// left
	m_physicsSys->AddRigidBody(
		CreateOBB(
			math::Vector3D(-15.0f, -5.f, 0.f),
			math::Vector3D(0, 0, 0),
			0.0f,
			math::Vector3D(1.0f, 25.0f, 1.0f),
			math::Vector3D(0.f, 0.f, 0.f)
		)
	);

	// right
	m_physicsSys->AddRigidBody(
		CreateOBB(
			math::Vector3D(15.0f, -5.f, 0.f),
			math::Vector3D(0, 0, 0),
			0.0f,
			math::Vector3D(1.0f, 25.0f, 1.0f),
			math::Vector3D(0.f, 0.f, 0.f)
		)
	);

	// back
	m_physicsSys->AddRigidBody(
		CreateOBB(
			math::Vector3D(0.0f, -5.f, -2.f),
			math::Vector3D(0, 0, 0),
			0.0f,
			math::Vector3D(15.0f, 25.0f, 1.0f),
			math::Vector3D(0.f, 0.f, 0.f)
		)
	);
	
	// bottom
	m_physicsSys->AddRigidBody(
		CreateOBB(
			math::Vector3D(0.0f, -30.f, 0.f), 
			math::Vector3D(0, 0, 0), 
			0.0f, 
			math::Vector3D(15.0f, 1.0f, 1.0f), 
			math::Vector3D(0.f, 0.f, 0.f)
		)
	);
	
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&start);
}

Game::~Game(void)
{
	delete m_physicsSys;
}



void Game::Update()
{
	// calculate dt based on the simulation loop rate using a timer
	QueryPerformanceCounter(&end);
	m_dt = static_cast<float>((end.QuadPart - start.QuadPart) / static_cast<double>(frequency.QuadPart));
	start = end;

	if (m_dt > 0.1f) m_dt = 0.1f;

	m_fps = static_cast<int>(1.0 / m_dt);

	m_physicsSys->Update(m_dt);

	Render();
}

void Game::Render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(0, 50, 100, 0, 0, 0, 0, 1, 0);
	
	for (auto b : m_physicsSys->Bodies())
	{
		switch (b->type)
		{
		case VolumeType::Sphere:
			Renderer::DrawSphere(b->sphereVolume);
			break;
		case VolumeType::AABB:
			Renderer::DrawAABBCube(b->aabbVolume);
			break;
		case VolumeType::OBB:
			Renderer::DrawOBBCube(b->obbVolume);
			break;
		}
	}

	SwapBuffers(m_hdc);
}

RigidBody* Game::CreateSphere(math::Vector3D pos, math::Vector3D vel, float mass, float radius)
{
	RigidBody* body = new RigidBody();
	body->position = pos;
	body->velocity = vel;
	body->mass = mass;
	body->type = VolumeType::Sphere;
	body->sphereVolume = Sphere(pos, radius);
	return body;
}

RigidBody* Game::CreateAABB(math::Vector3D pos, math::Vector3D vel, float mass, math::Vector3D size)
{
	RigidBody* body = new RigidBody();
	body->position = pos;
	body->velocity = vel;
	body->mass = mass;
	body->type = VolumeType::AABB;
	body->aabbVolume = AABB(pos, size);
	return body;
}

RigidBody* Game::CreateOBB(math::Vector3D pos, math::Vector3D vel, float mass, math::Vector3D size, math::Vector3D angles)
{
	RigidBody* body = new RigidBody();
	body->position = pos;
	body->velocity = vel;
	body->mass = mass;
	body->type = VolumeType::OBB;
	body->obbVolume = OBB(pos, size, math::rotation3x3(angles.x, angles.y, angles.z));
	return body;
}

