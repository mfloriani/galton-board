#include "Game.h"
#include "Renderer.h"

Game::Game(HDC hdc) : m_hdc(hdc), m_previousTime(0)
{
	Renderer::Init();

	m_physicsSys = new PhysicsSystem();
	
	m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(0, 0, 0), math::Vector3D(0, 0, 0), 1.0f, 5));
	m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(0, 20.f, 0), math::Vector3D(0, 0, 0), 1.0f, 5));
	m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(10.f, 20.f, 0), math::Vector3D(0, 0, 0), 1.0f, 5));
	m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(-10.f, 20.f, 0), math::Vector3D(0, 0, 0), 1.0f, 5));
	m_physicsSys->AddRigidBody(CreateCube(math::Vector3D(0, -20.f, 0), math::Vector3D(0, 0, 0), 0.0f, math::Vector3D(15.f, 1.f, 0)));


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
	
	// Draw plane (at y=-20)
	//glBegin(GL_QUADS);
	//	glColor3d(1, 1, 1);
	//	glVertex3d(-50, -20, -50);
	//	glVertex3d( 50, -20, -50);
	//	glVertex3d( 50, -20,  50);
	//	glVertex3d(-50, -20,  50);
	//glEnd();

	for (auto b : m_physicsSys->Bodies())
	{
		switch (b->type)
		{
		case VolumeType::Sphere:
			Renderer::DrawSphere(b->sphereVolume);
			break;
		case VolumeType::AABB:
			Renderer::DrawCube(b->boxVolume);
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
	body->boxVolume = AABB();
	return body;
}

RigidBody* Game::CreateCube(math::Vector3D pos, math::Vector3D vel, float mass, math::Vector3D size)
{
	RigidBody* body = new RigidBody();
	body->position = pos;
	body->velocity = vel;
	body->mass = mass;
	body->type = VolumeType::AABB;
	body->sphereVolume = Sphere();
	body->boxVolume = AABB(pos, size);
	return body;
	return nullptr;
}

