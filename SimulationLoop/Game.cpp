#include "Game.h"
#include "Renderer.h"
#include "Math\Matrix3.h"
#include <random>
#include<time.h>

#define BALL_SIZE  0.5f
#define BALL_TOTAL 100
#define BALL_MASS  1.f

#define BLACK math::Vector3D(0.f, 0.f, 0.f)
#define WHITE math::Vector3D(1.f, 1.f, 1.f)
#define GRAY  math::Vector3D(0.5f, 0.5f, 0.5f)

Game::Game(HDC hdc) : m_hdc(hdc), m_previousTime(0)
{
	camera = new Camera( math::Vector3D(0, 0, 150) );

	Renderer::Init();

	m_physicsSys = new PhysicsSystem();
	
	srand(time(0));

	LoadBoard();
	
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&start);
}

Game::~Game(void)
{
	delete camera;
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

	//for (auto b : m_physicsSys->Bodies()) 
	//{
	//	if (b->position.y < -45) 
	//		b->position.y = -45;
	//}

	Render();
}

void Game::Render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	math::Matrix4& view = camera->UpdateView();
	glMultMatrixf(view.data());
	
	for (auto b : m_physicsSys->Bodies())
	{
		Renderer::DrawSphere(b->sphereVolume);
	}

	for (auto& b : m_physicsSys->Constraints())
	{
		Renderer::DrawOBBCube(b);
	}

	SwapBuffers(m_hdc);
}

RigidBody* Game::CreateSphere(math::Vector3D pos, math::Vector3D vel, float mass, float radius, math::Vector3D color)
{
	RigidBody* body = new RigidBody();
	body->position = pos;
	body->velocity = vel;
	body->mass = mass;
	body->type = VolumeType::Sphere;
	body->sphereVolume = Sphere(pos, radius, color);
	return body;
}

//RigidBody* Game::CreateAABB(math::Vector3D pos, math::Vector3D vel, float mass, math::Vector3D size, math::Vector3D color)
//{
//	RigidBody* body = new RigidBody();
//	body->position = pos;
//	body->velocity = vel;
//	body->mass = mass;
//	body->type = VolumeType::AABB;
//	body->aabbVolume = AABB(pos, size, color);
//	return body;
//}
//
//RigidBody* Game::CreateOBB(math::Vector3D pos, math::Vector3D vel, float mass, math::Vector3D size, math::Vector3D angles, math::Vector3D color)
//{
//	RigidBody* body = new RigidBody();
//	body->position = pos;
//	body->velocity = vel;
//	body->mass = mass;
//	body->type = VolumeType::OBB;
//	body->obbVolume = OBB(pos, size, math::rotation3x3(angles.x, angles.y, angles.z), color);
//	return body;
//}



void Game::AddPegs()
{
	float PEG_Y = 0.f;
	constexpr float PEG_Z = 0.f;

#define PEG_COLOR WHITE

	PEG_Y = 20;

	for (int i = 0; i < 10; ++i)
	{
		PEG_Y += -4;

		if (i % 2 == 0)
		{
			for (int j = -16; j < 17; j += 4)
			{
				m_physicsSys->AddConstraint(
					OBB(math::Vector3D(j, PEG_Y, PEG_Z),
						math::Vector3D(1.0f, 1.0f, 1.0f),
						math::rotation3x3(0, 0, 45),
						PEG_COLOR)
				);
			}
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(-16, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(-12, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(-8, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(-4, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(0, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(4, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(8, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(12, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(16, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));

		}
		else
		{
			for (int j = -18; j < 19; j += 4)
			{
				m_physicsSys->AddConstraint(
					OBB(math::Vector3D(j, PEG_Y, PEG_Z),
						math::Vector3D(1.0f, 1.0f, 1.0f),
						math::rotation3x3(0, 0, 45),
						PEG_COLOR)
				);
			}
			
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(-18, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(-14, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(-10, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(-6, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(-2, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(2, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(6, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(10, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(14, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
			//m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(18, PEG_Y, PEG_Z), math::Vector3D(0, 0, 0), 0, 1.f, PEG_COLOR));
		}
	}
}

void Game::AddBins()
{
	float BIN_X = -16.f;

	for (int i = 0; i < 9; ++i)
	{
		m_physicsSys->AddConstraint(
			OBB(math::Vector3D(BIN_X, -35.f, 0.f),
				math::Vector3D(0.3f, 10.0f, 1.0f),
				math::rotation3x3(0, 0, 0),
				BLACK)
		);
		BIN_X += 4.0;
	}
}

void Game::AddBalls()
{
	for (int i = 0; i < BALL_TOTAL; ++i)
	{
		int x = rand() % 10;
		int y = rand() % 10;

		x = i % 2 == 0 ? x : -x;

		m_physicsSys->AddRigidBody(CreateSphere(math::Vector3D(x, 40 + y, 0), math::Vector3D(0, 0, 0), BALL_MASS, BALL_SIZE, WHITE));
	}
}

void Game::LoadBoard()
{
	m_physicsSys->Reset();

	AddBalls();
	AddPegs();
	AddBins();

	// left funnel
	m_physicsSys->AddConstraint(		
		OBB(math::Vector3D(-12.0f, 28.f, 0.f),
			math::Vector3D(12.0f, 1.0f, 1.0f),
			math::rotation3x3(0.f, 0.f, -35.f),
			BLACK)
	);
	

	// right funnel
	m_physicsSys->AddConstraint(
		OBB(math::Vector3D(12.0f, 28.f, 0.f),
			math::Vector3D(12.0f, 1.0f, 1.0f),
			math::rotation3x3(0.f, 0.f, 35.f),
			BLACK)
	);


	// left
	m_physicsSys->AddConstraint(
		OBB(math::Vector3D(-21.0f, -5.f, 0.f),
			math::Vector3D(1.0f, 40.0f, 1.0f),
			math::rotation3x3(0.f, 0.f, 0.f),
			BLACK)
	);


	// right
	m_physicsSys->AddConstraint(
		OBB(math::Vector3D(21.0f, -5.f, 0.f),
			math::Vector3D(1.0f, 40.0f, 1.0f),
			math::rotation3x3(0.f, 0.f, 0.f),
			BLACK)
	);


	// back
	m_physicsSys->AddConstraint(
		OBB(math::Vector3D(0.0f, -5.f, -2.f),
			math::Vector3D(20.0f, 40.0f, 1.0f),
			math::rotation3x3(0.f, 0.f, 0.f),
			GRAY)
	);

	// bottom
	m_physicsSys->AddConstraint(
		OBB(math::Vector3D(0.0f, -46.f, 0.f),
			math::Vector3D(22.0f, 2.0f, 1.0f),
			math::rotation3x3(0.f, 0.f, 0.f),
			BLACK)
	);

}