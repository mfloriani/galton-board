#include "Game.h"
#include "Renderer.h"
#include "Math\Matrix3.h"
#include "Constants.h"

#include <random>
#include <time.h>
#include <iostream>

float Game::frictionMag = FRICTION_MAG_DEFAULT;
float Game::restitutionMag = RESTITUTION_MAG_DEFAULT;
float Game::ballSize = BALL_SIZE_DEFAULT;
bool Game::debugMode = false;
bool Game::debugBoard = false;

Game::Game(HDC hdc) : m_hdc(hdc), m_previousTime(0)
{
	
}

Game::~Game(void)
{
	delete camera;
	delete m_physicsSys;
}

bool Game::Init()
{
	camera = new Camera(math::Vector3D(0, 0, 150));

	if (!Renderer::Init())
		return false;

	m_physicsSys = new PhysicsSystem();

	srand(time(0));

	Board();

	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&start);

	return true;
}

void Game::Update(float dt)
{
	// calculate dt based on the simulation loop rate using a timer
	QueryPerformanceCounter(&end);
	m_dt = static_cast<float>((end.QuadPart - start.QuadPart) / static_cast<double>(frequency.QuadPart));
	start = end;

	// variable delta time
	if (m_dt > 0.1f) m_dt = 0.1f;
	m_fps = static_cast<int>(1.0 / m_dt);
	m_dt *= m_timeScale;

	if(!m_paused)
		m_physicsSys->Update(dt * m_timeScale);

	Render();
}

void Game::Render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	math::Matrix4& view = camera->UpdateView();
	glMultMatrixf(view.data());

#if 1

	for (auto b : m_physicsSys->Bodies())
	{
		if(b->type == VolumeType::Sphere)
			Renderer::DrawSphere(b->sphereVolume);
		else if(b->type == VolumeType::OBB)
			Renderer::DrawOBBCube(b->obbVolume);
		else
			Renderer::DrawAABBCube(b->aabbVolume);
	}

	for (auto b : m_physicsSys->StaticBodies())
	{
		if (b->type == VolumeType::Sphere)
			Renderer::DrawSphere(b->sphereVolume);
		else if (b->type == VolumeType::OBB)
			Renderer::DrawOBBCube(b->obbVolume);
		else if (b->type == VolumeType::AABB)
			Renderer::DrawAABBCube(b->aabbVolume);
	}

#endif

	if(debugMode)
		m_physicsSys->Render();

	SwapBuffers(m_hdc);
}

RigidBody* Game::CreateBall(math::Vector3D pos)
{
	RigidBody* body = new RigidBody();
	body->position = pos;
	body->mass = BALL_MASS;
	body->friction = frictionMag;
	body->restitution = restitutionMag;
	body->type = VolumeType::Sphere;
	body->sphereVolume = Sphere(pos, ballSize, BALL_COLOR);
	return body;
}

RigidBody* Game::CreateCube(math::Vector3D pos)
{
	RigidBody* body = new RigidBody();
	body->position = pos;
	body->mass = CUBE_MASS;
	body->friction = frictionMag;
	body->restitution = restitutionMag;
	body->type = VolumeType::OBB;
	body->obbVolume = OBB(pos, CUBE_SIZE, math::rotation3x3(0.f,0.f,45.f), CUBE_COLOR);
	return body;
}

void Game::AddPegs()
{
	float PEG_Y = 0.f;
	constexpr float PEG_Z = 0.f;

	PEG_Y = 20;

	for (int i = 0; i < 10; ++i)
	{
		PEG_Y += -4;

		if (i % 2 == 0)
		{
			for (int j = -16; j < 17; j += 4)
			{
#ifndef PEG_SHAPE_SPHERE

				m_physicsSys->AddStaticRigidBody(
					CreateStaticOBB(
						math::Vector3D(j, PEG_Y, PEG_Z),
						math::Vector3D(PEG_OBB_SIZE, PEG_OBB_SIZE, 1.0f),
						math::rotation3x3(0, 0, 45),
						PEG_COLOR
					));

#else

				m_physicsSys->AddStaticRigidBody(
					CreateStaticSphere(
						math::Vector3D(j, PEG_Y, PEG_Z),
						PEG_SPHERE_SIZE,
						PEG_COLOR
					));

#endif // !PEG_SPHERE
			}
		}
		else
		{
			for (int j = -18; j < 19; j += 4)
			{
#ifndef PEG_SHAPE_SPHERE

				m_physicsSys->AddStaticRigidBody(
					CreateStaticOBB(
						math::Vector3D(j, PEG_Y, PEG_Z),
						math::Vector3D(PEG_OBB_SIZE, PEG_OBB_SIZE, 1.0f),
						math::rotation3x3(0, 0, 45),
						PEG_COLOR
					));

#else
				m_physicsSys->AddStaticRigidBody(
					CreateStaticSphere(
						math::Vector3D(j, PEG_Y, PEG_Z),
						PEG_SPHERE_SIZE,
						PEG_COLOR
					));


#endif // !PEG_SHAPE_SPHERE
			}
		}
	}
}

void Game::AddBins()
{
	float BIN_X = -16.f;

	for (int i = 0; i < 9; ++i)
	{
		m_physicsSys->AddStaticRigidBody(
			CreateStaticAABB(
				math::Vector3D(BIN_X, -35.f, 0.f),
				math::Vector3D(0.3f, 10.0f, 1.0f),
				BLACK
			));
		BIN_X += 4.0;
	}
}


RigidBody* Game::CreateStaticSphere(math::Vector3D pos, float radius, math::Vector3D color)
{
	RigidBody* body = new RigidBody();
	body->position = pos;
	body->mass = 0;
	body->friction = FRICTION_MAG_DEFAULT;
	body->restitution = RESTITUTION_MAG_DEFAULT;
	body->type = VolumeType::Sphere;
	body->sphereVolume = Sphere(pos, radius, color);
	return body;
}

RigidBody* Game::CreateStaticAABB(math::Vector3D pos, math::Vector3D size, math::Vector3D color)
{
	RigidBody* body = new RigidBody();
	body->position = pos;
	body->mass = 0;
	body->friction = FRICTION_MAG_DEFAULT;
	body->restitution = RESTITUTION_MAG_DEFAULT;
	body->type = VolumeType::AABB;
	body->aabbVolume = AABB(pos, size, color);
	return body;
}

RigidBody* Game::CreateStaticOBB(math::Vector3D pos, math::Vector3D size, math::Matrix3 orientation, math::Vector3D color)
{
	RigidBody* body = new RigidBody();
	body->position = pos;
	body->mass = 0;
	body->friction = FRICTION_MAG_DEFAULT;
	body->restitution = RESTITUTION_MAG_DEFAULT;
	body->type = VolumeType::OBB;
	body->obbVolume = OBB(pos, size, orientation, color);
	return body;
}

void Game::Log()
{
	const auto totalBalls = m_physicsSys->Bodies().size();

	std::cout << "\n";
	std::cout << "Total Balls: " << totalBalls << "\n";
	std::cout << "Restitution: " << restitutionMag << "\n";
	std::cout << "Friction:    " << frictionMag << "\n";
}


void Game::SpawnBall()
{
	int x = rand() % 10;
	int y = rand() % 10;

	x = m_physicsSys->Bodies().size() % 2 == 0 ? x : -x;

	m_physicsSys->AddRigidBody(
		CreateBall(math::Vector3D(x, 40 + y, 0))
	);
	Log();
}

void Game::SpawnBall(int x, int y)
{
	m_physicsSys->AddRigidBody(
		CreateBall(math::Vector3D(x, y, 0))
	);
	Log();
}

void Game::SpawnOBB()
{
	int x = rand() % 10;
	int y = rand() % 10;

	x = m_physicsSys->Bodies().size() % 2 == 0 ? x : -x;

	m_physicsSys->AddRigidBody(
		CreateCube(math::Vector3D(x, 40 + y, 0))
	);
}

void Game::SpawnBalls()
{
#if 0
	for (int i = 0; i < BALL_TOTAL; ++i)
	{
		int x = rand() % 10;
		int y = rand() % 10;

		x = i % 2 == 0 ? x : -x;

		m_physicsSys->AddRigidBody(
			CreateBall(math::Vector3D(x, 40 + y, 0))
		);
	}
#else
	int x = BALL_SPAWN_OFFSET_X;
	int y = BALL_SPAWN_OFFSET_Y;

	for (int i = 0; i < BALL_TOTAL; ++i)
	{
		if (i % BALL_MAX_PER_LINE == 0)
		{
			y += BALL_SPAWN_DIST_Y;
			x = BALL_SPAWN_OFFSET_X;
		}

		m_physicsSys->AddRigidBody(
			CreateBall(math::Vector3D(x, y, 0))
		);

		x += BALL_SPAWN_DIST_x;
	}
#endif
	Log();
}

void Game::Reset()
{
	if (debugBoard)
		DebugBoard();
	else
		Board();
}

void Game::Board()
{
	m_paused = false;
	m_timeScale = 1.f;
	frictionMag = FRICTION_MAG_DEFAULT;
	restitutionMag = RESTITUTION_MAG_DEFAULT;
	ballSize = BALL_SIZE_DEFAULT;

	m_physicsSys->Reset();

	AddPegs();
	AddBins();
	
	// left funnel
	m_physicsSys->AddStaticRigidBody(
		CreateStaticOBB(
			math::Vector3D(-12.0f, 30.f, 0.f),
			math::Vector3D(12.0f, 1.0f, 1.0f),
			math::rotation3x3(0.f, 0.f, -35.f),
			BLACK
		));


	// right funnel
	m_physicsSys->AddStaticRigidBody(
		CreateStaticOBB(
			math::Vector3D(12.0f, 30.f, 0.f),
			math::Vector3D(12.0f, 1.0f, 1.0f),
			math::rotation3x3(0.f, 0.f, 35.f),
			BLACK
		));

	// left
	m_physicsSys->AddStaticRigidBody(
		CreateStaticAABB(
			math::Vector3D(-21.0f, -5.f, 0.f),
			math::Vector3D(1.0f, 40.0f, 1.0f),
			BLACK
		));

	// right
	m_physicsSys->AddStaticRigidBody(
		CreateStaticAABB(
			math::Vector3D(21.0f, -5.f, 0.f),
			math::Vector3D(1.0f, 40.0f, 1.0f),
			BLACK
		));

	// back
	//m_physicsSys->AddStaticRigidBody(
	//	CreateStaticAABB(
	//		math::Vector3D(0.0f, -5.f, -15.f),
	//		math::Vector3D(30.0f, 50.0f, 1.0f),
	//		GRAY
	//	));

	// bottom
	m_physicsSys->AddStaticRigidBody(
		CreateStaticAABB(
			math::Vector3D(0.0f, -46.f, 0.f),
			math::Vector3D(22.0f, 2.0f, 1.0f),
			BLACK
		));

	SpawnBalls();
}

void Game::DebugBoard()
{
	m_paused = false;
	m_timeScale = 1.f;
	frictionMag = FRICTION_MAG_DEFAULT;
	restitutionMag = RESTITUTION_MAG_DEFAULT;
	ballSize = BALL_SIZE_DEFAULT;

	m_physicsSys->Reset();

	// left
	m_physicsSys->AddRigidBody(
		CreateStaticAABB(math::Vector3D(-9.0f, -3.f, 0.f), math::Vector3D(1.0f, 10.0f, 1.0f), BLACK)
	);

	// right	
	m_physicsSys->AddRigidBody(
		CreateStaticAABB(math::Vector3D(9.0f, -3.f, 0.f), math::Vector3D(1.0f, 10.0f, 1.0f), BLACK)
	);

	// bottom
	m_physicsSys->AddRigidBody(
		CreateStaticAABB(math::Vector3D(0.0f, -15.f, 0.f), math::Vector3D(8.0f, 2.0f, 1.0f), BLACK)
	);


	m_physicsSys->AddStaticRigidBody(
		CreateStaticOBB(
			math::Vector3D(0.0f, 0.f, 0.f), 
			math::Vector3D(8.0f, 2.0f, 1.0f),
			math::rotation3x3(0.f, 0.f, 45.f),
			GREEN)
	);

	SpawnBall(0, 0);
}

void Game::PauseResume()
{
	m_paused = !m_paused;
}

void Game::IncreaseTimeScale()
{
	m_timeScale += TIME_SCALE_RATE;
}

void Game::DecreaseTimeScale()
{
	m_timeScale -= TIME_SCALE_RATE;
	if (m_timeScale < 0.1f) m_timeScale = 0.1f;
}

void Game::IncreaseFriction()
{
	frictionMag += FRICTION_RATE;
	//if (frictionMag > 1.f) frictionMag = 1.f;
	m_physicsSys->UpdateFriction(frictionMag);
	Log();
}

void Game::DecreaseFriction()
{
	frictionMag -= FRICTION_RATE;
	if (frictionMag < 0.f) frictionMag = 0.f;
	m_physicsSys->UpdateFriction(frictionMag);
	Log();
}

void Game::IncreaseBallSize()
{
	ballSize += BALL_SIZE_RATE;
	if (ballSize > 0.9f) ballSize = 0.9f;
	m_physicsSys->UpdateBallSize(ballSize);
}

void Game::DecreaseBallSize()
{
	ballSize -= BALL_SIZE_RATE;
	if (ballSize < 0.1f) ballSize = 0.1f;
	m_physicsSys->UpdateBallSize(ballSize);
}

void Game::IncreaseRestitution()
{
	restitutionMag += RESTORATION_RATE;
	//if (restitutionMag > 1.0f) restitutionMag = 1.f;
	m_physicsSys->UpdateRestitution(restitutionMag);
	Log();
}

void Game::DecreaseRestitution()
{
	restitutionMag -= RESTORATION_RATE;
	if (restitutionMag < 0.1f) restitutionMag = 0.1f;
	m_physicsSys->UpdateRestitution(restitutionMag);
	Log();
}

void Game::ToggleDebugMode()
{
	debugMode = !debugMode;
}

void Game::ToggleDebugBoard()
{
	debugBoard = !debugBoard;
	if (debugBoard)
		DebugBoard();
	else
		Board();
}