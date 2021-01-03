#include "Game.h"
#include "Renderer.h"
#include "Math\Matrix3.h"
#include <random>
#include<time.h>

#define BALL_TOTAL 1
#define BALL_MASS  1.f

#define BLACK math::Vector3D(0.f, 0.f, 0.f)
#define WHITE math::Vector3D(1.f, 1.f, 1.f)
#define GRAY  math::Vector3D(0.5f, 0.5f, 0.5f)

#define FRICTION_MAG_DEFAULT 0.f
#define RESTITUTION_MAG_DEFAULT 0.1f
#define BALL_SIZE_DEFAULT 0.5f

float Game::frictionMag = FRICTION_MAG_DEFAULT;
float Game::restitutionMag = RESTITUTION_MAG_DEFAULT;
float Game::ballSize = BALL_SIZE_DEFAULT;
bool Game::debugMode = false;

Game::Game(HDC hdc) : m_hdc(hdc), m_previousTime(0)
{
	camera = new Camera( math::Vector3D(0, 0, 150) );

	Renderer::Init();

	m_physicsSys = new PhysicsSystem();
	
	srand(time(0));

	Board();
	
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

	m_dt *= m_timeScale;

	if(!m_paused)
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
		if(b->type == VolumeType::Sphere)
			Renderer::DrawSphere(b->sphereVolume);
		else if(b->type == VolumeType::OBB)
			Renderer::DrawOBBCube(b->obbVolume);
		else
			Renderer::DrawAABBCube(b->aabbVolume);
	}

	for (auto& b : m_physicsSys->Constraints())
	{
		Renderer::DrawOBBCube(b);
	}

	SwapBuffers(m_hdc);
}

RigidBody* Game::CreateBall(math::Vector3D pos)
{
	RigidBody* body = new RigidBody();
	body->position = pos;
	//body->velocity = vel;
	body->mass = BALL_MASS;
	body->friction = frictionMag;
	body->restitution = restitutionMag;
	body->type = VolumeType::Sphere;
	body->sphereVolume = Sphere(pos, ballSize, WHITE);
	return body;
}

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

void Game::SpawnBall()
{
	int x = rand() % 10;
	int y = rand() % 10;

	x = m_physicsSys->Bodies().size() % 2 == 0 ? x : -x;

	m_physicsSys->AddRigidBody(
		CreateBall(math::Vector3D(x, 40 + y, 0))
	);
}

void Game::SpawnBalls()
{
	for (int i = 0; i < BALL_TOTAL; ++i)
	{
		int x = rand() % 10;
		int y = rand() % 10;

		x = i % 2 == 0 ? x : -x;

		m_physicsSys->AddRigidBody(
			CreateBall(math::Vector3D(x, 40 + y, 0))
		);
	}
}

void Game::Reset()
{
	if (debugMode)
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

	SpawnBalls();
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

void Game::DebugBoard()
{
	m_paused = false;
	m_timeScale = 1.f;
	frictionMag = FRICTION_MAG_DEFAULT;
	restitutionMag = RESTITUTION_MAG_DEFAULT;
	ballSize = BALL_SIZE_DEFAULT;

	m_physicsSys->Reset();

	SpawnBall();

	m_physicsSys->AddConstraint(
		OBB(math::Vector3D(0.0f, -46.f, 0.f),
			math::Vector3D(22.0f, 2.0f, 1.0f),
			math::rotation3x3(0.f, 0.f, 0.f),
			BLACK)
	);

	//RigidBody* bottom = new RigidBody();
	//bottom->position = math::Vector3D(0.0f, -46.f, 0.f);
	//bottom->mass = 0.f;
	//bottom->type = VolumeType::OBB;

	//OBB bottomOBB;
	//bottomOBB.position = bottom->position;
	//bottomOBB.size = math::Vector3D(22.0f, 2.0f, 1.0f);
	//bottomOBB.orientation = math::rotation3x3(0.f, 0.f, 0.f);
	//bottomOBB.color = BLACK;
	//bottom->obbVolume = bottomOBB;

	//RigidBody* bottom = new RigidBody();
	//bottom->position = math::Vector3D(0.0f, -46.f, 0.f);
	//bottom->mass = 0.f;
	//bottom->type = VolumeType::AABB;

	//AABB aabb;
	//aabb.position = bottom->position;
	//aabb.size = math::Vector3D(22.0f, 2.0f, 1.0f);
	//aabb.color = BLACK;
	//bottom->aabbVolume = aabb;

	//m_physicsSys->AddRigidBody(bottom);
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
	if (frictionMag > 1.f) frictionMag = 1.f;
	m_physicsSys->UpdateFriction(frictionMag);
}

void Game::DecreaseFriction()
{
	frictionMag -= FRICTION_RATE;
	if (frictionMag < 0.f) frictionMag = 0.f;
	m_physicsSys->UpdateFriction(frictionMag);
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
	if (restitutionMag > 1.0f) restitutionMag = 1.f;
	m_physicsSys->UpdateRestitution(restitutionMag);
}

void Game::DecreaseRestitution()
{
	restitutionMag -= RESTORATION_RATE;
	if (restitutionMag < 0.1f) restitutionMag = 0.1f;
	m_physicsSys->UpdateRestitution(restitutionMag);
}

void Game::ToggleDebugMode()
{
	debugMode = !debugMode;
	if (debugMode)
		DebugBoard();
	else
		Board();
}
