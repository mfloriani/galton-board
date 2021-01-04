#pragma once

#include <Windows.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <gl\gl.h>
#include <gl\GLU.h>

#include "PhysicsSystem.h"
#include "Camera.h"

#define TIME_SCALE_RATE 0.1f
#define FRICTION_RATE 0.1f
#define BALL_SIZE_RATE 0.1f
#define RESTORATION_RATE 0.1f

class Game
{
public:
	Game(HDC hdc);
	~Game(void);

	Camera* camera;
	static float frictionMag;
	static float restitutionMag;
	static float ballSize;
	static bool  debugMode;

	void Update();
	void Render();

	static RigidBody* CreateBall(math::Vector3D pos);
	
	void Reset();
	void Board();
	void DebugBoard();
	void SpawnBall();
	void SpawnBall(int x, int y);
	void PauseResume();
	void IncreaseTimeScale();
	void DecreaseTimeScale();
	void IncreaseFriction();
	void DecreaseFriction();
	void IncreaseBallSize();
	void DecreaseBallSize();
	void IncreaseRestitution();
	void DecreaseRestitution();

	void ToggleDebugMode();

private:
	HDC   m_hdc;
	float m_dt;
	int	  m_fps;
	float m_previousTime;

	bool  m_paused{ false };
	float m_timeScale{ 1.f };
	
	
	
	PhysicsSystem* m_physicsSys;
	LARGE_INTEGER start, end, frequency;

private:
	
	void SpawnBalls();
	void AddPegs();
	void AddBins();
	


};

