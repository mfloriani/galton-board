#pragma once

#include <Windows.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <gl\gl.h>
#include <gl\GLU.h>

#include "PhysicsSystem.h"
#include "Camera.h"
#include "Constants.h"



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

	void Update(float dt);
	void Render();

	static RigidBody* CreateBall(math::Vector3D pos);
	static RigidBody* CreateCube(math::Vector3D pos);
	
	void Reset();
	void Board();
	
	void DebugBoard();
	void SpawnBall();
	void SpawnBall(int x, int y);
	void SpawnOBB();
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
	
	RigidBody* CreateStaticSphere(math::Vector3D pos, float radius, math::Vector3D color);
	RigidBody* CreateStaticAABB(math::Vector3D pos, math::Vector3D size, math::Vector3D color);
	RigidBody* CreateStaticOBB(math::Vector3D pos, math::Vector3D size, math::Matrix3 orientation, math::Vector3D color);

};

