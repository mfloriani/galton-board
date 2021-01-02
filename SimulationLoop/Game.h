#pragma once

#include <Windows.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <gl\gl.h>
#include <gl\GLU.h>

#include "PhysicsSystem.h"
#include "Camera.h"

class Game
{
public:
	Game(HDC hdc);
	~Game(void);

	Camera* camera;

	void Update();
	void Render();

	static RigidBody* CreateSphere(math::Vector3D pos, math::Vector3D vel, float mass, float radius, math::Vector3D color);
	static RigidBody* CreateAABB(math::Vector3D pos, math::Vector3D vel, float mass, math::Vector3D size, math::Vector3D color);
	static RigidBody* CreateOBB(math::Vector3D pos, math::Vector3D vel, float mass, math::Vector3D size, math::Vector3D angles, math::Vector3D color);

private:
	HDC   m_hdc;
	float m_dt;
	int	  m_fps;
	float m_previousTime;
	
	PhysicsSystem* m_physicsSys;
	LARGE_INTEGER start, end, frequency;

private:
	void AddBalls();
	void AddPegs();
	void AddBins();


};

