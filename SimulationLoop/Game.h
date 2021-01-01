#pragma once

#include <Windows.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <gl\gl.h>
#include <gl\GLU.h>

#include "PhysicsSystem.h"

class Game
{
public:
	Game(HDC hdc);
	~Game(void);

	void Update();
	void Render();

	static RigidBody* CreateSphere(Vector2f pos, Vector2f vel, float mass, float radius);
	static RigidBody* CreateCube(Vector2f pos, Vector2f vel, float mass, Vector2f size);

private:
	HDC   m_hdc;
	float m_dt;
	int	  m_fps;
	float m_previousTime;
	
	PhysicsSystem* m_physicsSys;
	LARGE_INTEGER start, end, frequency;
};

