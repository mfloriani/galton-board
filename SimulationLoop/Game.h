#pragma once

#include <Windows.h>
#include "Sphere.h"
#include "ContactManifold.h"

class Game
{
public:
	Game(HDC hdc);
	~Game(void);

	void Update();

private:
	void SimulationLoop();
	void CalculateObjectPhysics();
	void DynamicCollisionDetection();
	void DynamicCollisionResponse();
	void UpdateObjectPhysics();
	void Render();

private:
	HDC   m_hdc;
	float m_dt;
	int	  m_fps;
	float m_previousTime;
	Sphere *m_sphere1;
	Sphere *m_sphere2;
	Sphere *m_sphere3;
	ContactManifold *m_manifold;
	LARGE_INTEGER start, end, frequency;
};

