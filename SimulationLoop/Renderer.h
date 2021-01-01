#pragma once

#include "TextureLoader.h"
#include "Geometry.h"

class Renderer
{
public:
	static void Init();
	static void DrawSphere(const Sphere& s);
	static void DrawCube(const AABB& c);

	static GLuint m_sphereTex;
};
