#pragma once

#include "TextureLoader.h"
#include "Geometry.h"

class Renderer
{
public:
	static void Init();
	static void DrawSphere(const Sphere& s);
	static void DrawAABBCube(const AABB& c);
	static void DrawOBBCube(const OBB& c);

	static GLuint m_sphereTex;
};
