#pragma once

#include "TextureLoader.h"
#include "Geometry.h"
#include "Geometry2D.h"


class Renderer
{
public:
	static bool Init();
	static void DrawRect(const Rectangle2D& r, math::Vector3D c);
	static void DrawSphere(const Sphere& s);
	static void DrawAABBCube(const AABB& c);
	static void DrawOBBCube(const OBB& c);
	
	static GLuint m_sphereTex;

};
