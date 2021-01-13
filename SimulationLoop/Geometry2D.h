#pragma once

#include "Math\Vector2D.h"
#include "Math\Vector3D.h"

struct Rectangle2D
{
	math::Vector2D origin;
	math::Vector2D size;

	Rectangle2D() : size(1, 1) { }
	Rectangle2D(const math::Vector2D& o, const math::Vector2D& s) : origin(o), size(s) { }
	

};

math::Vector2D GetMin(const Rectangle2D& rect);
math::Vector2D GetMax(const Rectangle2D& rect);
Rectangle2D FromMinMax(const math::Vector2D& min, const math::Vector2D& max);
bool RectangleRectangle(const Rectangle2D& rect1, const Rectangle2D& rect2);

void FillRect2dFrom3d(Rectangle2D& rect, const math::Vector3D& pos, const math::Vector3D& size);
void FillRect2dFrom3d(Rectangle2D& rect, const math::Vector3D& pos, float radius);
void FillRect2dOriginFrom3d(math::Vector2D& origin, const math::Vector3D& pos, const math::Vector3D& size);
void FillRect2dOriginFrom3d(math::Vector2D& origin, const math::Vector3D& pos, float radius);