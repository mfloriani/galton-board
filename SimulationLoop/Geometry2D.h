#pragma once

#include "Math\Vector2D.h"

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
