#include "Geometry2D.h"

math::Vector2D GetMin(const Rectangle2D& rect) {
	math::Vector2D p1 = rect.origin;
	math::Vector2D p2 = rect.origin + rect.size;

	return math::Vector2D(fminf(p1.x, p2.x), fminf(p1.y, p2.y));
}

math::Vector2D GetMax(const Rectangle2D& rect) {
	math::Vector2D p1 = rect.origin;
	math::Vector2D p2 = rect.origin + rect.size;

	return math::Vector2D(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y));
}

Rectangle2D FromMinMax(const math::Vector2D& min, const math::Vector2D& max) {
	return Rectangle2D(min, max - min);
}

bool RectangleRectangle(const Rectangle2D& rect1, const Rectangle2D& rect2) {
	math::Vector2D aMin = GetMin(rect1);
	math::Vector2D aMax = GetMax(rect1);
	math::Vector2D bMin = GetMin(rect2);
	math::Vector2D bMax = GetMax(rect2);

	bool xOverlap = ((bMin.x <= aMax.x) && (aMin.x <= bMax.x));
	bool yOverlap = ((bMin.y <= aMax.y) && (aMin.y <= bMax.y));

	return xOverlap && yOverlap;
}