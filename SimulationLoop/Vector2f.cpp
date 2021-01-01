#include "Vector2f.h"
#define _USE_MATH_DEFINES
#include <math.h>

Vector2f::Vector2f(void) 
{
}

Vector2f::Vector2f(float x, float y) : x(x), y(y) 
{
}

Vector2f::~Vector2f(void)
{
}

Vector2f Vector2f::add(const Vector2f &vec) const
{
	return Vector2f(x + vec.x, y + vec.y);
}

Vector2f Vector2f::subtract(const Vector2f &vec) const
{
	return Vector2f(x - vec.x, y - vec.y);
}

Vector2f Vector2f::mult(float n) const
{
	return Vector2f(x*n, y*n);
}

Vector2f Vector2f::divide(float n) const
{
	return Vector2f(x/n, y/n);
}

float Vector2f::dot(const Vector2f &vec) const
{
	return x*vec.x + y*vec.y;
}

float Vector2f::length() const
{
	return sqrtf(x*x + y*y);
}

float Vector2f::lengthSq() const
{
	return x * x + y * y;
}

float Vector2f::distance(const Vector2f &vec) const
{
	return subtract(vec).length();
}

Vector2f Vector2f::normalise()
{
	float len = length();
	x = x / len;
	y = y / len;
	return *this;
}


Vector2f operator+ (const Vector2f &lhs, const Vector2f &rhs)
{
	return lhs.add(rhs);
}

Vector2f operator- (const Vector2f &lhs, const Vector2f &rhs)
{
	return lhs.subtract(rhs);
}

Vector2f operator* (const Vector2f &lhs, float n)
{
	return lhs.mult(n);
}

Vector2f operator* (float n, const Vector2f &rhs)
{
	return rhs.mult(n);
}

Vector2f operator/ (const Vector2f &lhs, float n)
{
	return lhs.divide(n);
}