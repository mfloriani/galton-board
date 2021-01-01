#pragma once
class Vector2f
{
public:
	Vector2f(void);
	Vector2f(float x, float y);
	~Vector2f(void);

	Vector2f add(const Vector2f &vec) const;
	Vector2f subtract(const Vector2f &vec) const;
	Vector2f mult(float n) const;
	Vector2f divide(float n) const;
	float dot(const Vector2f &vec) const;
	float length() const;
	float lengthSq() const;
	float distance(const Vector2f &vec) const;
	Vector2f normalise();

public:
	float x, y;
};

Vector2f operator+ (const Vector2f &lhs, const Vector2f &rhs);
Vector2f operator- (const Vector2f &lhs, const Vector2f &rhs);
Vector2f operator* (const Vector2f &lhs, float n);
Vector2f operator* (float n, const Vector2f &lhs);
Vector2f operator/ (const Vector2f &lhs, float n);