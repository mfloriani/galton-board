#include "Matrix4.h"
#include "Matrix3.h"
#include "Vector3D.h"

#include <algorithm>

using namespace math;

Matrix4::Matrix4()
{
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			A[i][j] = 0;
}

Matrix4::Matrix4(
	float a, float b, float c, float d,
	float e, float f, float g, float h,
	float k, float l, float m, float n,
	float o, float p, float q, float r
)
{
	set(a, b, c, d, 
		e, f, g, h, 
		k, l, m, n,
		o, p, q, r);
}

Matrix4::Matrix4(const Matrix4& other)
{
	memcpy(A, (void*)(other.A), sizeof(other.A));
}

Matrix4& Matrix4::operator=(const Matrix4& other)
{
	return set(other.a(), other.b(), other.c(), other.d(), 
		       other.e(), other.f(), other.g(), other.h(), 
		       other.k(), other.l(), other.m(), other.n(),
		       other.o(), other.p(), other.q(), other.r());
}

Matrix4 Matrix4::newIdentity()
{
	return Matrix4(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	);
}

Matrix4& Matrix4::set(
	float a, float b, float c, float d,
	float e, float f, float g, float h,
	float k, float l, float m, float n,
	float o, float p, float q, float r
)
{
	A[0][0] = a;
	A[0][1] = b;
	A[0][2] = c;
	A[0][3] = d;

	A[1][0] = e;
	A[1][1] = f;
	A[1][2] = g;
	A[1][3] = h;

	A[2][0] = k;
	A[2][1] = l;
	A[2][2] = m;
	A[2][3] = n;

	A[3][0] = o;
	A[3][1] = p;
	A[3][2] = q;
	A[3][3] = r;

	return *this;
}

Matrix4& Matrix4::operator += (const Matrix4& other)
{
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			A[i][j] += other.A[i][j];

	return *this;
}

Matrix4 Matrix4::operator + (const Matrix4& other) const
{
	return Matrix4(*this) += other;
}

Matrix4& Matrix4::operator -= (const Matrix4& other)
{
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			A[i][j] -= other.A[i][j];

	return *this;
}

Matrix4 Matrix4::operator - (const Matrix4& other) const
{
	return Matrix4(*this) -= other;
}

Matrix4& Matrix4::operator *= (const Matrix4& other)
{
	return *this = (*this * other);
}

Matrix4 Matrix4::operator * (const Matrix4& other) const
{
	Matrix4 C;
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			for (int k = 0; k < 4; ++k)
				C.A[i][j] += A[i][k] * other.A[k][j];

	return C;
}

Matrix4 math::operator *(float scalar, const Matrix4& matrix)
{
	return matrix * scalar;
}

Matrix4& Matrix4::operator *= (float scalar)
{
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			A[i][j] *= scalar;

	return *this;
}

Matrix4 Matrix4::operator * (float scalar) const
{
	return Matrix4(*this) *= scalar;
}

Matrix4 math::operator /(float scalar, const Matrix4& matrix)
{
	return matrix / scalar;
}

Matrix4& Matrix4::operator /= (float scalar)
{
	Matrix4& self = *this;
	self *= 1.0f / scalar;
	return self;
}

Matrix4 Matrix4::operator / (float scalar) const
{
	return Matrix4(*this) /= scalar;
}

bool Matrix4::operator == (Matrix4& other) const
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			if (A[i][j] != other.A[i][j])
				return false;
	return true;
}

bool Matrix4::operator != (Matrix4& other) const
{
	return !(*this == other);
}

Matrix4 math::transpose(const Matrix4& matrix)
{
	Matrix4 other;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			other(i, j) = matrix(j, i);

	return other;
}

Matrix4 math::scale(const Vector3D& vec)
{
	return Matrix4(
		vec.x, 0.0f, 0.0f, 0.0f,
		0.0f, vec.y, 0.0f, 0.0f,
		0.0f, 0.0f, vec.z, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	);
}

Matrix4 math::translation(const Vector3D& pos)
{
	return Matrix4(
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		pos.x, pos.y, pos.z, 1.0f
	);
}

Matrix4 math::FromMatrix3(const Matrix3& mat) 
{
	Matrix4 result = Matrix4::newIdentity();

	result.set(0, 0, mat.get(0, 0));
	result.set(0, 1, mat.get(0, 1));
	result.set(0, 2, mat.get(0, 2));

	result.set(1, 0, mat.get(1, 0));
	result.set(1, 1, mat.get(1, 1));
	result.set(1, 2, mat.get(1, 2));

	result.set(2, 0, mat.get(2, 0));
	result.set(2, 1, mat.get(2, 1));
	result.set(2, 2, mat.get(2, 2));

	return result;
}