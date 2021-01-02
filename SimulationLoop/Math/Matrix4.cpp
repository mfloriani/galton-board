#include "Matrix4.h"
#include "Matrix3.h"
#include "Vector3D.h"
#include "../Utils.h"

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

Matrix4 math::fromMatrix3(const Matrix3& mat) 
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

Matrix4 math::inverse(const Matrix4& m)
{
	/*float det = Determinant(m);
	if (CMP(det, 0.0f)) { return mat4(); }
	return Adjugate(m) * (1.0f / det);*/

	// The code below is the expanded form of the above equation.
	// This optimization avoids loops and function calls

	const float det
		= m.a() * m.f() * m.m() * m.r() + m.a() * m.g() * m.n() * m.p() + m.a() * m.h() * m.l() * m.q()
		+ m.b() * m.e() * m.n() * m.q() + m.b() * m.g() * m.k() * m.r() + m.b() * m.h() * m.m() * m.o()
		+ m.c() * m.e() * m.l() * m.r() + m.c() * m.f() * m.n() * m.o() + m.c() * m.h() * m.k() * m.p()
		+ m.d() * m.e() * m.m() * m.p() + m.d() * m.f() * m.k() * m.q() + m.d() * m.g() * m.l() * m.o()
		- m.a() * m.f() * m.n() * m.q() - m.a() * m.g() * m.l() * m.r() - m.a() * m.h() * m.m() * m.p()
		- m.b() * m.e() * m.m() * m.r() - m.b() * m.g() * m.n() * m.o() - m.b() * m.h() * m.k() * m.q()
		- m.c() * m.e() * m.n() * m.p() - m.c() * m.f() * m.k() * m.r() - m.c() * m.h() * m.l() * m.o()
		- m.d() * m.e() * m.l() * m.q() - m.d() * m.f() * m.m() * m.o() - m.d() * m.g() * m.k() * m.p();

	if (CMP(det, 0.0f)) {
		return Matrix4();
	}
	const float i_det = 1.0f / det;

	Matrix4 result;
	result.set(0,0, (m.f() * m.m() * m.r() + m.g() * m.n() * m.p() + m.h() * m.l() * m.q() - m.f() * m.n() * m.q() - m.g() * m.l() * m.r() - m.h() * m.m() * m.p()) * i_det);
	result.set(0,1, (m.b() * m.n() * m.q() + m.c() * m.l() * m.r() + m.d() * m.m() * m.p() - m.b() * m.m() * m.r() - m.c() * m.n() * m.p() - m.d() * m.l() * m.q()) * i_det);
	result.set(0,2, (m.b() * m.g() * m.r() + m.c() * m.h() * m.p() + m.d() * m.f() * m.q() - m.b() * m.h() * m.q() - m.c() * m.f() * m.r() - m.d() * m.g() * m.p()) * i_det);
	result.set(0,3, (m.b() * m.h() * m.m() + m.c() * m.f() * m.n() + m.d() * m.g() * m.l() - m.b() * m.g() * m.n() - m.c() * m.h() * m.l() - m.d() * m.f() * m.m()) * i_det);
	result.set(1,0, (m.e() * m.n() * m.q() + m.g() * m.k() * m.r() + m.h() * m.m() * m.o() - m.e() * m.m() * m.r() - m.g() * m.n() * m.o() - m.h() * m.k() * m.q()) * i_det);
	result.set(1,1, (m.a() * m.m() * m.r() + m.c() * m.n() * m.o() + m.d() * m.k() * m.q() - m.a() * m.n() * m.q() - m.c() * m.k() * m.r() - m.d() * m.m() * m.o()) * i_det);
	result.set(1,2, (m.a() * m.h() * m.q() + m.c() * m.e() * m.r() + m.d() * m.g() * m.o() - m.a() * m.g() * m.r() - m.c() * m.h() * m.o() - m.d() * m.e() * m.q()) * i_det);
	result.set(1,3, (m.a() * m.g() * m.n() + m.c() * m.h() * m.k() + m.d() * m.e() * m.m() - m.a() * m.h() * m.m() - m.c() * m.e() * m.n() - m.d() * m.g() * m.k()) * i_det);
	result.set(2,0, (m.e() * m.l() * m.r() + m.f() * m.n() * m.o() + m.h() * m.k() * m.p() - m.e() * m.n() * m.p() - m.f() * m.k() * m.r() - m.h() * m.l() * m.o()) * i_det);
	result.set(2,1, (m.a() * m.n() * m.p() + m.b() * m.k() * m.r() + m.d() * m.l() * m.o() - m.a() * m.l() * m.r() - m.b() * m.n() * m.o() - m.d() * m.k() * m.p()) * i_det);
	result.set(2,2, (m.a() * m.f() * m.r() + m.b() * m.h() * m.o() + m.d() * m.e() * m.p() - m.a() * m.h() * m.p() - m.b() * m.e() * m.r() - m.d() * m.f() * m.o()) * i_det);
	result.set(2,3, (m.a() * m.h() * m.l() + m.b() * m.e() * m.n() + m.d() * m.f() * m.k() - m.a() * m.f() * m.n() - m.b() * m.h() * m.k() - m.d() * m.e() * m.l()) * i_det);
	result.set(3,0, (m.e() * m.m() * m.p() + m.f() * m.k() * m.q() + m.g() * m.l() * m.o() - m.e() * m.l() * m.q() - m.f() * m.m() * m.o() - m.g() * m.k() * m.p()) * i_det);
	result.set(3,1, (m.a() * m.l() * m.q() + m.b() * m.m() * m.o() + m.c() * m.k() * m.p() - m.a() * m.m() * m.p() - m.b() * m.k() * m.q() - m.c() * m.l() * m.o()) * i_det);
	result.set(3,2, (m.a() * m.g() * m.p() + m.b() * m.e() * m.q() + m.c() * m.f() * m.o() - m.a() * m.f() * m.q() - m.b() * m.g() * m.o() - m.c() * m.e() * m.p()) * i_det);
	result.set(3,3, (m.a() * m.f() * m.m() + m.b() * m.g() * m.k() + m.c() * m.e() * m.l() - m.a() * m.g() * m.l() - m.b() * m.e() * m.m() - m.c() * m.f() * m.k()) * i_det);

	/*if (result * m != mat4()) {
		std::cout << "ERROR! Expecting matrix x inverse to equal identity!\n";
	}*/

	return result;
}

Vector3D math::multiplyVector(const Vector3D& vec, const Matrix4& mat)
{
	Vector3D result;
	result.x = vec.x * mat.a() + vec.y * mat.e() + vec.z * mat.k() + 0.0f * mat.o();
	result.y = vec.x * mat.b() + vec.y * mat.f() + vec.z * mat.l() + 0.0f * mat.p();
	result.z = vec.x * mat.c() + vec.y * mat.g() + vec.z * mat.m() + 0.0f * mat.q();
	return result;
}