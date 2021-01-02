#pragma once

namespace math
{
	class Vector3D;
	class Matrix3;

	class Matrix4
	{
	private:
		float A[4][4];

	public:
		Matrix4();

		Matrix4(float a, float b, float c, float d,
			    float e, float f, float g, float h,
			    float k, float l, float m, float n,
			    float o, float p, float q, float r);

		Matrix4(const Matrix4& other);

		Matrix4& operator=(const Matrix4& other);

		static Matrix4 newIdentity();

		inline float get(int row, int col) const
		{
			return A[row][col];
		}

		Matrix4& set(
			float a, float b, float c, float d, 
			float e, float f, float g, float h, 
			float k, float l, float m, float n,
			float o, float p, float q, float r
		);

		inline Matrix4& set(int row, int col, float value)
		{
			A[row][col] = value;
			return *this;
		}

		inline float a() const { return A[0][0]; }
		
		inline float b() const { return A[0][1]; }

		inline float c() const { return A[0][2]; }

		inline float d() const { return A[0][3]; }
		
		inline float e() const { return A[1][0]; }

		inline float f() const { return A[1][1]; }

		inline float g() const { return A[1][2]; }

		inline float h() const { return A[1][3]; }

		inline float k() const { return A[2][0]; }
		
		inline float l() const { return A[2][1]; }
		
		inline float m() const { return A[2][2]; }
		
		inline float n() const { return A[2][3]; }

		inline float o() const { return A[3][0]; }

		inline float p() const { return A[3][1]; }

		inline float q() const { return A[3][2]; }

		inline float r() const { return A[3][3]; }

		inline float& operator() (int row, int col)
		{
			return A[row][col];
		}

		float operator() (int row, int col) const
		{
			return A[row][col];
		}

		Matrix4& operator += (const Matrix4& other);
		Matrix4 operator + (const Matrix4& other) const;
		Matrix4& operator -= (const Matrix4& other);
		Matrix4 operator - (const Matrix4& other) const;
		Matrix4& operator *= (const Matrix4& other);
		Matrix4 operator * (const Matrix4& other) const;
		Matrix4& operator *= (float scalar);
		Matrix4 operator * (float scalar) const;

		Matrix4& operator /= (float scalar);
		Matrix4 operator / (float scalar) const;

		bool operator == (Matrix4& other) const;
		bool operator != (Matrix4& other) const;

		float* data() { return *A; }
	};

	Matrix4 operator *(float scalar, const Matrix4& matrix);
	Matrix4 operator /(float scalar, const Matrix4& matrix);
	Matrix4 transpose(const Matrix4& matrix);
	Matrix4 scale(const Vector3D& vec);
	Matrix4 translation(const Vector3D& pos);
	Matrix4 fromMatrix3(const Matrix3& mat);
	Matrix4 inverse(const Matrix4& m);
	Vector3D multiplyVector(const Vector3D& vec, const Matrix4& mat);
}