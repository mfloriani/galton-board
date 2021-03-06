/******************************************************************************
*
* COPYRIGHT Vin�cius G. Mendon�a ALL RIGHTS RESERVED.
*
* This software cannot be copied, stored, distributed without  
* Vin�cius G.Mendon�a prior authorization.
*
* This file was made available on http://www.pontov.com.br and it is free 
* to be restributed or used under Creative Commons license 2.5 br: 
* http://creativecommons.org/licenses/by-sa/2.5/br/
*
*******************************************************************************
* Este software nao pode ser copiado, armazenado, distribuido sem autoriza��o 
* a priori de Vin�cius G. Mendon�a
*
* Este arquivo foi disponibilizado no site http://www.pontov.com.br e esta 
* livre para distribui��o seguindo a licen�a Creative Commons 2.5 br: 
* http://creativecommons.org/licenses/by-sa/2.5/br/
*
******************************************************************************/
#ifndef __MATRIX3_H__
#define __MATRIX3_H__

namespace math
{
	class Vector2D;
	class Vector3D;

	/**
	* Represents a 3x3 matrix.
	* <p>
	* All mathematical operator methods comes with two signatures. The first one creates a copy of the matrix, prior to the
	* operation. The second one, suffixed with "Me", applies the operation the calling matrix.
	* <p>
	* This class also provides convenient access methods to retrieve or set the matrix elements as:
	*
	* <pre>
	* [a b c]
	* [d e f]
	* [g h k]
	* </pre>
	*
	* These methods are named {@link #a()}, {@link #b()}, {@link #c()}, {@link #d()}, {@link #e()},
	* {@link #f()}, {@link #g()}, {@link #h()} and {@link #k()}.
	* <p>
	* Finally, this class provides methods to use this matrix as an 2D column oriented Affine transformation. 
	* Several static factory methods are provided, and the transform method allows matrix by 2D vector 
	* multiplication.
	*
	* @author Vinicius G. Mendonca
	*/
	class Matrix3 
	{
		private:
			float A[3][3];

		public:
			Matrix3();

			Matrix3(float a, float b, float c, 
					 float d, float e, float f, 
					 float g, float h, float k);			
				
			/**
			 * Creates an identity matrix
			 */
			static Matrix3 newIdentity();


			/** Copy constructor */
			Matrix3(const Matrix3& other);

			/** Assignment operator */
			Matrix3& operator=(const Matrix3& other);

			/**
			* Returns the element on the given row and column.
			*
			* @param row The row to retrieve
			* @param col The column to retrieve
			* @return The element value
			*/
			inline float get(int row, int col) const
			{
				return A[row][col];
			}

			/**
			* Change the value of the element in the given row and column
			*
			* @param row The row
			* @param col The column
			* @param value The value to set.
			* @return This matrix.
			*/
			inline Matrix3& set(int row, int col, float value)
			{
				A[row][col] = value;
				return *this;
			}

			Matrix3& set(
				float a, float b, float c,
				float d, float e, float f,
				float g, float h, float k);
			/**
			* Returns the "a" element in the following mapping:
			*
			* <pre>
			* [a b c]
			* [d e f]
			* [g h k]
			* </pre>
			*
			* @return the A[0][0] element.
			*/
			inline float a() const { return A[0][0]; }
			/**
			* Returns the "b" element in the following mapping:
			*
			* <pre>
			* [a b c]
			* [d e f]
			* [g h k]
			* </pre>
			*
			* @return the A[0][1] element.
			*/
			inline float b() const { return A[0][1]; }

			/**
			* Returns the "c" element in the following mapping:
			*
			* <pre>
			* [a b c]
			* [d e f]
			* [g h k]
			* </pre>
			*
			* @return the A[0][2] element.
			*/
			inline float c() const { return A[0][2]; }

			/**
			* Returns the "d" element in the following mapping:
			*
			* <pre>
			* [a b c]
			* [d e f]
			* [g h k]
			* </pre>
			*
			* @return the A[1][0] element.
			*/
			inline float d() const { return A[1][0]; }

			/**
			* Returns the "e" element in the following mapping:
			*
			* <pre>
			* [a b c]
			* [d e f]
			* [g h k]
			* </pre>
			*
			* @return the A[1][1] element.
			*/
			inline float e() const { return A[1][1]; }

			/**
			* Returns the "e" element in the following mapping:
			*
			* <pre>
			* [a b c]
			* [d e f]
			* [g h k]
			* </pre>
			*
			* @return the A[1][1] element.
			*/
			inline float f() const { return A[1][2]; }

			/**
			* Returns the "g" element in the following mapping:
			*
			* <pre>
			* [a b c]
			* [d e f]
			* [g h k]
			* </pre>
			*
			* @return the A[2][0] element.
			*/
			inline float g() const { return A[2][0]; }

			/**
			* Returns the "h" element in the following mapping:
			*
			* <pre>
			* [a b c]
			* [d e f]
			* [g h k]
			* </pre>
			*
			* @return the A[2][1] element.
			*/
			inline float h() const { return A[2][1]; }

	
			/**
			* Returns the "k" element in the following mapping:
			*
			* <pre>
			* [a b c]
			* [d e f]
			* [g h k]
			* </pre>
			*
			* @return the A[2][2] element.
			*/
			inline float k() const { return A[2][2]; }

			inline float& operator() (int row, int col)
			{
				return A[row][col];
			}

			float operator() (int row, int col) const
			{
				return A[row][col];
			}

			Matrix3& operator += (const Matrix3& other);
			Matrix3 operator + (const Matrix3& other) const;
			Matrix3& operator -= (const Matrix3& other);
			Matrix3 operator - (const Matrix3& other) const;
			Matrix3& operator *= (const Matrix3& other);
			Matrix3 operator * (const Matrix3& other) const;		
			Matrix3& operator *= (float scalar);
			Matrix3 operator * (float scalar) const;
            
			Matrix3& operator /= (float scalar);
			Matrix3 operator / (float scalar) const;
            
			bool operator == (Matrix3& other) const;
			bool operator != (Matrix3& other) const;

			/**
			 * Calculates and returns the determinant of this matrix.		 
			 */
			float determinant() const;

			/**
			* A matrix is invertible if its determinant is not 0.
			* @return True if this matrix is invertible, false if not. 
			*/
			bool isInvertible() const;
            
			/**
			* Calculate the adjoint matrix. 
			*
			* @return The adjoint matrix
			* @see #inverse()
			*/
			Matrix3& adjoint();

			/**
			* Inverses this matrix. Not all matrices are invertible, so check the {@link #isInvertible()} method prior to
			* calling this method if you are not sure if you can inverse this matrix or not.
			*
			* <pre>
			* this * inverse = identity
			* </pre>
			*
			* @return This matrix, after inversion. If the matrix is not invertible, the result will be undefined.
			* @see #isInvertible()
			* @see #inverse()
			*/
			Matrix3& inverse();

			/**
			* Transposes this matrix. If this matrix is:
			*
			* <pre>
			* [a b c]
			* [d e f]
			* [g h k]
			* </pre>
			*
			* After transposition it will become:
			*
			* <pre>
			* [a d g]
			* [b e h]
			* [c f k]
			* </pre>
			*
			* @return This matrix.
			*/
			Matrix3& transpose();

			float* data() { return *A; }
	};

	Matrix3 operator *(float scalar, const Matrix3& matrix);
    Matrix3 operator /(float scalar, const Matrix3& matrix);
	Matrix3 transpose(const Matrix3& matrix);
	Matrix3 adjoint(const Matrix3& matrix);
	Matrix3 inverse(const Matrix3& matrix);
	Matrix3 xRotation3x3(float angle);
	Matrix3 yRotation3x3(float angle);
	Matrix3 zRotation3x3(float angle);
	Matrix3 rotation3x3(float pitch, float yaw, float roll);
	

}

#endif
