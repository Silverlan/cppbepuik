// Copyright (c) 2023 Bepu Entertainment LLC
// Copyright (c) 2023 Silverlan
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/vec3.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace BEPUik
{
	using Quaternion = glm::quat;
	using Vector3 = glm::vec3;
	using Vector2 = glm::vec2;
	using Matrix3x3 = glm::mat3;

	/// <summary>
	/// Threshold value used for floating point comparisons.
	/// </summary>
	constexpr float Epsilon = 1e-7f;

	/// <summary>
	/// Approximate value of Pi.
	/// </summary>
	constexpr float Pi = 3.141592653589793239f;

	/// <summary>
	/// Approximate value of Pi multiplied by two.
	/// </summary>
	constexpr float TwoPi = 6.283185307179586477f;

	/// <summary>
	/// Approximate value of Pi divided by two.
	/// </summary>
	constexpr float PiOver2 = 1.570796326794896619f;

	/// <summary>
	/// Approximate value of Pi divided by four.
	/// </summary>
	constexpr float PiOver4 = 0.785398163397448310f;

	namespace matrix
	{
		Matrix3x3 Create(float value);

		/// <summary>
		/// Creates an empty matrix.
		/// </summary>
		/// <param name="result">Reference to store the identity matrix in.</param>
		Matrix3x3 Create();

		/// <summary>
		/// Creates an identity matrix.
		/// </summary>
		/// <param name="result">Reference to store the identity matrix in.</param>
		Matrix3x3 GetIdentity();

		/// <summary>
		/// Constructs a uniform scaling matrix.
		/// </summary>
		/// <param name="scale">Value to use in the diagonal.</param>
		/// <param name="matrix">Scaling matrix.</param>
		Matrix3x3 CreateScale(float scale);

		/// <summary>
		/// Scales the matrix.
		/// </summary>
		/// <param name="matrix">Matrix to scale.</param>
		/// <param name="scale">Amount to scale.</param>
		/// <param name="result">Scaled matrix.</param>
		Matrix3x3 Multiply(const Matrix3x3& a, const Matrix3x3& b);

		/// <summary>
		/// Multiplies a matrix with a transposed matrix.
		/// </summary>
		/// <param name="matrix">Matrix to be multiplied.</param>
		/// <param name="transpose">Matrix to be transposed and multiplied.</param>
		/// <param name="result">Product of the multiplication.</param>
		Matrix3x3 MultiplyByTransposed(const Matrix3x3& matrix, const Matrix3x3& transpose);

		/// <summary>
		/// Adds the two matrices together on a per-element basis.
		/// </summary>
		/// <param name="a">First matrix to add.</param>
		/// <param name="b">Second matrix to add.</param>
		/// <param name="result">Sum of the two matrices.</param>
		Matrix3x3 Add(const Matrix3x3& a, const Matrix3x3& b);

		/// <summary>
		/// Inverts the largest nonsingular submatrix in the matrix, excluding 2x2's that involve M13 or M31, and excluding 1x1's that include nondiagonal elements.
		/// </summary>
		/// <param name="matrix">Matrix to be inverted.</param>
		/// <param name="result">Inverted matrix.</param>
		Matrix3x3 AdaptiveInvert(const Matrix3x3& matrix);

		/// <summary>
		/// Transforms the vector by the matrix.
		/// </summary>
		/// <param name="v">Vector3 to transform.</param>
		/// <param name="matrix">Matrix to use as the transformation.</param>
		/// <param name="result">Product of the transformation.</param>
		Vector3 Transform(const Vector3& v, const Matrix3x3& matrix);

		/// <summary>
		/// Transforms the vector by the matrix's transpose.
		/// </summary>
		/// <param name="v">Vector3 to transform.</param>
		/// <param name="matrix">Matrix to use as the transformation transpose.</param>
		/// <param name="result">Product of the transformation.</param>
		Vector3 TransformTranspose(const Vector3& v, const Matrix3x3& matrix);

		/// <summary>
		/// Creates a skew symmetric matrix M from vector A such that M * B for some other vector B is equivalent to the cross product of A and B.
		/// </summary>
		/// <param name="v">Vector to base the matrix on.</param>
		/// <param name="result">Skew-symmetric matrix result.</param>
		Matrix3x3 CreateCrossProduct(const Vector3& v);

		/// <summary>
		/// Computes the transposed matrix of a matrix.
		/// </summary>
		/// <param name="matrix">Matrix to transpose.</param>
		/// <param name="result">Transposed matrix.</param>
		Matrix3x3 Transpose(const Matrix3x3& matrix);

		/// <summary>
		/// Negates every element in the matrix.
		/// </summary>
		/// <param name="matrix">Matrix to negate.</param>
		/// <param name="result">Negated matrix.</param>
		Matrix3x3 Negate(const Matrix3x3& matrix);

		/// <summary>
		/// Inverts the given matix.
		/// </summary>
		/// <param name="matrix">Matrix to be inverted.</param>
		/// <param name="result">Inverted matrix.</param>
		Matrix3x3 Invert(const Matrix3x3& matrix);

		/// <summary>
		/// Creates a 3x3 matrix representing the orientation stored in the quaternion.
		/// </summary>
		/// <param name="quaternion">Quaternion to use to create a matrix.</param>
		/// <param name="result">Matrix representing the quaternion's orientation.</param>
		Matrix3x3 CreateFromQuaternion(const Quaternion& quaternion);

		/// <summary>
		/// Multiplies a transposed matrix with another matrix.
		/// </summary>
		/// <param name="matrix">Matrix to be multiplied.</param>
		/// <param name="transpose">Matrix to be transposed and multiplied.</param>
		/// <param name="result">Product of the multiplication.</param>
		Matrix3x3 MultiplyTransposed(const Matrix3x3& transpose, const Matrix3x3& matrix);

		/// <summary>
		/// Calculates the determinant of the matrix.
		/// </summary>
		/// <returns>The matrix's determinant.</returns>
		float Determinant(const Matrix3x3& matrix);

		/// <summary>
		/// Calculates the determinant of largest nonsingular submatrix, excluding 2x2's that involve M13 or M31, and excluding all 1x1's that involve nondiagonal elements.
		/// </summary>
		/// <param name="subMatrixCode">Represents the submatrix that was used to compute the determinant.
		/// 0 is the full 3x3.  1 is the upper left 2x2.  2 is the lower right 2x2.  3 is the four corners.
		/// 4 is M11.  5 is M22.  6 is M33.</param>
		/// <returns>The matrix's determinant.</returns>
		float AdaptiveDeterminant(const Matrix3x3& matrix, int& subMatrixCode);
	};

	namespace vector3
	{
		/// <summary>
		/// Creates a new vector.
		/// </summary>
		/// <param name="a">First vector to add.</param>
		/// <param name="b">Second vector to add.</param>
		/// <param name="sum">Sum of the two vectors.</param>
		Vector3 Create();

		/// <summary>
		/// Adds two vectors together.
		/// </summary>
		/// <param name="a">First vector to add.</param>
		/// <param name="b">Second vector to add.</param>
		/// <param name="sum">Sum of the two vectors.</param>
		Vector3 Add(const Vector3& a, const Vector3& b);

		/// <summary>
		/// Subtracts two vectors.
		/// </summary>
		/// <param name="a">Vector to subtract from.</param>
		/// <param name="b">Vector to subtract from the first vector.</param>
		/// <param name="difference">Result of the subtraction.</param>
		Vector3 Subtract(const Vector3& a, const Vector3& b);

		/// <summary>
		/// Scales a vector.
		/// </summary>
		/// <param name="v">Vector to scale.</param>
		/// <param name="scale">Amount to scale.</param>
		/// <param name="result">Scaled vector.</param>
		Vector3 Multiply(const Vector3& v, float scale);

		/// <summary>
		/// Negates a vector.
		/// </summary>
		/// <param name="v">Vector to negate.</param>
		/// <param name="negated">Negated vector.</param>
		Vector3 Negate(const Vector3& v);

		/// <summary>
		/// Computes the cross product between two vectors.
		/// </summary>
		/// <param name="a">First vector.</param>
		/// <param name="b">Second vector.</param>
		/// <param name="result">Cross product of the two vectors.</param>
		Vector3 Cross(const Vector3& a, const Vector3& b);

		/// <summary>
		/// Computes the dot product of two vectors.
		/// </summary>
		/// <param name="a">First vector in the product.</param>
		/// <param name="b">Second vector in the product.</param>
		/// <param name="product">Resulting dot product.</param>
		float Dot(const Vector3& a, const Vector3& b);

		/// <summary>
		/// Computes the distance between two two vectors.
		/// </summary>
		/// <param name="a">First vector.</param>
		/// <param name="b">Second vector.</param>
		/// <param name="distance">Distance between the two vectors.</param>
		float Distance(const Vector3& a, const Vector3& b);

		/// <summary>
		/// Divides a vector's components by some amount.
		/// </summary>
		/// <param name="v">Vector to divide.</param>
		/// <param name="divisor">Value to divide the vector's components.</param>
		/// <param name="result">Result of the division.</param>
		Vector3 Divide(const Vector3& v, float divisor);

		/// <summary>
		/// Normalizes the given vector.
		/// </summary>
		/// <param name="v">Vector to normalize.</param>
		/// <param name="result">Normalized vector.</param>
		void Normalize(Vector3& v);

		/// <summary>
		/// Computes a vector with the maximum components of the given vectors.
		/// </summary>
		/// <param name="a">First vector.</param>
		/// <param name="b">Second vector.</param>
		/// <param name="result">Vector with the larger components of each input vector.</param>
		Vector3 Max(const Vector3& a, const Vector3& b);

		float Length(const Vector3& v);
		float LengthSqr(const Vector3& v);

		/// <summary>
		/// Vector pointing in the up direction.
		/// </summary>
		const Vector3 Up = Vector3(0, 1, 0);

		/// <summary>
		/// Vector pointing in the right direction.
		/// </summary>
		const Vector3 Right = Vector3(1, 0, 0);

		/// <summary>
		/// Vector with all components equal to zero.
		/// </summary>
		const Vector3 Zero = Vector3(0, 0, 0);
	};

	namespace quaternion
	{
		Quaternion Create(float x, float y, float z, float w);

		/// <summary>
		/// Transforms the vector using a quaternion.
		/// </summary>
		/// <param name="v">Vector to transform.</param>
		/// <param name="rotation">Rotation to apply to the vector.</param>
		/// <param name="result">Transformed vector.</param>
		Vector3 Transform(const Vector3& v, const Quaternion& rotation);

		/// <summary>
		/// Computes the conjugate of the quaternion.
		/// </summary>
		/// <param name="quaternion">Quaternion to conjugate.</param>
		/// <param name="result">Conjugated quaternion.</param>
		Quaternion Conjugate(const Quaternion& quaternion);

		/// <summary>
		/// Multiplies two quaternions.
		/// </summary>
		/// <param name="a">First quaternion to multiply.</param>
		/// <param name="b">Second quaternion to multiply.</param>
		/// <param name="result">Product of the multiplication.</param>
		Quaternion Multiply(const Quaternion& a, const Quaternion& b);

		/// <summary>
		/// Computes the axis angle representation of a normalized quaternion.
		/// </summary>
		/// <param name="q">Quaternion to be converted.</param>
		/// <param name="axis">Axis represented by the quaternion.</param>
		/// <param name="angle">Angle around the axis represented by the quaternion.</param>
		void GetAxisAngleFromQuaternion(const Quaternion& q, Vector3& axis, float& angle);

		/// <summary>
		/// Multiplies two quaternions together in opposite order.
		/// </summary>
		/// <param name="a">First quaternion to multiply.</param>
		/// <param name="b">Second quaternion to multiply.</param>
		/// <param name="result">Product of the multiplication.</param>
		Quaternion Concatenate(const Quaternion& a, const Quaternion& b);

		/// <summary>
		/// Computes the quaternion rotation between two normalized vectors.
		/// </summary>
		/// <param name="v1">First unit-length vector.</param>
		/// <param name="v2">Second unit-length vector.</param>
		/// <param name="q">Quaternion representing the rotation from v1 to v2.</param>
		Quaternion GetQuaternionBetweenNormalizedVectors(const Vector3& v1, const Vector3& v2);

		/// <summary>
		/// Scales the quaternion such that it has unit length.
		/// </summary>
		void Normalize(Quaternion& q);

		/// <summary>
		/// Adds two quaternions together.
		/// </summary>
		/// <param name="a">First quaternion to add.</param>
		/// <param name="b">Second quaternion to add.</param>
		/// <param name="result">Sum of the addition.</param>
		Quaternion Add(const Quaternion& a, const Quaternion& b);
	};

	const Quaternion quat_identity = quaternion::Create(1.f, 0.f, 0.f, 0.f);
};
