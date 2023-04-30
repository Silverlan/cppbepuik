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

#include "bepuik/math.hpp"

BEPUik::Matrix3x3 BEPUik::matrix::Create(float value)
{
	return BEPUik::Matrix3x3{
		value,0.f,0.f,
		0.f,value,0.f,
		0.f,0.f,value
	};
}

BEPUik::Matrix3x3 BEPUik::matrix::Create()
{
	return Create(0.f);
}

BEPUik::Matrix3x3 BEPUik::matrix::GetIdentity()
{
	Matrix3x3 result;
	result[0][0] = 1;
	result[0][1] = 0;
	result[0][2] = 0;
	result[1][0] = 0;
	result[1][1] = 1;
	result[1][2] = 0;
	result[2][0] = 0;
	result[2][1] = 0;
	result[2][2] = 1;
	return result;
}

BEPUik::Matrix3x3 BEPUik::matrix::CreateScale(float scale)
{
	Matrix3x3 matrix;
	matrix[0][0] = scale;
	matrix[0][1] = 0;
	matrix[0][2] = 0;

	matrix[1][0] = 0;
	matrix[1][1] = scale;
	matrix[1][2] = 0;

	matrix[2][0] = 0;
	matrix[2][1] = 0;
	matrix[2][2] = scale;
	return matrix;
}

BEPUik::Matrix3x3 BEPUik::matrix::Multiply(const Matrix3x3& a, const Matrix3x3& b)
{
	float resultM11 = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0];
	float resultM12 = a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1];
	float resultM13 = a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2];

	float resultM21 = a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0];
	float resultM22 = a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1];
	float resultM23 = a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2];

	float resultM31 = a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0];
	float resultM32 = a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1];
	float resultM33 = a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2];

	Matrix3x3 result;
	result[0][0] = resultM11;
	result[0][1] = resultM12;
	result[0][2] = resultM13;

	result[1][0] = resultM21;
	result[1][1] = resultM22;
	result[1][2] = resultM23;

	result[2][0] = resultM31;
	result[2][1] = resultM32;
	result[2][2] = resultM33;
	return result;
}

BEPUik::Matrix3x3 BEPUik::matrix::MultiplyByTransposed(const Matrix3x3& matrix, const Matrix3x3& transpose)
{
	float resultM11 = matrix[0][0] * transpose[0][0] + matrix[0][1] * transpose[0][1] + matrix[0][2] * transpose[0][2];
	float resultM12 = matrix[0][0] * transpose[1][0] + matrix[0][1] * transpose[1][1] + matrix[0][2] * transpose[1][2];
	float resultM13 = matrix[0][0] * transpose[2][0] + matrix[0][1] * transpose[2][1] + matrix[0][2] * transpose[2][2];

	float resultM21 = matrix[1][0] * transpose[0][0] + matrix[1][1] * transpose[0][1] + matrix[1][2] * transpose[0][2];
	float resultM22 = matrix[1][0] * transpose[1][0] + matrix[1][1] * transpose[1][1] + matrix[1][2] * transpose[1][2];
	float resultM23 = matrix[1][0] * transpose[2][0] + matrix[1][1] * transpose[2][1] + matrix[1][2] * transpose[2][2];

	float resultM31 = matrix[2][0] * transpose[0][0] + matrix[2][1] * transpose[0][1] + matrix[2][2] * transpose[0][2];
	float resultM32 = matrix[2][0] * transpose[1][0] + matrix[2][1] * transpose[1][1] + matrix[2][2] * transpose[1][2];
	float resultM33 = matrix[2][0] * transpose[2][0] + matrix[2][1] * transpose[2][1] + matrix[2][2] * transpose[2][2];

	Matrix3x3 result;
	result[0][0] = resultM11;
	result[0][1] = resultM12;
	result[0][2] = resultM13;

	result[1][0] = resultM21;
	result[1][1] = resultM22;
	result[1][2] = resultM23;

	result[2][0] = resultM31;
	result[2][1] = resultM32;
	result[2][2] = resultM33;
	return result;
}

BEPUik::Matrix3x3 BEPUik::matrix::Add(const Matrix3x3& a, const Matrix3x3& b)
{
	float m11 = a[0][0] + b[0][0];
	float m12 = a[0][1] + b[0][1];
	float m13 = a[0][2] + b[0][2];

	float m21 = a[1][0] + b[1][0];
	float m22 = a[1][1] + b[1][1];
	float m23 = a[1][2] + b[1][2];

	float m31 = a[2][0] + b[2][0];
	float m32 = a[2][1] + b[2][1];
	float m33 = a[2][2] + b[2][2];

	Matrix3x3 result;
	result[0][0] = m11;
	result[0][1] = m12;
	result[0][2] = m13;

	result[1][0] = m21;
	result[1][1] = m22;
	result[1][2] = m23;

	result[2][0] = m31;
	result[2][1] = m32;
	result[2][2] = m33;
	return result;
}

BEPUik::Matrix3x3 BEPUik::matrix::AdaptiveInvert(const Matrix3x3& matrix)
{
	int submatrix;
	float determinantInverse = 1 / AdaptiveDeterminant(matrix, submatrix);
	float m11, m12, m13, m21, m22, m23, m31, m32, m33;
	switch (submatrix)
	{
	case 0: //Full matrix.
		m11 = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) * determinantInverse;
		m12 = (matrix[0][2] * matrix[2][1] - matrix[2][2] * matrix[0][1]) * determinantInverse;
		m13 = (matrix[0][1] * matrix[1][2] - matrix[1][1] * matrix[0][2]) * determinantInverse;

		m21 = (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) * determinantInverse;
		m22 = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * determinantInverse;
		m23 = (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) * determinantInverse;

		m31 = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) * determinantInverse;
		m32 = (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) * determinantInverse;
		m33 = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) * determinantInverse;
		break;
	case 1: //Upper left matrix, m11, m12, m21, m22.
		m11 = matrix[1][1] * determinantInverse;
		m12 = -matrix[0][1] * determinantInverse;
		m13 = 0;

		m21 = -matrix[1][0] * determinantInverse;
		m22 = matrix[0][0] * determinantInverse;
		m23 = 0;

		m31 = 0;
		m32 = 0;
		m33 = 0;
		break;
	case 2: //Lower right matrix, m22, m23, m32, m33.
		m11 = 0;
		m12 = 0;
		m13 = 0;

		m21 = 0;
		m22 = matrix[2][2] * determinantInverse;
		m23 = -matrix[1][2] * determinantInverse;

		m31 = 0;
		m32 = -matrix[2][1] * determinantInverse;
		m33 = matrix[1][1] * determinantInverse;
		break;
	case 3: //Corners, m11, m31, m13, m33.
		m11 = matrix[2][2] * determinantInverse;
		m12 = 0;
		m13 = -matrix[0][2] * determinantInverse;

		m21 = 0;
		m22 = 0;
		m23 = 0;

		m31 = -matrix[2][0] * determinantInverse;
		m32 = 0;
		m33 = matrix[0][0] * determinantInverse;
		break;
	case 4: //M11
		m11 = 1 / matrix[0][0];
		m12 = 0;
		m13 = 0;

		m21 = 0;
		m22 = 0;
		m23 = 0;

		m31 = 0;
		m32 = 0;
		m33 = 0;
		break;
	case 5: //M22
		m11 = 0;
		m12 = 0;
		m13 = 0;

		m21 = 0;
		m22 = 1 / matrix[1][1];
		m23 = 0;

		m31 = 0;
		m32 = 0;
		m33 = 0;
		break;
	case 6: //M33
		m11 = 0;
		m12 = 0;
		m13 = 0;

		m21 = 0;
		m22 = 0;
		m23 = 0;

		m31 = 0;
		m32 = 0;
		m33 = 1 / matrix[2][2];
		break;
	default: //Completely singular.
		m11 = 0; m12 = 0; m13 = 0; m21 = 0; m22 = 0; m23 = 0; m31 = 0; m32 = 0; m33 = 0;
		break;
	}

	Matrix3x3 result;
	result[0][0] = m11;
	result[0][1] = m12;
	result[0][2] = m13;

	result[1][0] = m21;
	result[1][1] = m22;
	result[1][2] = m23;

	result[2][0] = m31;
	result[2][1] = m32;
	result[2][2] = m33;
	return result;
}

BEPUik::Vector3 BEPUik::matrix::Transform(const Vector3& v, const Matrix3x3& matrix)
{
	float vX = v.x;
	float vY = v.y;
	float vZ = v.z;

	Vector3 result;
	result.x = vX * matrix[0][0] + vY * matrix[1][0] + vZ * matrix[2][0];
	result.y = vX * matrix[0][1] + vY * matrix[1][1] + vZ * matrix[2][1];
	result.z = vX * matrix[0][2] + vY * matrix[1][2] + vZ * matrix[2][2];
	return result;
}

BEPUik::Vector3 BEPUik::matrix::TransformTranspose(const Vector3& v, const Matrix3x3& matrix)
{
	float vX = v.x;
	float vY = v.y;
	float vZ = v.z;

	Vector3 result;
	result.x = vX * matrix[0][0] + vY * matrix[0][1] + vZ * matrix[0][2];
	result.y = vX * matrix[1][0] + vY * matrix[1][1] + vZ * matrix[1][2];
	result.z = vX * matrix[2][0] + vY * matrix[2][1] + vZ * matrix[2][2];
	return result;
}

BEPUik::Matrix3x3 BEPUik::matrix::CreateCrossProduct(const Vector3& v)
{
	Matrix3x3 result;
	result[0][0] = 0;
	result[0][1] = -v.z;
	result[0][2] = v.y;
	result[1][0] = v.z;
	result[1][1] = 0;
	result[1][2] = -v.x;
	result[2][0] = -v.y;
	result[2][1] = v.x;
	result[2][2] = 0;
	return result;
}

BEPUik::Matrix3x3 BEPUik::matrix::Transpose(const Matrix3x3& matrix)
{
	float m21 = matrix[0][1];
	float m31 = matrix[0][2];
	float m12 = matrix[1][0];
	float m32 = matrix[1][2];
	float m13 = matrix[2][0];
	float m23 = matrix[2][1];

	Matrix3x3 result;
	result[0][0] = matrix[0][0];
	result[0][1] = m12;
	result[0][2] = m13;
	result[1][0] = m21;
	result[1][1] = matrix[1][1];
	result[1][2] = m23;
	result[2][0] = m31;
	result[2][1] = m32;
	result[2][2] = matrix[2][2];
	return result;
}

BEPUik::Matrix3x3 BEPUik::matrix::Negate(const Matrix3x3& matrix)
{
	Matrix3x3 result;
	result[0][0] = -matrix[0][0];
	result[0][1] = -matrix[0][1];
	result[0][2] = -matrix[0][2];

	result[1][0] = -matrix[1][0];
	result[1][1] = -matrix[1][1];
	result[1][2] = -matrix[1][2];

	result[2][0] = -matrix[2][0];
	result[2][1] = -matrix[2][1];
	result[2][2] = -matrix[2][2];
	return result;
}

BEPUik::Matrix3x3 BEPUik::matrix::Invert(const Matrix3x3& matrix)
{
	float determinantInverse = 1 / Determinant(matrix);
	float m11 = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) * determinantInverse;
	float m12 = (matrix[0][2] * matrix[2][1] - matrix[2][2] * matrix[0][1]) * determinantInverse;
	float m13 = (matrix[0][1] * matrix[1][2] - matrix[1][1] * matrix[0][2]) * determinantInverse;

	float m21 = (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) * determinantInverse;
	float m22 = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * determinantInverse;
	float m23 = (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) * determinantInverse;

	float m31 = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) * determinantInverse;
	float m32 = (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) * determinantInverse;
	float m33 = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) * determinantInverse;

	Matrix3x3 result;
	result[0][0] = m11;
	result[0][1] = m12;
	result[0][2] = m13;

	result[1][0] = m21;
	result[1][1] = m22;
	result[1][2] = m23;

	result[2][0] = m31;
	result[2][1] = m32;
	result[2][2] = m33;
	return result;
}

BEPUik::Matrix3x3 BEPUik::matrix::CreateFromQuaternion(const Quaternion& quaternion)
{
	float XX = 2 * quaternion.x * quaternion.x;
	float YY = 2 * quaternion.y * quaternion.y;
	float ZZ = 2 * quaternion.z * quaternion.z;
	float XY = 2 * quaternion.x * quaternion.y;
	float XZ = 2 * quaternion.x * quaternion.z;
	float XW = 2 * quaternion.x * quaternion.w;
	float YZ = 2 * quaternion.y * quaternion.z;
	float YW = 2 * quaternion.y * quaternion.w;
	float ZW = 2 * quaternion.z * quaternion.w;

	Matrix3x3 result;
	result[0][0] = 1 - YY - ZZ;
	result[1][0] = XY - ZW;
	result[2][0] = XZ + YW;

	result[0][1] = XY + ZW;
	result[1][1] = 1 - XX - ZZ;
	result[2][1] = YZ - XW;

	result[0][2] = XZ - YW;
	result[1][2] = YZ + XW;
	result[2][2] = 1 - XX - YY;
	return result;
}

BEPUik::Matrix3x3 BEPUik::matrix::MultiplyTransposed(const Matrix3x3& transpose, const Matrix3x3& matrix)
{
	float resultM11 = transpose[0][0] * matrix[0][0] + transpose[1][0] * matrix[1][0] + transpose[2][0] * matrix[2][0];
	float resultM12 = transpose[0][0] * matrix[0][1] + transpose[1][0] * matrix[1][1] + transpose[2][0] * matrix[2][1];
	float resultM13 = transpose[0][0] * matrix[0][2] + transpose[1][0] * matrix[1][2] + transpose[2][0] * matrix[2][2];

	float resultM21 = transpose[0][1] * matrix[0][0] + transpose[1][1] * matrix[1][0] + transpose[2][1] * matrix[2][0];
	float resultM22 = transpose[0][1] * matrix[0][1] + transpose[1][1] * matrix[1][1] + transpose[2][1] * matrix[2][1];
	float resultM23 = transpose[0][1] * matrix[0][2] + transpose[1][1] * matrix[1][2] + transpose[2][1] * matrix[2][2];

	float resultM31 = transpose[0][2] * matrix[0][0] + transpose[1][2] * matrix[1][0] + transpose[2][2] * matrix[2][0];
	float resultM32 = transpose[0][2] * matrix[0][1] + transpose[1][2] * matrix[1][1] + transpose[2][2] * matrix[2][1];
	float resultM33 = transpose[0][2] * matrix[0][2] + transpose[1][2] * matrix[1][2] + transpose[2][2] * matrix[2][2];

	Matrix3x3 result;
	result[0][0] = resultM11;
	result[0][1] = resultM12;
	result[0][2] = resultM13;

	result[1][0] = resultM21;
	result[1][1] = resultM22;
	result[1][2] = resultM23;

	result[2][0] = resultM31;
	result[2][1] = resultM32;
	result[2][2] = resultM33;

	return result;
}

float BEPUik::matrix::Determinant(const Matrix3x3& matrix)
{
	return matrix[0][0] * matrix[1][1] * matrix[2][2] + matrix[0][1] * matrix[1][2] * matrix[2][0] + matrix[0][2] * matrix[1][0] * matrix[2][1] -
		matrix[2][0] * matrix[1][1] * matrix[0][2] - matrix[2][1] * matrix[1][2] * matrix[0][0] - matrix[2][2] * matrix[1][0] * matrix[0][1];
}

float BEPUik::matrix::AdaptiveDeterminant(const Matrix3x3& matrix, int& subMatrixCode)
{
	//Try the full matrix first.
	float determinant = matrix[0][0] * matrix[1][1] * matrix[2][2] + matrix[0][1] * matrix[1][2] * matrix[2][0] + matrix[0][2] * matrix[1][0] * matrix[2][1] -
		matrix[2][0] * matrix[1][1] * matrix[0][2] - matrix[2][1] * matrix[1][2] * matrix[0][0] - matrix[2][2] * matrix[1][0] * matrix[0][1];
	if (determinant != 0) //This could be a little numerically flimsy.  Fortunately, the way this method is used, that doesn't matter!
	{
		subMatrixCode = 0;
		return determinant;
	}
	//Try m11, m12, m21, m22.
	determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
	if (determinant != 0)
	{
		subMatrixCode = 1;
		return determinant;
	}
	//Try m22, m23, m32, m33.
	determinant = matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1];
	if (determinant != 0)
	{
		subMatrixCode = 2;
		return determinant;
	}
	//Try m11, m13, m31, m33.
	determinant = matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[0][1];
	if (determinant != 0)
	{
		subMatrixCode = 3;
		return determinant;
	}
	//Try m11.
	if (matrix[0][0] != 0)
	{
		subMatrixCode = 4;
		return matrix[0][0];
	}
	//Try m22.
	if (matrix[1][1] != 0)
	{
		subMatrixCode = 5;
		return matrix[1][1];
	}
	//Try m33.
	if (matrix[2][2] != 0)
	{
		subMatrixCode = 6;
		return matrix[2][2];
	}
	//It's completely singular!
	subMatrixCode = -1;
	return 0;
}

///////////

BEPUik::Vector3 BEPUik::vector3::Create()
{
	return Vector3{ 0.f,0.f,0.f };
}

BEPUik::Vector3 BEPUik::vector3::Add(const Vector3& a, const Vector3& b)
{
	return a + b;
}

BEPUik::Vector3 BEPUik::vector3::Subtract(const Vector3& a, const Vector3& b)
{
	return a - b;
}

BEPUik::Vector3 BEPUik::vector3::Multiply(const Vector3& v, float scale)
{
	return v * scale;
}

BEPUik::Vector3 BEPUik::vector3::Negate(const Vector3& v)
{
	return -v;
}

BEPUik::Vector3 BEPUik::vector3::Cross(const Vector3& a, const Vector3& b)
{
	return glm::cross(a, b);
}

float BEPUik::vector3::Dot(const Vector3& a, const Vector3& b)
{
	return glm::dot(a, b);
}

float BEPUik::vector3::Distance(const Vector3& a, const Vector3& b)
{
	return glm::distance(a, b);
}

BEPUik::Vector3 BEPUik::vector3::Divide(const Vector3& v, float divisor)
{
	Vector3 result;
	float inverse = 1 / divisor;
	result.x = v.x * inverse;
	result.y = v.y * inverse;
	result.z = v.z * inverse;
	return result;
}

void BEPUik::vector3::Normalize(Vector3& v)
{
	float inverse = (float)(1 / sqrt(v.x * v.x + v.y * v.y + v.z * v.z));
	auto& result = v;
	result.x = v.x * inverse;
	result.y = v.y * inverse;
	result.z = v.z * inverse;
}

BEPUik::Vector3 BEPUik::vector3::Max(const Vector3& a, const Vector3& b)
{
	Vector3 result;
	result.x = a.x > b.x ? a.x : b.x;
	result.y = a.y > b.y ? a.y : b.y;
	result.z = a.z > b.z ? a.z : b.z;
	return result;
}

float BEPUik::vector3::Length(const Vector3& v) { return glm::length(v); }
float BEPUik::vector3::LengthSqr(const Vector3& v) { return glm::length2(v); }

///////////

BEPUik::Quaternion BEPUik::quaternion::Create(float x, float y, float z, float w)
{
	return Quaternion{ w,x,y,z };
}

BEPUik::Vector3 BEPUik::quaternion::Transform(const Vector3& v, const Quaternion& rotation)
{
	//This operation is an optimized-down version of v' = q * v * q^-1.
	//The expanded form would be to treat v as an 'axis only' quaternion
	//and perform standard quaternion multiplication.  Assuming q is normalized,
	//q^-1 can be replaced by a conjugation.
	float x2 = rotation.x + rotation.x;
	float y2 = rotation.y + rotation.y;
	float z2 = rotation.z + rotation.z;
	float xx2 = rotation.x * x2;
	float xy2 = rotation.x * y2;
	float xz2 = rotation.x * z2;
	float yy2 = rotation.y * y2;
	float yz2 = rotation.y * z2;
	float zz2 = rotation.z * z2;
	float wx2 = rotation.w * x2;
	float wy2 = rotation.w * y2;
	float wz2 = rotation.w * z2;
	//Defer the component setting since they're used in computation.
	float transformedX = v.x * ((1.0f - yy2) - zz2) + v.y * (xy2 - wz2) + v.z * (xz2 + wy2);
	float transformedY = v.x * (xy2 + wz2) + v.y * ((1.0f - xx2) - zz2) + v.z * (yz2 - wx2);
	float transformedZ = v.x * (xz2 - wy2) + v.y * (yz2 + wx2) + v.z * ((1.0f - xx2) - yy2);
	Vector3 result;
	result.x = transformedX;
	result.y = transformedY;
	result.z = transformedZ;
	return result;
}

BEPUik::Quaternion BEPUik::quaternion::Conjugate(const Quaternion& quaternion)
{
	Quaternion result;
	result.x = -quaternion.x;
	result.y = -quaternion.y;
	result.z = -quaternion.z;
	result.w = quaternion.w;
	return result;
}

BEPUik::Quaternion BEPUik::quaternion::Multiply(const Quaternion& a, const Quaternion& b)
{
	float x = a.x;
	float y = a.y;
	float z = a.z;
	float w = a.w;
	float bX = b.x;
	float bY = b.y;
	float bZ = b.z;
	float bW = b.w;
	Quaternion result;
	result.x = x * bW + bX * w + y * bZ - z * bY;
	result.y = y * bW + bY * w + z * bX - x * bZ;
	result.z = z * bW + bZ * w + x * bY - y * bX;
	result.w = w * bW - x * bX - y * bY - z * bZ;
	return result;
}

void BEPUik::quaternion::GetAxisAngleFromQuaternion(const Quaternion& q, Vector3& axis, float& angle)
{
	float qx = q.x;
	float qy = q.y;
	float qz = q.z;
	float qw = q.w;
	if (qw < 0)
	{
		qx = -qx;
		qy = -qy;
		qz = -qz;
		qw = -qw;
	}
	if (qw > 1 - 1e-12)
	{
		axis = vector3::Up;
		angle = 0;
	}
	else
	{
		angle = 2 * acos(qw);
		float denominator = 1 / sqrt(1 - qw * qw);
		axis.x = qx * denominator;
		axis.y = qy * denominator;
		axis.z = qz * denominator;
	}
}

BEPUik::Quaternion BEPUik::quaternion::Concatenate(const Quaternion& a, const Quaternion& b)
{
	float aX = a.x;
	float aY = a.y;
	float aZ = a.z;
	float aW = a.w;
	float x = b.x;
	float y = b.y;
	float z = b.z;
	float w = b.w;
	Quaternion result;
	result.x = x * aW + aX * w + y * aZ - z * aY;
	result.y = y * aW + aY * w + z * aX - x * aZ;
	result.z = z * aW + aZ * w + x * aY - y * aX;
	result.w = w * aW - x * aX - y * aY - z * aZ;
	return result;
}

BEPUik::Quaternion BEPUik::quaternion::GetQuaternionBetweenNormalizedVectors(const Vector3& v1, const Vector3& v2)
{
	float dot;
	dot = vector3::Dot(v1, v2);
	Vector3 axis;
	axis = vector3::Cross(v1, v2);
	//For non-normal vectors, the multiplying the axes length squared would be necessary:
	//float w = dot + (float)Math.Sqrt(v1.LengthSquared() * v2.LengthSquared());
	Quaternion q;
	if (dot < -0.9999f) //parallel, opposing direction
		q = Quaternion(0.f, -v1.z, v1.y, v1.x);
	else
		q = Quaternion(dot + 1, axis.x, axis.y, axis.z);
	Normalize(q);
	return q;
}

void BEPUik::quaternion::Normalize(Quaternion& q)
{
	float inverse = (float)(1 / std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w));
	q.x *= inverse;
	q.y *= inverse;
	q.z *= inverse;
	q.w *= inverse;
}

BEPUik::Quaternion BEPUik::quaternion::Add(const Quaternion& a, const Quaternion& b)
{
	Quaternion result;
	result.x = a.x + b.x;
	result.y = a.y + b.y;
	result.z = a.z + b.z;
	result.w = a.w + b.w;
	return result;
}
