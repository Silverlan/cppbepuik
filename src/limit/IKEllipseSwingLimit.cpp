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

#include "bepuik/limit/IKEllipseSwingLimit.hpp"

BEPUik::Vector3 BEPUik::IKEllipseSwingLimit::GetAxisA() const { return quaternion::Transform(LocalAxisA, m_connectionA->Orientation); }
void BEPUik::IKEllipseSwingLimit::SetAxisA(const Vector3& value) { LocalAxisA = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionA->Orientation)); }

BEPUik::Vector3 BEPUik::IKEllipseSwingLimit::GetAxisB() const { return quaternion::Transform(LocalAxisB, m_connectionB->Orientation); }
void BEPUik::IKEllipseSwingLimit::SetAxisB(const Vector3& value)
{
	LocalAxisB = quaternion::Transform(value, quaternion::Conjugate(m_connectionB->Orientation));
	LocalAxisBRelToA = quaternion::Transform(value, quaternion::Conjugate(m_connectionB->Orientation));
}

float BEPUik::IKEllipseSwingLimit::GetMaximumAngleX()
{
	return maximumAngleX;
}

void BEPUik::IKEllipseSwingLimit::SetMaximumAngleX(float angle)
{
	maximumAngleX = std::max(0.f, angle);
}

float BEPUik::IKEllipseSwingLimit::GetMaximumAngleY()
{
	return maximumAngleY;
}

void BEPUik::IKEllipseSwingLimit::SetMaximumAngleY(float angle)
{
	maximumAngleY = std::max(0.f, angle);
}

BEPUik::Vector3 BEPUik::IKEllipseSwingLimit::GetXAxis() const
{
	Vector3 axis;
	axis = quaternion::Transform(LocalXAxis, m_connectionB->Orientation);
	return axis;
}

void BEPUik::IKEllipseSwingLimit::SetXAxis(const Vector3& axis)
{
	Quaternion conjugate;
	conjugate = quaternion::Conjugate(m_connectionA->Orientation);
	LocalXAxis = quaternion::Transform(axis, conjugate);
}

BEPUik::IKEllipseSwingLimit::IKEllipseSwingLimit(Bone& connectionA, Bone& connectionB, const Vector3& axisA, const Vector3& axisB, float maximumAngleX, float maximumAngleY)
	: IKLimit(connectionA, connectionB)
{
	SetAxisA(axisA);
	SetAxisB(axisB);
	SetMaximumAngleX(maximumAngleX);
	SetMaximumAngleY(maximumAngleY);
	SetupJointTransforms(axisB);
}

void BEPUik::IKEllipseSwingLimit::SetupJointTransforms(const Vector3& twistAxis)
{
	//Compute a vector which is perpendicular to the axis.  It'll be added in local space to both connections.
	Vector3 UpVector = vector3::Up;
	Vector3 RightVector = vector3::Right;
	xAxis = vector3::Cross(twistAxis, UpVector);
	float length = vector3::LengthSqr(xAxis);
	if (length < Epsilon)
		xAxis = vector3::Cross(twistAxis, RightVector);

	yAxis = vector3::Cross(twistAxis, xAxis);

	//Put the axes into the joint transform of A.
	vector3::Normalize(xAxis);
	vector3::Normalize(yAxis);
	xAxis = glm::rotate(glm::inverse(m_connectionA->Orientation), xAxis);
	yAxis = glm::rotate(glm::inverse(m_connectionA->Orientation), yAxis);
}

void BEPUik::IKEllipseSwingLimit::UpdateJacobiansAndVelocityBias()
{
	//This constraint doesn't consider linear motion.
	linearJacobianA = matrix::Create();
	linearJacobianB = matrix::Create();

	//Compute the world axes.
	Vector3 axisA, axisB;
	axisA = quaternion::Transform(LocalAxisA, m_connectionA->Orientation);
	axisB = quaternion::Transform(LocalAxisB, m_connectionB->Orientation);

	float angleX;
	float angleY;
	float error;
	auto worldTwistAxisB = axisB;
	auto primaryAxis = axisA;
	auto relativeRotation = quaternion::GetQuaternionBetweenNormalizedVectors(worldTwistAxisB, primaryAxis);
	Vector3 axis;
	float angle;
	quaternion::GetAxisAngleFromQuaternion(relativeRotation, axis, angle);

	Vector3 axisAngle;
	//This combined axis-angle representation is similar to angular velocity in describing a rotation.
	//Just like you can dot an axis with angular velocity to get a velocity around that axis,
	//dotting an axis with the axis-angle representation gets the angle of rotation around that axis.
	//(As far as the constraint is concerned, anyway.)
	axisAngle.x = axis.x * angle;
	axisAngle.y = axis.y * angle;
	axisAngle.z = axis.z * angle;

	auto basisXAxis = xAxis;
	basisXAxis = glm::rotate(m_connectionA->Orientation, basisXAxis);

	auto basisYAxis = yAxis;
	basisYAxis = glm::rotate(m_connectionA->Orientation, basisYAxis);
	//basis.rotationMatrix = connectionA.orientationMatrix;

	//Compute the individual swing angles.
	angleX = vector3::Dot(axisAngle, basisXAxis);
	angleY = vector3::Dot(axisAngle, basisYAxis);

	//The position constraint states that the angles must be within an ellipse. The following is just a reorganization of the x^2 / a^2 + y^2 / b^2 <= 1 definition of an ellipse's area.
	float maxAngleXSquared = maximumAngleX * maximumAngleX;
	float maxAngleYSquared = maximumAngleY * maximumAngleY;
	error = angleX * angleX * maxAngleYSquared + angleY * angleY * maxAngleXSquared - maxAngleXSquared * maxAngleYSquared;

	//One angular DOF is constrained by this limit.
	Vector3 hingeAxis;
	hingeAxis = vector3::Cross(axisA, axisB);

	angularJacobianA = matrix::Create();
	angularJacobianA[0][0] = hingeAxis.x;
	angularJacobianA[0][1] = hingeAxis.y;
	angularJacobianA[0][2] = hingeAxis.z;
	angularJacobianB = matrix::Create();
	angularJacobianB[0][0] = -hingeAxis.x;
	angularJacobianB[0][1] = -hingeAxis.y;
	angularJacobianB[0][2] = -hingeAxis.z;

	//Note how we've computed the jacobians despite the limit being potentially inactive.
	//This is to enable 'speculative' limits.
	if (error >= 0.f) {
		velocityBias = Vector3(errorCorrectionFactor *error,0.f,0.f);
	}
	else {
		//The constraint is not yet violated. But, it may be- allow only as much motion as could occur withviolating the constraint.
		//Limits can't 'pull,' so this will not result in erroneous sticking.
		velocityBias = Vector3(error,0.f,0.f);
	}
}
