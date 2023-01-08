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

#include "bepuik/joint/IKPointOnLineJoint.hpp"

const BEPUik::Vector3 &BEPUik::IKPointOnLineJoint::GetLocalLineDirection() const { return localLineDirection; }
void BEPUik::IKPointOnLineJoint::SetLocalLineDirection(const Vector3 &value)
{
	localLineDirection = value;
	ComputeRestrictedAxes();
}

BEPUik::Vector3 BEPUik::IKPointOnLineJoint::GetLineAnchor() {
	return vector3::Add(m_connectionA->Position, quaternion::Transform(LocalLineAnchor, m_connectionA->Orientation));
}
void BEPUik::IKPointOnLineJoint::SetLineAnchor(const Vector3 &value) {
	LocalLineAnchor = quaternion::Transform(vector3::Subtract(value, m_connectionA->Position), BEPUik::quaternion::Conjugate(m_connectionA->Orientation));
}

BEPUik::Vector3 BEPUik::IKPointOnLineJoint::GetLineDirection() const {
	return quaternion::Transform(localLineDirection, m_connectionA->Orientation);
}
void BEPUik::IKPointOnLineJoint::SetLineDirection(const Vector3 &value) {
	localLineDirection = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionA->Orientation));
}

BEPUik::Vector3 BEPUik::IKPointOnLineJoint::GetAnchorB() {
	return vector3::Add(m_connectionB->Position, quaternion::Transform(LocalAnchorB, m_connectionB->Orientation));
}
void BEPUik::IKPointOnLineJoint::SetAnchorB(const Vector3 &value) {
	LocalAnchorB = quaternion::Transform(vector3::Subtract(value, m_connectionB->Position), BEPUik::quaternion::Conjugate(m_connectionB->Orientation));
}

void BEPUik::IKPointOnLineJoint::ComputeRestrictedAxes()
{
	Vector3 cross;
	cross = vector3::Cross(localLineDirection, vector3::Up);
	float lengthSquared = vector3::LengthSqr(cross);
	if (lengthSquared > Epsilon)
	{
		localRestrictedAxis1 = vector3::Divide(cross, (float)std::sqrt(lengthSquared));
	}
	else
	{
		//Oops! The direction is aligned with the up vector.
		cross = vector3::Cross(localLineDirection, vector3::Right);
		localRestrictedAxis1 = cross;
		vector3::Normalize(localRestrictedAxis1);
	}
	//Don't need to normalize this; cross product of two unit length perpendicular vectors.
	localRestrictedAxis2 = vector3::Cross(localRestrictedAxis1, localLineDirection);
}

BEPUik::IKPointOnLineJoint::IKPointOnLineJoint(Bone &connectionA, Bone &connectionB, const Vector3 &lineAnchor, const Vector3 &lineDirection, const Vector3 &anchorB)
: IKJoint(connectionA, connectionB)
{
	SetLineAnchor(lineAnchor);
	SetLineDirection(lineDirection);
	SetAnchorB(anchorB);

}

void BEPUik::IKPointOnLineJoint::UpdateJacobiansAndVelocityBias()
{

	//Transform local stuff into world space
	Vector3 worldRestrictedAxis1, worldRestrictedAxis2;
	worldRestrictedAxis1 = quaternion::Transform(localRestrictedAxis1, m_connectionA->Orientation);
	worldRestrictedAxis2 = quaternion::Transform(localRestrictedAxis2, m_connectionA->Orientation);

	Vector3 worldLineAnchor;
	worldLineAnchor = quaternion::Transform(LocalLineAnchor, m_connectionA->Orientation);
	worldLineAnchor = vector3::Add(worldLineAnchor, m_connectionA->Position);
	Vector3 lineDirection;
	lineDirection = quaternion::Transform(localLineDirection, m_connectionA->Orientation);

	Vector3 rB;
	rB = quaternion::Transform(LocalAnchorB, m_connectionB->Orientation);
	Vector3 worldPoint;
	worldPoint = vector3::Add(rB, m_connectionB->Position);

	//Find the point on the line closest to the world point.
	Vector3 offset;
	offset = vector3::Subtract(worldPoint, worldLineAnchor);
	float distanceAlongAxis;
	distanceAlongAxis = vector3::Dot(offset, lineDirection);

	Vector3 worldNearPoint;
	offset = vector3::Multiply(lineDirection, distanceAlongAxis);
	worldNearPoint = vector3::Add(worldLineAnchor, offset);
	Vector3 rA;
	rA = vector3::Subtract(worldNearPoint, m_connectionA->Position);

	//Error
	Vector3 error3D;
	error3D = vector3::Subtract(worldPoint, worldNearPoint);

	Vector2 error;
	error.x = vector3::Dot(error3D, worldRestrictedAxis1);
	error.y = vector3::Dot(error3D, worldRestrictedAxis2);

	velocityBias.x = errorCorrectionFactor * error.x;
	velocityBias.y = errorCorrectionFactor * error.y;


	//Set up the jacobians
	Vector3 angularA1, angularA2, angularB1, angularB2;
	angularA1 = vector3::Cross(rA, worldRestrictedAxis1);
	angularA2 = vector3::Cross(rA, worldRestrictedAxis2);
	angularB1 = vector3::Cross(worldRestrictedAxis1, rB);
	angularB2 = vector3::Cross(worldRestrictedAxis2, rB);

	//Put all the 1x3 jacobians into a 3x3 matrix representation.
	linearJacobianA = matrix::Create();
	linearJacobianA[0][0] = worldRestrictedAxis1.x;
	linearJacobianA[0][1] = worldRestrictedAxis1.y;
	linearJacobianA[0][2] = worldRestrictedAxis1.z;
	linearJacobianA[1][0] = worldRestrictedAxis2.x;
	linearJacobianA[1][1] = worldRestrictedAxis2.y;
	linearJacobianA[1][2] = worldRestrictedAxis2.z;
	linearJacobianB = matrix::Negate(linearJacobianA);

	angularJacobianA = matrix::Create();
	angularJacobianA[0][0] = angularA1.x;
	angularJacobianA[0][1] = angularA1.y;
	angularJacobianA[0][2] = angularA1.z;
	angularJacobianA[1][0] = angularA2.x;
	angularJacobianA[1][1] = angularA2.y;
	angularJacobianA[1][2] = angularA2.z;

	angularJacobianB = matrix::Create();
	angularJacobianB[0][0] = angularB1.x;
	angularJacobianB[0][1] = angularB1.y;
	angularJacobianB[0][2] = angularB1.z;
	angularJacobianB[1][0] = angularB2.x;
	angularJacobianB[1][1] = angularB2.y;
	angularJacobianB[1][2] = angularB2.z;
}