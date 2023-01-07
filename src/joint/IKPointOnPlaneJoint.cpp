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

#include "bepuik/joint/IKPointOnPlaneJoint.hpp"

BEPUik::Vector3 BEPUik::IKPointOnPlaneJoint::GetPlaneAnchor() const { return m_connectionA->Position + quaternion::Transform(LocalPlaneAnchor, m_connectionA->Orientation); }
void BEPUik::IKPointOnPlaneJoint::SetPlaneAnchor(const Vector3 &value) { LocalPlaneAnchor = quaternion::Transform(value - m_connectionA->Position, BEPUik::quaternion::Conjugate(m_connectionA->Orientation)); }

BEPUik::Vector3 BEPUik::IKPointOnPlaneJoint::GetPlaneNormal() const { return quaternion::Transform(LocalPlaneNormal, m_connectionA->Orientation); }
void BEPUik::IKPointOnPlaneJoint::SetPlaneNormal(const Vector3 &value) { LocalPlaneNormal = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionA->Orientation)); }

BEPUik::Vector3 BEPUik::IKPointOnPlaneJoint::GetAnchorB() const { return m_connectionB->Position + quaternion::Transform(LocalAnchorB, m_connectionB->Orientation); }
void BEPUik::IKPointOnPlaneJoint::SetAnchorB(const Vector3 &value) { LocalAnchorB = quaternion::Transform(value - m_connectionB->Position, BEPUik::quaternion::Conjugate(m_connectionB->Orientation)); }

BEPUik::IKPointOnPlaneJoint::IKPointOnPlaneJoint(Bone &connectionA, Bone &connectionB, const Vector3 &planeAnchor, const Vector3 &planeNormal, const Vector3 &anchorB)
    : IKJoint(connectionA, connectionB)
{
    SetPlaneAnchor(planeAnchor);
    SetPlaneNormal(planeNormal);
    SetAnchorB(anchorB);
}

void BEPUik::IKPointOnPlaneJoint::UpdateJacobiansAndVelocityBias()
{
    //Transform the anchors and offsets into world space.
    Vector3 offsetA, offsetB, lineDirection;
    offsetA = quaternion::Transform(LocalPlaneAnchor, m_connectionA->Orientation);
    lineDirection = quaternion::Transform(LocalPlaneNormal, m_connectionA->Orientation);
    offsetB = quaternion::Transform(LocalAnchorB, m_connectionB->Orientation);
    Vector3 anchorA, anchorB;
    anchorA = vector3::Add(m_connectionA->Position, offsetA);
    anchorB = vector3::Add(m_connectionB->Position, offsetB);

    //Compute the distance.
    Vector3 separation;
    separation = vector3::Subtract(anchorB, anchorA);
    //This entire constraint is very similar to the IKDistanceLimit, except the current distance is along an axis.
    float currentDistance;
    currentDistance = vector3::Dot(separation, lineDirection);
    velocityBias = Vector3(errorCorrectionFactor * currentDistance, 0, 0);

    //Compute jacobians
    Vector3 angularA, angularB;
    //We can't just use the offset to anchor for A's jacobian- the 'collision' location is way there at anchorB!
    Vector3 rA;
    rA = vector3::Subtract(anchorB, m_connectionA->Position);
    angularA = vector3::Cross(rA, lineDirection);
    //linearB = -linearA, so just swap the cross product order.
    angularB = vector3::Cross(lineDirection, offsetB);

    //Put all the 1x3 jacobians into a 3x3 matrix representation.
	linearJacobianA = matrix::Create();
	linearJacobianA[0][0] = lineDirection.x;
	linearJacobianA[0][1] = lineDirection.y;
	linearJacobianA[0][2] = lineDirection.z;
	linearJacobianB = matrix::Create();
	linearJacobianB[0][0] = -lineDirection.x;
	linearJacobianB[0][1] = -lineDirection.y;
	linearJacobianB[0][2] = -lineDirection.z;
	angularJacobianA = matrix::Create();
	angularJacobianA[0][0] = angularA.x;
	angularJacobianA[0][1] = angularA.y;
	angularJacobianA[0][2] = angularA.z;
	angularJacobianB = matrix::Create();
	angularJacobianB[0][0] = angularB.x;
	angularJacobianB[0][1] = angularB.y;
	angularJacobianB[0][2] = angularB.z;

}