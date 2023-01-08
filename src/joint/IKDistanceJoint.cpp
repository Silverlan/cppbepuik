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

#include "bepuik/joint/IKDistanceJoint.hpp"

BEPUik::Vector3 BEPUik::IKDistanceJoint::GetAnchorA() {
	return vector3::Add(m_connectionA->Position, quaternion::Transform(LocalAnchorA, m_connectionA->Orientation));
}
void BEPUik::IKDistanceJoint::SetAnchorA(const Vector3 &value) {
	LocalAnchorA = quaternion::Transform(vector3::Subtract(value, m_connectionA->Position), BEPUik::quaternion::Conjugate(m_connectionA->Orientation));
}

BEPUik::Vector3 BEPUik::IKDistanceJoint::GetAnchorB() {
	return vector3::Add(m_connectionB->Position, quaternion::Transform(LocalAnchorB, m_connectionB->Orientation));
}
void BEPUik::IKDistanceJoint::SetAnchorB(const Vector3 &value) {
	LocalAnchorB = quaternion::Transform(vector3::Subtract(value, m_connectionB->Position), BEPUik::quaternion::Conjugate(m_connectionB->Orientation));
}

float BEPUik::IKDistanceJoint::GetDistance() const { return distance; }
void BEPUik::IKDistanceJoint::SetDistance(float value) { distance = std::max(0.f, value); }

BEPUik::IKDistanceJoint::IKDistanceJoint(Bone &connectionA, Bone &connectionB, const Vector3 &anchorA, const Vector3 &anchorB)
    : IKJoint(connectionA, connectionB)
{
	SetAnchorA(anchorA);
	SetAnchorB(anchorB);
    SetDistance(vector3::Distance(anchorA, anchorB));
}

void BEPUik::IKDistanceJoint::UpdateJacobiansAndVelocityBias()
{
    //Transform the anchors and offsets into world space.
    Vector3 offsetA = quaternion::Transform(LocalAnchorA, m_connectionA->Orientation);
    Vector3 offsetB = quaternion::Transform(LocalAnchorB, m_connectionB->Orientation);
    Vector3 anchorA = vector3::Add(m_connectionA->Position, offsetA);
    Vector3 anchorB = vector3::Add(m_connectionB->Position, offsetB);

    //Compute the distance.
    Vector3 separation = vector3::Subtract(anchorB, anchorA);
    float currentDistance = vector3::Length(separation);

    //Compute jacobians
	Vector3 linearA = vector3::Create();
    if (currentDistance > Epsilon)
    {
        linearA.x = separation.x / currentDistance;
        linearA.y = separation.y / currentDistance;
        linearA.z = separation.z / currentDistance;

        velocityBias = Vector3(errorCorrectionFactor * (currentDistance - distance), 0, 0);
    }
    else
    {
        velocityBias = vector3::Create();
        linearA = vector3::Create();
    }

    Vector3 angularA = vector3::Cross(offsetA, linearA);
    //linearB = -linearA, so just swap the cross product order.
    Vector3 angularB = vector3::Cross(linearA, offsetB);

    //Put all the 1x3 jacobians into a 3x3 matrix representation.
	linearJacobianA = matrix::Create();
	linearJacobianA[0][0] = linearA.x;
	linearJacobianA[0][1] = linearA.y;
	linearJacobianA[0][2] = linearA.z;
	linearJacobianB = matrix::Create();
	linearJacobianB[0][0] = -linearA.x;
	linearJacobianB[0][1] = -linearA.y;
	linearJacobianB[0][2] = -linearA.z;
	angularJacobianA = matrix::Create();
	angularJacobianA[0][0] = angularA.x;
	angularJacobianA[0][1] = angularA.y;
	angularJacobianA[0][2] = angularA.z;
	angularJacobianB = matrix::Create();
	angularJacobianB[0][0] = angularB.x;
	angularJacobianB[0][1] = angularB.y;
	angularJacobianB[0][2] = angularB.z;

}
