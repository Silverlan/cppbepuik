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

#include "bepuik/joint/IKSwivelHingeJoint.hpp"

BEPUik::Vector3 BEPUik::IKSwivelHingeJoint::GetWorldHingeAxis() const { return quaternion::Transform(LocalHingeAxis, m_connectionA->Orientation); }
void BEPUik::IKSwivelHingeJoint::SetWorldHingeAxis(const Vector3 &value)
{
    LocalHingeAxis = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionA->Orientation));
}

BEPUik::Vector3 BEPUik::IKSwivelHingeJoint::GetWorldTwistAxis() const { return quaternion::Transform(LocalTwistAxis, m_connectionB->Orientation); }
void BEPUik::IKSwivelHingeJoint::SetWorldTwistAxis(const Vector3 &value)
{
    LocalTwistAxis = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionB->Orientation));
}

BEPUik::IKSwivelHingeJoint::IKSwivelHingeJoint(Bone &connectionA, Bone &connectionB, const Vector3 &worldHingeAxis, const Vector3 &worldTwistAxis)
    : IKJoint(connectionA, connectionB)
{
    SetWorldHingeAxis(worldHingeAxis);
    SetWorldTwistAxis(worldTwistAxis);
}

void BEPUik::IKSwivelHingeJoint::UpdateJacobiansAndVelocityBias()
{
    linearJacobianA = matrix::Create();
	linearJacobianB = matrix::Create();


    //There are two free axes and one restricted axis.
    //The constraint attempts to keep the hinge axis attached to connection A and the twist axis attached to connection B perpendicular to each other.
    //The restricted axis is the cross product between the twist and hinge axes.

    Vector3 worldTwistAxis, worldHingeAxis;
    worldHingeAxis = quaternion::Transform(LocalHingeAxis, m_connectionA->Orientation);
    worldTwistAxis = quaternion::Transform(LocalTwistAxis, m_connectionB->Orientation);

    Vector3 restrictedAxis;
    restrictedAxis = vector3::Cross(worldHingeAxis, worldTwistAxis);
    //Attempt to normalize the restricted axis.
    float lengthSquared = vector3::LengthSqr(restrictedAxis);
    if (lengthSquared > Epsilon)
    {
        restrictedAxis = vector3::Divide(restrictedAxis, (float)std::sqrt(lengthSquared));
    }
    else
    {
		restrictedAxis = vector3::Create();
    }


	angularJacobianA = matrix::Create();
	angularJacobianA[0][0] = restrictedAxis.x;
	angularJacobianA[0][1] = restrictedAxis.y;
	angularJacobianA[0][2] = restrictedAxis.z;
    angularJacobianB = matrix::Negate(angularJacobianA);

    float error;
    error = vector3::Dot(worldHingeAxis, worldTwistAxis);
    error = (float)std::acos(std::clamp(error, -1.f, 1.f)) - PiOver2;

    velocityBias = Vector3(errorCorrectionFactor * error, 0, 0);


}