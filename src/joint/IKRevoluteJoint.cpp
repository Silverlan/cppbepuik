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

#include "bepuik/joint/IKRevoluteJoint.hpp"

const BEPUik::Vector3 &BEPUik::IKRevoluteJoint::GetLocalFreeAxisA() const { return localFreeAxisA; }
void BEPUik::IKRevoluteJoint::SetLocalFreeAxisA(const Vector3 &value)
{
    localFreeAxisA = value;
    ComputeConstrainedAxes();
}

const BEPUik::Vector3 &BEPUik::IKRevoluteJoint::GetLocalFreeAxisB() const { return localFreeAxisB; }
void BEPUik::IKRevoluteJoint::SetLocalFreeAxisB(const Vector3 &value)
{
    localFreeAxisB = value;
    ComputeConstrainedAxes();
}

BEPUik::Vector3 BEPUik::IKRevoluteJoint::GetWorldFreeAxisA() const { return quaternion::Transform(localFreeAxisA, m_connectionA->Orientation); }
void BEPUik::IKRevoluteJoint::SetWorldFreeAxisA(const Vector3 &value)
{
    SetLocalFreeAxisA(quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionA->Orientation)));
}

BEPUik::Vector3 BEPUik::IKRevoluteJoint::GetWorldFreeAxisB() const { return quaternion::Transform(localFreeAxisB, m_connectionB->Orientation); }
void BEPUik::IKRevoluteJoint::SetWorldFreeAxisB(const Vector3 &value)
{
    SetLocalFreeAxisB(quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionB->Orientation)));
}

void BEPUik::IKRevoluteJoint::ComputeConstrainedAxes()
{
    Vector3 worldAxisA = GetWorldFreeAxisA();
    Vector3 error = vector3::Cross(worldAxisA, GetWorldFreeAxisB());
    float lengthSquared = vector3::LengthSqr(error);
    Vector3 worldConstrainedAxis1, worldConstrainedAxis2;
    //Find the first constrained axis.
    if (lengthSquared > Epsilon)
    {
        //The error direction can be used as the first axis!
        worldConstrainedAxis1 = vector3::Divide(error, (float)std::sqrt(lengthSquared));
    }
    else
    {
        //There's not enough error for it to be a good constrained axis.
        //We'll need to create the constrained axes arbitrarily.
        worldConstrainedAxis1 = vector3::Cross(vector3::Up, worldAxisA);
        lengthSquared = vector3::LengthSqr(worldConstrainedAxis1);
        if (lengthSquared > Epsilon)
        {
            //The up vector worked!
            worldConstrainedAxis1 = vector3::Divide(worldConstrainedAxis1, (float)std::sqrt(lengthSquared));
        }
        else
        {
            //The up vector didn't work. Just try the right vector.
            worldConstrainedAxis1 = vector3::Cross(vector3::Right, worldAxisA);
            vector3::Normalize(worldConstrainedAxis1);
        }
    }
    //Don't have to normalize the second constraint axis; it's the cross product of two perpendicular normalized vectors.
    worldConstrainedAxis2 = vector3::Cross(worldAxisA, worldConstrainedAxis1);

    localConstrainedAxis1 = quaternion::Transform(worldConstrainedAxis1, BEPUik::quaternion::Conjugate(m_connectionA->Orientation));
    localConstrainedAxis2 = quaternion::Transform(worldConstrainedAxis2, BEPUik::quaternion::Conjugate(m_connectionA->Orientation));
}

BEPUik::IKRevoluteJoint::IKRevoluteJoint(Bone &connectionA, Bone &connectionB, const Vector3 &freeAxis)
    : IKJoint(connectionA, connectionB)
{
    SetWorldFreeAxisA(freeAxis);
    SetWorldFreeAxisB(freeAxis);
}

void BEPUik::IKRevoluteJoint::UpdateJacobiansAndVelocityBias()
{
    linearJacobianA = matrix::Create();
	linearJacobianB = matrix::Create();

    //We know the one free axis. We need the two restricted axes. This amounts to completing the orthonormal basis.
    //We can grab one of the restricted axes using a cross product of the two world axes. This is not guaranteed
    //to be nonzero, so the normalization requires protection.

    Vector3 worldAxisA, worldAxisB;
    worldAxisA = quaternion::Transform(localFreeAxisA, m_connectionA->Orientation);
    worldAxisB = quaternion::Transform(localFreeAxisB, m_connectionB->Orientation);

    Vector3 error;
    error = vector3::Cross(worldAxisA, worldAxisB);

    Vector3 worldConstrainedAxis1, worldConstrainedAxis2;
    worldConstrainedAxis1 = quaternion::Transform(localConstrainedAxis1, m_connectionA->Orientation);
    worldConstrainedAxis2 = quaternion::Transform(localConstrainedAxis2, m_connectionA->Orientation);


    angularJacobianA = matrix::Create();
	angularJacobianA[0][0] = worldConstrainedAxis1.x;
	angularJacobianA[0][1] = worldConstrainedAxis1.y;
	angularJacobianA[0][2] = worldConstrainedAxis1.z;
	angularJacobianA[1][0] = worldConstrainedAxis2.x;
	angularJacobianA[1][1] = worldConstrainedAxis2.y;
	angularJacobianA[1][2] = worldConstrainedAxis2.z;

    angularJacobianB = matrix::Negate(angularJacobianA);


    Vector2 constraintSpaceError;
    constraintSpaceError.x = vector3::Dot(error, worldConstrainedAxis1);
    constraintSpaceError.y = vector3::Dot(error, worldConstrainedAxis2);
    velocityBias.x = errorCorrectionFactor * constraintSpaceError.x;
    velocityBias.y = errorCorrectionFactor * constraintSpaceError.y;


}
