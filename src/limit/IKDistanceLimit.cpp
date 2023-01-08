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

#include "bepuik/limit/IKDistanceLimit.hpp"
#include <algorithm>

BEPUik::Vector3 BEPUik::IKDistanceLimit::GetAnchorA() const { return vector3::Add(m_connectionA->Position, quaternion::Transform(LocalAnchorA, m_connectionA->Orientation)); }
void BEPUik::IKDistanceLimit::SetAnchorA(const Vector3 &value) { LocalAnchorA = quaternion::Transform(vector3::Subtract(value, m_connectionA->Position), BEPUik::quaternion::Conjugate(m_connectionA->Orientation)); }

BEPUik::Vector3 BEPUik::IKDistanceLimit::GetAnchorB() const { return vector3::Add(m_connectionB->Position, quaternion::Transform(LocalAnchorB, m_connectionB->Orientation)); }
void BEPUik::IKDistanceLimit::SetAnchorB(const Vector3 &value) { LocalAnchorB = quaternion::Transform(vector3::Subtract(value, m_connectionB->Position), BEPUik::quaternion::Conjugate(m_connectionB->Orientation)); }

float BEPUik::IKDistanceLimit::GetMinimumDistance() const { return MinimumDistance; }
void BEPUik::IKDistanceLimit::SetMinimumDistance(float value) { MinimumDistance = std::max(0.f, value); }

float BEPUik::IKDistanceLimit::GetMaximumDistance() { return MaximumDistance; }
void BEPUik::IKDistanceLimit::SetMaximumDistance(float value) { MaximumDistance = std::max(0.f, value); }

BEPUik::IKDistanceLimit::IKDistanceLimit(Bone &connectionA, Bone &connectionB, const Vector3 &anchorA, const Vector3 &anchorB, float minimumDistance, float maximumDistance)
    : IKLimit(connectionA, connectionB)
{
    SetAnchorA(anchorA);
    SetAnchorB(anchorB);
    MinimumDistance = minimumDistance;
    MaximumDistance = maximumDistance;
}

void BEPUik::IKDistanceLimit::UpdateJacobiansAndVelocityBias()
{
    //Transform the anchors and offsets into world space.
    Vector3 offsetA, offsetB;
    offsetA = quaternion::Transform(LocalAnchorA, m_connectionA->Orientation);
    offsetB = quaternion::Transform(LocalAnchorB, m_connectionB->Orientation);
    Vector3 anchorA, anchorB;
    anchorA = vector3::Add(m_connectionA->Position, offsetA);
    anchorB = vector3::Add(m_connectionB->Position, offsetB);

    //Compute the distance.
    Vector3 separation;
    separation = vector3::Subtract(anchorB, anchorA);
    float currentDistance = vector3::Length(separation);

    //Compute jacobians
    Vector3 linearA = vector3::Create();
    if (currentDistance > Epsilon)
    {
        linearA.x = separation.x / currentDistance;
        linearA.y = separation.y / currentDistance;
        linearA.z = separation.z / currentDistance;

        if (currentDistance > MaximumDistance)
        {
            //We are exceeding the maximum limit.
            velocityBias = Vector3(errorCorrectionFactor * (currentDistance - MaximumDistance), 0, 0);
        }
        else if (currentDistance < MinimumDistance)
        {
            //We are exceeding the minimum limit.
            velocityBias = Vector3(errorCorrectionFactor * (MinimumDistance - currentDistance), 0, 0);
            //The limit can only push in one direction. Flip the jacobian!
            linearA = vector3::Negate(linearA);
        }
        else if (currentDistance - MinimumDistance > (MaximumDistance - MinimumDistance) * 0.5f)
        {
            //The objects are closer to hitting the maximum limit.
            velocityBias = Vector3(currentDistance - MaximumDistance, 0, 0);
        }
        else
        {
            //The objects are closer to hitting the minimum limit.
            velocityBias = Vector3(MinimumDistance - currentDistance, 0, 0);
            //The limit can only push in one direction. Flip the jacobian!
            linearA = vector3::Negate(linearA);
        }
    }
    else
    {
        velocityBias = vector3::Create();
        linearA = vector3::Create();
    }

    Vector3 angularA, angularB;
    angularA = vector3::Cross(offsetA, linearA);
    //linearB = -linearA, so just swap the cross product order.
    angularB = vector3::Cross(linearA, offsetB);

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
