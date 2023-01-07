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

#include "bepuik/limit/IKLinearAxisLimit.hpp"

BEPUik::Vector3 BEPUik::IKLinearAxisLimit::GetLineAnchor() const { return m_connectionA->Position + quaternion::Transform(LocalLineAnchor, m_connectionA->Orientation); }
void BEPUik::IKLinearAxisLimit::SetLineAnchor(const Vector3 &value) { LocalLineAnchor = quaternion::Transform(value - m_connectionA->Position, BEPUik::quaternion::Conjugate(m_connectionA->Orientation)); }

BEPUik::Vector3 BEPUik::IKLinearAxisLimit::GetLineDirection() const { return quaternion::Transform(LocalLineDirection, m_connectionA->Orientation); }
void BEPUik::IKLinearAxisLimit::SetLineDirection(const Vector3 &value) { LocalLineDirection = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionA->Orientation)); }

BEPUik::Vector3 BEPUik::IKLinearAxisLimit::GetAnchorB() const { return m_connectionB->Position + quaternion::Transform(LocalAnchorB, m_connectionB->Orientation); }
void BEPUik::IKLinearAxisLimit::SetAnchorB(const Vector3 &value) { LocalAnchorB = quaternion::Transform(value - m_connectionB->Position, BEPUik::quaternion::Conjugate(m_connectionB->Orientation)); }

float BEPUik::IKLinearAxisLimit::GetMinimumDistance() const { return minimumDistance; }
void BEPUik::IKLinearAxisLimit::SetMinimumDistance(float value) { minimumDistance = value; }

float BEPUik::IKLinearAxisLimit::GetMaximumDistance() const { return maximumDistance; }
void BEPUik::IKLinearAxisLimit::SetMaximumDistance(float value) { maximumDistance = value; }

BEPUik::IKLinearAxisLimit::IKLinearAxisLimit(Bone connectionA, Bone connectionB, Vector3 lineAnchor, Vector3 lineDirection, Vector3 anchorB, float minimumDistance, float maximumDistance)
    : IKLimit(connectionA, connectionB)
{
    SetLineAnchor(lineAnchor);
    SetLineDirection(lineDirection);
    SetAnchorB(anchorB);
    SetMinimumDistance(minimumDistance);
    SetMaximumDistance(maximumDistance);
}

void BEPUik::IKLinearAxisLimit::UpdateJacobiansAndVelocityBias()
{
    //Transform the anchors and offsets into world space.
    Vector3 offsetA, offsetB, lineDirection;
    offsetA = quaternion::Transform(LocalLineAnchor, m_connectionA->Orientation);
    lineDirection = quaternion::Transform(LocalLineDirection, m_connectionA->Orientation);
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

    //Compute jacobians
    if (currentDistance > maximumDistance)
    {
        //We are exceeding the maximum limit.
        velocityBias = Vector3(errorCorrectionFactor * (currentDistance - maximumDistance), 0, 0);
    }
    else if (currentDistance < minimumDistance)
    {
        //We are exceeding the minimum limit.
        velocityBias = Vector3(errorCorrectionFactor * (minimumDistance - currentDistance), 0, 0);
        //The limit can only push in one direction. Flip the jacobian!
        lineDirection = vector3::Negate(lineDirection);
    }
    else if (currentDistance - minimumDistance > (maximumDistance - minimumDistance) * 0.5f)
    {
        //The objects are closer to hitting the maximum limit.
        velocityBias = Vector3(currentDistance - maximumDistance, 0, 0);
    }
    else
    {
        //The objects are closer to hitting the minimum limit.
        velocityBias = Vector3(minimumDistance - currentDistance, 0, 0);
        //The limit can only push in one direction. Flip the jacobian!
        lineDirection = vector3::Negate(lineDirection);
    }

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
