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

#include "bepuik/limit/IKSwingLimit.hpp"

BEPUik::Vector3 BEPUik::IKSwingLimit::GetAxisA() const { return quaternion::Transform(LocalAxisA, m_connectionA->Orientation); }
void BEPUik::IKSwingLimit::SetAxisA(const Vector3 &value) { LocalAxisA = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionA->Orientation)); }

BEPUik::Vector3 BEPUik::IKSwingLimit::GetAxisB() const { return quaternion::Transform(LocalAxisB, m_connectionB->Orientation); }
void BEPUik::IKSwingLimit::SetAxisB(const Vector3 &value) { LocalAxisB = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionB->Orientation)); }

float BEPUik::IKSwingLimit::GetMaximumAngle() const { return maximumAngle; }
void BEPUik::IKSwingLimit::SetMaximumAngle(float value) { maximumAngle = std::max(0.f, value); }

BEPUik::IKSwingLimit::IKSwingLimit(Bone &connectionA, Bone &connectionB, const Vector3 &axisA, const Vector3 &axisB, float maximumAngle)
    : IKLimit(connectionA, connectionB)
{
    SetAxisA(axisA);
    SetAxisB(axisB);
    SetMaximumAngle(maximumAngle);
}

void BEPUik::IKSwingLimit::UpdateJacobiansAndVelocityBias()
{

    //This constraint doesn't consider linear motion.
    linearJacobianA = matrix::Create();
	linearJacobianB = matrix::Create();

    //Compute the world axes.
    Vector3 axisA, axisB;
    axisA = quaternion::Transform(LocalAxisA, m_connectionA->Orientation);
    axisB = quaternion::Transform(LocalAxisB, m_connectionB->Orientation);

    float dot;
    dot = vector3::Dot(axisA, axisB);

    //Yes, we could avoid this acos here. Performance is not the highest goal of this system; the less tricks used, the easier it is to understand.
    float angle = (float)std::acos(std::clamp(dot, -1.f, 1.f));

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
    if (angle >= maximumAngle)
    {
        velocityBias = Vector3(errorCorrectionFactor * (angle - maximumAngle), 0, 0);
    }
    else
    {
        //The constraint is not yet violated. But, it may be- allow only as much motion as could occur withviolating the constraint.
        //Limits can't 'pull,' so this will not result in erroneous sticking.
        velocityBias = Vector3(angle - maximumAngle, 0, 0);
    }


}
