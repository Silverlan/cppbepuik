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

#include "bepuik/joint/IKBallSocketJoint.hpp"

BEPUik::Vector3 BEPUik::IKBallSocketJoint::GetOffsetA() const {
	return quaternion::Transform(LocalOffsetA, m_connectionA->Orientation);
}
void BEPUik::IKBallSocketJoint::SetOffsetA(const Vector3 &value) {
	Quaternion conjugate = BEPUik::quaternion::Conjugate(m_connectionA->Orientation);
	LocalOffsetA = quaternion::Transform(value, conjugate);
}

BEPUik::Vector3 BEPUik::IKBallSocketJoint::GetOffsetB() const {
	BEPUik::Vector3 result = quaternion::Transform(LocalOffsetB, m_connectionB->Orientation);
	return result;
}
void BEPUik::IKBallSocketJoint::SetOffsetB(const Vector3 &value) {
	Quaternion conjugate = BEPUik::quaternion::Conjugate(m_connectionB->Orientation);
	LocalOffsetB = quaternion::Transform(value, conjugate);
}

BEPUik::IKBallSocketJoint::IKBallSocketJoint(Bone &connectionA, Bone &connectionB, const Vector3 &anchor)
    : IKJoint(connectionA, connectionB)
{
    SetOffsetA(vector3::Subtract(anchor, m_connectionA->Position));
    SetOffsetB(vector3::Subtract(anchor, m_connectionB->Position));
}

void BEPUik::IKBallSocketJoint::UpdateJacobiansAndVelocityBias()
{
	linearJacobianA = matrix::GetIdentity();
    //The jacobian entries are is [ La, Aa, -Lb, -Ab ] because the relative velocity is computed using A-B. So, negate B's jacobians!
	linearJacobianB = matrix::Create(-1.f);
    Vector3 rA = quaternion::Transform(LocalOffsetA, m_connectionA->Orientation);
    angularJacobianA = matrix::CreateCrossProduct(rA);
    //Transposing a skew-symmetric matrix is equivalent to negating it.
    angularJacobianA = matrix::Transpose(angularJacobianA);

    Vector3 worldPositionA = vector3::Add(m_connectionA->Position, rA);

    Vector3 rB = quaternion::Transform(LocalOffsetB, m_connectionB->Orientation);
    angularJacobianB = matrix::CreateCrossProduct(rB);

    Vector3 worldPositionB;
    worldPositionB = vector3::Add(m_connectionB->Position, rB);

    Vector3 linearError = vector3::Subtract(worldPositionB, worldPositionA);
    velocityBias = vector3::Multiply(linearError, errorCorrectionFactor);

}