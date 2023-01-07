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

#include "bepuik/SingleBoneLinearMotor.hpp"
#include "bepuik/Bone.hpp"

BEPUik::Vector3 BEPUik::SingleBoneLinearMotor::GetOffset() const { return quaternion::Transform(LocalOffset, TargetBone->Orientation); }
void BEPUik::SingleBoneLinearMotor::SetOffset(const Vector3 &value) { LocalOffset = quaternion::Transform(value, BEPUik::quaternion::Conjugate(TargetBone->Orientation)); }

void BEPUik::SingleBoneLinearMotor::UpdateJacobiansAndVelocityBias()
{
    linearJacobian = matrix::GetIdentity();
    Vector3 r;
    r = quaternion::Transform(LocalOffset, TargetBone->Orientation);
    angularJacobian = matrix::CreateCrossProduct(r);
    //Transposing a skew symmetric matrix is equivalent to negating it.
    angularJacobian = matrix::Transpose(angularJacobian);

    Vector3 worldPosition;
    worldPosition = vector3::Add(TargetBone->Position, r);

    //Error is in world space.
    Vector3 linearError;
    linearError = vector3::Subtract(TargetPosition, worldPosition);
    //This is equivalent to projecting the error onto the linear jacobian. The linear jacobian just happens to be the identity matrix!
    velocityBias = vector3::Multiply(linearError, errorCorrectionFactor);
}
