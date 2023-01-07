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

#include "bepuik/SingleBoneRevoluteConstraint.hpp"
#include "bepuik/Bone.hpp"

const BEPUik::Vector3 &BEPUik::SingleBoneRevoluteConstraint::GetFreeAxis() const { return freeAxis; }
void BEPUik::SingleBoneRevoluteConstraint::SetFreeAxis(const Vector3 &value)
{
    freeAxis = value;
    constrainedAxis1 = vector3::Cross(freeAxis, vector3::Up);
    if (glm::length2(constrainedAxis1) < Epsilon)
    {
        constrainedAxis1 = vector3::Cross(freeAxis, vector3::Right);
    }
    constrainedAxis1 = glm::normalize(constrainedAxis1);
    constrainedAxis2 = vector3::Cross(freeAxis, constrainedAxis1);
}

void BEPUik::SingleBoneRevoluteConstraint::UpdateJacobiansAndVelocityBias()
{
 

    linearJacobian = matrix::Create();

    Vector3 boneAxis;
    boneAxis = quaternion::Transform(BoneLocalFreeAxis, TargetBone->Orientation);

	angularJacobian = matrix::Create();
	angularJacobian[0][0]=constrainedAxis1.x;
	angularJacobian[0][1]=constrainedAxis1.y;
	angularJacobian[0][2]=constrainedAxis1.z;
	angularJacobian[1][0]=constrainedAxis2.x;
	angularJacobian[1][1]=constrainedAxis2.y;
	angularJacobian[1][2]=constrainedAxis2.z;
	angularJacobian[2][0]=0;
	angularJacobian[2][1]=0;
	angularJacobian[2][2]=0;


    Vector3 error;
    error = vector3::Cross(boneAxis, freeAxis);
    Vector2 constraintSpaceError;
    constraintSpaceError.x = vector3::Dot(error, constrainedAxis1);
    constraintSpaceError.y = vector3::Dot(error, constrainedAxis2);
    velocityBias.x = errorCorrectionFactor * constraintSpaceError.x;
    velocityBias.y = errorCorrectionFactor * constraintSpaceError.y;


}
