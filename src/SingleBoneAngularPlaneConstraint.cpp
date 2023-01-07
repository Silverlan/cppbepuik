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

#include "bepuik/SingleBoneAngularPlaneConstraint.hpp"
#include "bepuik/Bone.hpp"

void BEPUik::SingleBoneAngularPlaneConstraint::UpdateJacobiansAndVelocityBias()
{
 

    linearJacobian = matrix::Create();

    Vector3 boneAxis;
    boneAxis = quaternion::Transform(BoneLocalAxis, TargetBone->Orientation);

    Vector3 jacobian;
    jacobian = vector3::Cross(boneAxis, PlaneNormal);

	angularJacobian = matrix::Create();
	angularJacobian[0][0]=jacobian.x;
	angularJacobian[0][1]=jacobian.y;
	angularJacobian[0][2]=jacobian.z;
	angularJacobian[1][0]=0;
	angularJacobian[1][1]=0;
	angularJacobian[1][2]=0;
	angularJacobian[2][0]=0;
	angularJacobian[2][1]=0;
	angularJacobian[2][2]=0;


    velocityBias.x = vector3::Dot(boneAxis, PlaneNormal);
    velocityBias.x = -errorCorrectionFactor * velocityBias.x;


}
