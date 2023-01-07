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

#include "bepuik/SingleBoneAngularMotor.hpp"
#include "bepuik/Bone.hpp"

void BEPUik::SingleBoneAngularMotor::UpdateJacobiansAndVelocityBias()
{
    linearJacobian = BEPUik::matrix::Create();
	angularJacobian = BEPUik::matrix::GetIdentity();

    //Error is in world space. It gets projected onto the jacobians later.
    Quaternion errorQuaternion;
    errorQuaternion = BEPUik::quaternion::Conjugate(TargetBone->Orientation);
    errorQuaternion = BEPUik::quaternion::Multiply(TargetOrientation, errorQuaternion);
    float angle;
    Vector3 angularError;
    BEPUik::quaternion::GetAxisAngleFromQuaternion(errorQuaternion, angularError, angle);
    angularError = vector3::Multiply(angularError, angle);

    //This is equivalent to projecting the error onto the angular jacobian. The angular jacobian just happens to be the identity matrix!
    velocityBias = vector3::Multiply(angularError, errorCorrectionFactor);
}
