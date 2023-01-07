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

#include "bepuik/joint/IKAngularJoint.hpp"

BEPUik::IKAngularJoint::IKAngularJoint(Bone &connectionA, Bone &connectionB)
    : IKJoint(connectionA, connectionB)
{  
    Quaternion orientationAConjugate;
	orientationAConjugate = glm::conjugate(m_connectionA->Orientation);
    //Store the orientation from A to B in A's local space in the GoalRelativeOrientation.
    GoalRelativeOrientation = quaternion::Concatenate(m_connectionB->Orientation, orientationAConjugate);

}

void BEPUik::IKAngularJoint::UpdateJacobiansAndVelocityBias()
{
	linearJacobianA = matrix::Create();
	linearJacobianB = matrix::Create();
    angularJacobianA = matrix::Create(1.f);
    angularJacobianB = matrix::Create(-1.f);

    //The error is computed using this equation:
    //GoalRelativeOrientation * m_connectionA->Orientation * Error = m_connectionB->Orientation
    //GoalRelativeOrientation is the original rotation from A to B in A's local space.
    //Multiplying by A's orientation gives us where B *should* be.
    //Of course, B won't be exactly where it should be after initialization.
    //The Error component holds the difference between what is and what should be.
    //Error = (GoalRelativeOrientation * m_connectionA->Orientation)^-1 * m_connectionB->Orientation
    Quaternion bTarget = quaternion::Concatenate(GoalRelativeOrientation, m_connectionA->Orientation);
    Quaternion bTargetConjugate = BEPUik::quaternion::Conjugate(bTarget);

    Quaternion error = quaternion::Concatenate(bTargetConjugate, m_connectionB->Orientation);

    //Convert the error into an axis-angle vector usable for bias velocity.
    float angle;
    Vector3 axis;
    BEPUik::quaternion::GetAxisAngleFromQuaternion(error, axis, angle);

    velocityBias.x = errorCorrectionFactor * axis.x * angle;
    velocityBias.y = errorCorrectionFactor * axis.y * angle;
    velocityBias.z = errorCorrectionFactor * axis.z * angle;


}
