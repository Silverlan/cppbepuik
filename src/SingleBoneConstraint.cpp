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

#include "bepuik/SingleBoneConstraint.hpp"
#include "bepuik/Bone.hpp"

BEPUik::Bone *BEPUik::SingleBoneConstraint::GetTargetBone() {return TargetBone;}
void BEPUik::SingleBoneConstraint::SetTargetBone(Bone *bone) {TargetBone = bone;}
void BEPUik::SingleBoneConstraint::ComputeEffectiveMass()
{
    //For all constraints, the effective mass matrix is 1 / (J * M^-1 * JT).
    //For single bone constraints, J has 2 3x3 matrices. M^-1 (W below) is a 6x6 matrix with 2 3x3 block diagonal matrices.
    //To compute the whole denominator,
    Matrix3x3 linearW;
    linearW = matrix::CreateScale(TargetBone->inverseMass);
    Matrix3x3 linear;
    linear = matrix::Multiply(linearJacobian, linearW); //Compute J * M^-1 for linear component
    linear = matrix::MultiplyByTransposed(linear, linearJacobian); //Compute (J * M^-1) * JT for linear component

    Matrix3x3 angular;
    angular = matrix::Multiply(angularJacobian, TargetBone->inertiaTensorInverse); //Compute J * M^-1 for angular component
    angular = matrix::MultiplyByTransposed(angular, angularJacobian); //Compute (J * M^-1) * JT for angular component

    //A nice side effect of the block diagonal nature of M^-1 is that the above separated components are now combined into the complete denominator matrix by addition!
    effectiveMass = matrix::Add(linear, angular);

    //Incorporate the constraint softness into the effective mass denominator. This pushes the matrix away from singularity.
    //Softness will also be incorporated into the velocity solve iterations to complete the implementation.
    if (effectiveMass[0][0] != 0)
        effectiveMass[0][0] += softness;
    if (effectiveMass[1][1] != 0)
        effectiveMass[1][1] += softness;
    if (effectiveMass[2][2] != 0)
        effectiveMass[2][2] += softness;

    //Invert! Takes us from J * M^-1 * JT to 1 / (J * M^-1 * JT).
    effectiveMass = matrix::AdaptiveInvert(effectiveMass);

}

void BEPUik::SingleBoneConstraint::WarmStart()
{
    //Take the accumulated impulse and transform it into world space impulses using the jacobians by P = JT * lambda
    //(where P is the impulse, JT is the transposed jacobian matrix, and lambda is the accumulated impulse).
    //Recall the jacobian takes impulses from world space into constraint space, and transpose takes them from constraint space into world space.
    //Compute and apply linear impulse.
    Vector3 impulse;
    impulse = matrix::Transform(accumulatedImpulse, linearJacobian);
    TargetBone->ApplyLinearImpulse(impulse);

    //Compute and apply angular impulse.
    impulse = matrix::Transform(accumulatedImpulse, angularJacobian);
    TargetBone->ApplyAngularImpulse(impulse);
}

void BEPUik::SingleBoneConstraint::SolveVelocityIteration()
{
    //Compute the 'relative' linear and angular velocities. For single bone constraints, it's based entirely on the one bone's velocities!
    //They have to be pulled into constraint space first to compute the necessary impulse, though.
    Vector3 linearContribution;
    linearContribution = matrix::TransformTranspose(TargetBone->linearVelocity, linearJacobian);
    Vector3 angularContribution;
    angularContribution = matrix::TransformTranspose(TargetBone->angularVelocity, angularJacobian);

    //The constraint velocity error will be the velocity we try to remove.
    Vector3 constraintVelocityError;
    constraintVelocityError = vector3::Add(linearContribution, angularContribution);
    //However, we need to take into account two extra sources of velocities which modify our target velocity away from zero.
    //First, the velocity bias from position correction:
    constraintVelocityError = vector3::Subtract(constraintVelocityError, velocityBias);
    //And second, the bias from softness:
    Vector3 softnessBias;
    softnessBias = vector3::Multiply(accumulatedImpulse, -softness);
    constraintVelocityError = vector3::Subtract(constraintVelocityError, softnessBias);

    //By now, the constraint velocity error contains all the velocity we want to get rid of.
    //Convert it into an impulse using the effective mass matrix.
    Vector3 constraintSpaceImpulse;
    constraintSpaceImpulse = matrix::Transform(constraintVelocityError, effectiveMass);

    constraintSpaceImpulse = vector3::Negate(constraintSpaceImpulse);

    //Add the constraint space impulse to the accumulated impulse so that warm starting and softness work properly.
    Vector3 preadd = accumulatedImpulse;
    accumulatedImpulse = vector3::Add(constraintSpaceImpulse, accumulatedImpulse);
    //But wait! The accumulated impulse may exceed this constraint's capacity! Check to make sure!
    float impulseSquared = vector3::LengthSqr(accumulatedImpulse);
    if (impulseSquared > MaximumImpulseSquared)
    {
        //Oops! Clamp that down.
        accumulatedImpulse = vector3::Multiply(accumulatedImpulse, MaximumImpulse / (float)std::sqrt(impulseSquared));
        //Update the impulse based upon the clamped accumulated impulse and the original, pre-add accumulated impulse.
        constraintSpaceImpulse = vector3::Subtract(accumulatedImpulse, preadd);
    }

    //The constraint space impulse now represents the impulse we want to apply to the bone... but in constraint space.
    //Bring it to world space using the transposed jacobian.
    Vector3 linearImpulse;
    linearImpulse = matrix::Transform(constraintSpaceImpulse, linearJacobian);
    Vector3 angularImpulse;
    angularImpulse = matrix::Transform(constraintSpaceImpulse, angularJacobian);

    //Apply them!
    TargetBone->ApplyLinearImpulse(linearImpulse);
    TargetBone->ApplyAngularImpulse(angularImpulse);
}

void BEPUik::SingleBoneConstraint::ClearAccumulatedImpulses()
{
    accumulatedImpulse = vector3::Create();
}
