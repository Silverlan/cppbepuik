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

#include "bepuik/limit/IKLimit.hpp"

BEPUik::IKLimit::IKLimit(Bone &connectionA, Bone &connectionB)
    : IKJoint(connectionA, connectionB)
{
}

void BEPUik::IKLimit::SolveVelocityIteration()
{
    //Compute the 'relative' linear and angular velocities. For single bone constraints, it's based entirely on the one bone's velocities!
    //They have to be pulled into constraint space first to compute the necessary impulse, though.
    Vector3 linearContributionA;
    linearContributionA = matrix::TransformTranspose(m_connectionA->linearVelocity, linearJacobianA);
    Vector3 angularContributionA;
    angularContributionA = matrix::TransformTranspose(m_connectionA->angularVelocity, angularJacobianA);
    Vector3 linearContributionB;
    linearContributionB = matrix::TransformTranspose(m_connectionB->linearVelocity, linearJacobianB);
    Vector3 angularContributionB;
    angularContributionB = matrix::TransformTranspose(m_connectionB->angularVelocity, angularJacobianB);

    //The constraint velocity error will be the velocity we try to remove.
    Vector3 constraintVelocityError;
    constraintVelocityError = vector3::Add(linearContributionA, angularContributionA);
    constraintVelocityError = vector3::Add(constraintVelocityError, linearContributionB);
    constraintVelocityError = vector3::Add(constraintVelocityError, angularContributionB);
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
    //Limits can only apply positive impulses.
    accumulatedImpulse = vector3::Max(vector3::Zero, accumulatedImpulse);
    //But wait! The accumulated impulse may exceed this constraint's capacity! Check to make sure!
    float impulseSquared = vector3::LengthSqr(accumulatedImpulse);
    if (impulseSquared > MaximumImpulseSquared)
    {
        //Oops! Clamp that down.
        accumulatedImpulse = vector3::Multiply(accumulatedImpulse, MaximumImpulse / (float)std::sqrt(impulseSquared));
    }
    //Update the impulse based upon the clamped accumulated impulse and the original, pre-add accumulated impulse.
    constraintSpaceImpulse = vector3::Subtract(accumulatedImpulse, preadd);

    //The constraint space impulse now represents the impulse we want to apply to the bone... but in constraint space.
    //Bring it to world space using the transposed jacobian.
    if (!m_connectionA->Pinned)//Treat pinned elements as if they have infinite inertia.
    {
        Vector3 linearImpulseA;
        linearImpulseA = matrix::Transform(constraintSpaceImpulse, linearJacobianA);
        Vector3 angularImpulseA;
        angularImpulseA = matrix::Transform(constraintSpaceImpulse, angularJacobianA);

        //Apply them!
        m_connectionA->ApplyLinearImpulse(linearImpulseA);
        m_connectionA->ApplyAngularImpulse(angularImpulseA);
    }
    if (!m_connectionB->Pinned)//Treat pinned elements as if they have infinite inertia.
    {
        Vector3 linearImpulseB;
        linearImpulseB = matrix::Transform(constraintSpaceImpulse, linearJacobianB);
        Vector3 angularImpulseB;
        angularImpulseB = matrix::Transform(constraintSpaceImpulse, angularJacobianB);

        //Apply them!
        m_connectionB->ApplyLinearImpulse(linearImpulseB);
        m_connectionB->ApplyAngularImpulse(angularImpulseB);
    }

}
