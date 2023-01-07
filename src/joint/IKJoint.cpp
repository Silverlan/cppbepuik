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

#include "bepuik/joint/IKJoint.hpp"

BEPUik::Bone *BEPUik::IKJoint::GetConnectionA() {return m_connectionA;}
BEPUik::Bone* BEPUik::IKJoint::GetConnectionB() {return m_connectionB;}

bool BEPUik::IKJoint::GetEnabled() const {return m_enabled;}
void BEPUik::IKJoint::SetEnabled(bool value)
{
    //The bones must know which joints they are associated with so that the bone-joint graph can be traversed.
    if (m_enabled && !value)
    {
		m_connectionA->joints.erase(std::find(m_connectionA->joints.begin(), m_connectionA->joints.end(), this));
		m_connectionB->joints.erase(std::find(m_connectionB->joints.begin(), m_connectionB->joints.end(), this));
    }
    else if (!m_enabled && value)
    {
		m_connectionA->joints.push_back(this);
		m_connectionB->joints.push_back(this);
    }
    m_enabled = value;
}

BEPUik::IKJoint::IKJoint(Bone &connectionA, Bone &connectionB)
{
    m_connectionA = &connectionA;
    m_connectionB = &connectionB;
    SetEnabled(true);
}

void BEPUik::IKJoint::ComputeEffectiveMass()
{
    //For all constraints, the effective mass matrix is 1 / (J * M^-1 * JT).
    //For two bone constraints, J has 4 3x3 matrices. M^-1 (W below) is a 12x12 matrix with 4 3x3 block diagonal matrices.
    //To compute the whole denominator,
    Matrix3x3 linearW;
    Matrix3x3 linearA, angularA, linearB, angularB;

    if (!m_connectionA->Pinned)
    {
        linearW = matrix::CreateScale(m_connectionA->inverseMass);
        linearA = matrix::Multiply(linearJacobianA, linearW); //Compute J * M^-1 for linear component
        linearA = matrix::MultiplyByTransposed(linearA, linearJacobianA); //Compute (J * M^-1) * JT for linear component

        angularA = matrix::Multiply(angularJacobianA, m_connectionA->inertiaTensorInverse); //Compute J * M^-1 for angular component
        angularA = matrix::MultiplyByTransposed(angularA, angularJacobianA); //Compute (J * M^-1) * JT for angular component
    }
    else
    {
        //Treat pinned bones as if they have infinite inertia.
        linearA = matrix::Create();
        angularA = matrix::Create();
    }

    if (!m_connectionB->Pinned)
    {
        linearW = matrix::CreateScale(m_connectionB->inverseMass);
        linearB = matrix::Multiply(linearJacobianB, linearW); //Compute J * M^-1 for linear component
        linearB = matrix::MultiplyByTransposed(linearB, linearJacobianB); //Compute (J * M^-1) * JT for linear component

        angularB = matrix::Multiply(angularJacobianB, m_connectionB->inertiaTensorInverse); //Compute J * M^-1 for angular component
        angularB = matrix::MultiplyByTransposed(angularB, angularJacobianB); //Compute (J * M^-1) * JT for angular component
    }
    else
    {
        //Treat pinned bones as if they have infinite inertia.
        linearB = matrix::Create();
        angularB = matrix::Create();
    }

    //A nice side effect of the block diagonal nature of M^-1 is that the above separated components are now combined into the complete denominator matrix by addition!
    effectiveMass = matrix::Add(linearA, angularA);
    effectiveMass = matrix::Add(effectiveMass, linearB);
    effectiveMass = matrix::Add(effectiveMass, angularB);

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

void BEPUik::IKJoint::WarmStart()
{
    //Take the accumulated impulse and transform it into world space impulses using the jacobians by P = JT * lambda
    //(where P is the impulse, JT is the transposed jacobian matrix, and lambda is the accumulated impulse).
    //Recall the jacobian takes impulses from world space into constraint space, and transpose takes them from constraint space into world space.

    Vector3 impulse;
    if (!m_connectionA->Pinned) //Treat pinned elements as if they have infinite inertia.
    {
        //Compute and apply linear impulse for A.
        impulse = matrix::Transform(accumulatedImpulse, linearJacobianA);
        m_connectionA->ApplyLinearImpulse(impulse);

        //Compute and apply angular impulse for A.
        impulse = matrix::Transform(accumulatedImpulse, angularJacobianA);
        m_connectionA->ApplyAngularImpulse(impulse);
    }

    if (!m_connectionB->Pinned) //Treat pinned elements as if they have infinite inertia.
    {
        //Compute and apply linear impulse for B.
        impulse = matrix::Transform(accumulatedImpulse, linearJacobianB);
        m_connectionB->ApplyLinearImpulse(impulse);

        //Compute and apply angular impulse for B.
        impulse = matrix::Transform(accumulatedImpulse, angularJacobianB);
        m_connectionB->ApplyAngularImpulse(impulse);
    }
}

void BEPUik::IKJoint::SolveVelocityIteration()
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
    //But wait! The accumulated impulse may exceed this constraint's capacity! Check to make sure!
    float impulseSquared = glm::length2(accumulatedImpulse);
    if (impulseSquared > MaximumImpulseSquared)
    {
        //Oops! Clamp that down.
        accumulatedImpulse = vector3::Multiply(accumulatedImpulse, MaximumImpulse / (float)std::sqrt(impulseSquared));
        //Update the impulse based upon the clamped accumulated impulse and the original, pre-add accumulated impulse.
        constraintSpaceImpulse = vector3::Subtract(accumulatedImpulse, preadd);
    }

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

void BEPUik::IKJoint::ClearAccumulatedImpulses()
{
	accumulatedImpulse = {};
}
