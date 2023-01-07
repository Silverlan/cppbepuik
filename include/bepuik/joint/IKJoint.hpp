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

#pragma once

#include "bepuik/IKConstraint.hpp"
#include "bepuik/Bone.hpp"
#include "bepuik/math.hpp"

namespace BEPUik
{
    /// <summary>
    /// Connects two bones together.
    /// </summary>
    class IKJoint : public IKConstraint
    {
	public:
        /// <summary>
        /// Gets the first bone connected by this joint.
        /// </summary>
        Bone *GetConnectionA();
        /// <summary>
        /// Gets the second bone connected by this joint.
        /// </summary>
		Bone* GetConnectionB();

        /// <summary>
        /// Gets whether or not the joint is a member of the active set as determined by the last IK solver execution.
        /// </summary>
        bool IsActive = false;

		Bone *m_connectionA;
		Bone *m_connectionB;
        bool m_enabled = false;
        /// <summary>
        /// Gets or sets whether or not this joint is enabled. If set to true, this joint will be a part of
        /// the joint graph and will undergo solving. If set to false, this joint will be removed from the connected bones and will no longer be traversable.
        /// </summary>
		bool GetEnabled() const;
		void SetEnabled(bool value);

        IKJoint(Bone &connectionA, Bone &connectionB);



        Vector3 velocityBias;
        Matrix3x3 linearJacobianA;
        Matrix3x3 angularJacobianA;
        Matrix3x3 linearJacobianB;
        Matrix3x3 angularJacobianB;
        Matrix3x3 effectiveMass;

		Vector3 accumulatedImpulse {0.f,0.f,0.f};





        virtual void ComputeEffectiveMass() override;

        virtual void WarmStart() override;

        virtual void SolveVelocityIteration() override;

        virtual void ClearAccumulatedImpulses() override;
    };
}
