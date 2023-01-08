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

#include <limits>
#include <algorithm>
#include <stdexcept>

namespace BEPUik
{
	class IKConstraint
	{
	public:
		IKConstraint()=default;
		IKConstraint(const IKConstraint*)=delete;
		IKConstraint &operator=(const IKConstraint&)=delete;
		virtual ~IKConstraint() {}
        float softness;

        float errorCorrectionFactor;


        /// <summary>
        /// The rigidity of a constraint is used to derive the stiffness and damping coefficients using a fixed stiffness:damping ratio.
        /// </summary>
        /// <remarks>
        /// This is used over independent coefficients because IK usages of the constraints don't really vary in behavior, just strength.
        /// </remarks>
        static constexpr float StiffnessOverDamping = 0.25f;

        float Rigidity = 16;
        /// <summary>
        /// Gets the rigidity of the constraint. Higher values correspond to more rigid constraints, lower values to less rigid constraints. Must be positive.
        /// </summary>
        /// <remarks>
        /// Scaling up the rigidity is like preparing the constraint to handle a heavier load. If the load is proportionally heavier, the damping ratio stays the same. 
        /// If the load stays the same but the rigidity increases, the result is a more rigid joint, but with a slightly different damping ratio.
        /// In other words, modifying rigidity withmodifying the effective mass of the system results in a variable damping ratio. 
        /// This isn't a huge problem in practice- there is a massive ultra-damping hack in IK bone position integration that make a little physical deviation or underdamping irrelevant.
        /// </remarks>
		float GetRigidity() const;
		void SetRigidity(float value);

        float MaximumImpulse;
        float MaximumImpulseSquared;
        float MaximumForce = std::numeric_limits<float>::max();

        /// <summary>
        /// Gets or sets the maximum force that the constraint can apply.
        /// </summary>
		float GetMaximumForce() const;
		void SetMaximumForce(float value);

        /// <summary>
        /// Updates the softness, bias factor, and maximum impulse based on the current time step.
        /// </summary>
        /// <param name="dt">Time step duration.</param>
        /// <param name="updateRate">Inverse time step duration.</param>
        void Preupdate(float dt, float updateRate);

        /// <summary>
        /// Update the jacobians for the latest position and orientation bone states and store a velocity bias based on the error.
        /// </summary>
        virtual void UpdateJacobiansAndVelocityBias()=0;

        /// <summary>
        /// Computes the effective mass matrix for the constraint for the current jacobians.
        /// </summary>
        virtual void ComputeEffectiveMass()=0;

        /// <summary>
        /// Applies the accumulated impulse to warm up the constraint solving process.
        /// </summary>
        virtual void WarmStart()=0;

        /// <summary>
        /// Applies impulses to satisfy the velocity constraint.
        /// </summary>
        virtual void SolveVelocityIteration()=0;

        /// <summary>
        /// Clears the accumulated impulse.
        /// </summary>
        virtual void ClearAccumulatedImpulses()=0;
    };
}
