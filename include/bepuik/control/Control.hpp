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

#include "bepuik/Bone.hpp"

namespace BEPUik
{
    /// <summary>
    /// Constrains an individual bone in an attempt to reach some goal.
    /// Controls act as groups of single bone constraints. They are used
    /// by the solver to determine the active set of body constraints.
    /// </summary>
    class Control
    {
	public:
        /// <summary>
        /// Gets or sets the controlled bone.
        /// </summary>
        virtual Bone *GetTargetBone()=0;
		virtual void SetTargetBone(Bone *bone)=0;

        virtual void Preupdate(float dt, float updateRate)=0;

        virtual void UpdateJacobiansAndVelocityBias()=0;

        virtual void ComputeEffectiveMass()=0;

        virtual void WarmStart()=0;

        virtual void SolveVelocityIteration()=0;

        virtual void ClearAccumulatedImpulses()=0;

        virtual float GetMaximumForce() const=0;

        virtual void SetMaximumForce(float value)=0;
    };
}