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

#include "bepuik/control/Control.hpp"

namespace BEPUik
{
	class SingleBoneRevoluteConstraint;
    /// <summary>
    /// Constrains an individual bone in an attempt to keep a bone-attached axis aligned with a specified world axis.
    /// </summary>
    class RevoluteControl : public Control
    {
        /// <summary>
        /// Gets or sets the controlled bone.
        /// </summary>
        virtual Bone *GetTargetBone() override;
        virtual void SetTargetBone(Bone *value) override;

        /// <summary>
        /// Gets or sets the linear motor used by the control.
        /// </summary>
        SingleBoneRevoluteConstraint *AngularMotor;
		SingleBoneRevoluteConstraint *GetAngularMotor();
		void SetAngularMotor(SingleBoneRevoluteConstraint *value);

        RevoluteControl();

        virtual void Preupdate(float dt, float updateRate) override;

        virtual void UpdateJacobiansAndVelocityBias() override;

        virtual void ComputeEffectiveMass() override;

        virtual void WarmStart() override;

        virtual void SolveVelocityIteration() override;

        virtual void ClearAccumulatedImpulses() override;

        virtual float GetMaximumForce() const override;
        virtual void SetMaximumForce(float value) override;
    };
}