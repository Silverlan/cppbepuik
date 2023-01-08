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
#include <memory>

namespace BEPUik
{
	class SingleBoneAngularPlaneConstraint;
    /// <summary>
    /// Constrains an individual bone in an attempt to keep a bone-attached axis flat on a plane defined by a world space normal.
    /// </summary>
    class AngularPlaneControl : public Control
    {
	public:
		virtual ~AngularPlaneControl() override;
        /// <summary>
        /// Gets or sets the controlled bone.
        /// </summary>
		virtual Bone *GetTargetBone() override;
		virtual void SetTargetBone(Bone *value) override;

        /// <summary>
        /// Gets or sets the linear motor used by the control.
        /// </summary>
        std::unique_ptr<SingleBoneAngularPlaneConstraint> AngularMotor;
        SingleBoneAngularPlaneConstraint *GetAngularMotor();
		void SetAngularMotor(std::unique_ptr<SingleBoneAngularPlaneConstraint> value);

        AngularPlaneControl();

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