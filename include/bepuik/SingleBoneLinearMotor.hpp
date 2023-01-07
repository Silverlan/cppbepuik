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

#include "bepuik/SingleBoneConstraint.hpp"

namespace BEPUik
{
    class SingleBoneLinearMotor : public SingleBoneConstraint
    {
	public:
        /// <summary>
        /// Gets or sets the target position to apply to the target bone.
        /// </summary>
        Vector3 TargetPosition;

        /// <summary>
        /// Gets or sets the offset in the bone's local space to the point which will be pulled towards the target position.
        /// </summary>
        Vector3 LocalOffset;


        Vector3 GetOffset() const;
        void SetOffset(const Vector3 &value);

        virtual void UpdateJacobiansAndVelocityBias() override;


    };
}
