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
#include "bepuik/math.hpp"

namespace BEPUik
{
    class SingleBoneAngularPlaneConstraint : public SingleBoneConstraint
    {
	public:
        /// <summary>
        /// Gets or sets normal of the plane which the bone's axis will be constrained to..
        /// </summary>
        Vector3 PlaneNormal;



        /// <summary>
        /// Axis to constrain to the plane in the bone's local space.
        /// </summary>
        Vector3 BoneLocalAxis;

        virtual void UpdateJacobiansAndVelocityBias() override;


    };
}
