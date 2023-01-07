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

#include "bepuik/joint/IKJoint.hpp"
#include "bepuik/math.hpp"

namespace BEPUik
{
	class Bone;
    /// <summary>
    /// Attempts to maintain the relative orientation between two bones.
    /// </summary>
    class IKAngularJoint : public IKJoint
    {
	public:
        /// <summary>
        /// Gets or sets the rotation from connection A's orientation to connection B's orientation in A's local space.
        /// </summary>
        Quaternion GoalRelativeOrientation;


        /// <summary>
        /// Constructs a 3DOF angular joint which tries to keep two bones in angular alignment.
        /// </summary>
        /// <param name="connectionA">First bone to connect to the joint.</param>
        /// <param name="connectionB">Second bone to connect to the joint.</param>
        IKAngularJoint(Bone &connectionA, Bone &connectionB);

        virtual void UpdateJacobiansAndVelocityBias() override;
    };
}
