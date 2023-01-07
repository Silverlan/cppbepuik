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

namespace BEPUik
{
    //Keeps the anchors from two connections near each other.
    class IKBallSocketJoint : public IKJoint
    {
        /// <summary>
        /// Gets or sets the offset in connection A's local space from the center of mass to the anchor point.
        /// </summary>
        Vector3 LocalOffsetA;
        /// <summary>
        /// Gets or sets the offset in connection B's local space from the center of mass to the anchor point.
        /// </summary>
        Vector3 LocalOffsetB;

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection A to the anchor point.
        /// </summary>
        Vector3 GetOffsetA() const;
		void SetOffsetA(const Vector3 &value);

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection B to the anchor point.
        /// </summary>
        Vector3 GetOffsetB() const;
        void SetOffsetB(const Vector3 &value);

        /// <summary>
        /// Builds a ball socket joint.
        /// </summary>
        /// <param name="connectionA">First connection in the pair.</param>
        /// <param name="connectionB">Second connection in the pair.</param>
        /// <param name="anchor">World space anchor location used to initialize the local anchors.</param>
        IKBallSocketJoint(Bone &connectionA, Bone &connectionB, const Vector3 &anchor);

        virtual void UpdateJacobiansAndVelocityBias() override;
    };
}

