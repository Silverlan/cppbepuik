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
    /// <summary>
    /// Keeps the anchor points on two bones at the same distance.
    /// </summary>
    class IKDistanceJoint : public IKJoint
    {
        /// <summary>
        /// Gets or sets the offset in connection A's local space from the center of mass to the anchor point.
        /// </summary>
        Vector3 LocalAnchorA;
        /// <summary>
        /// Gets or sets the offset in connection B's local space from the center of mass to the anchor point.
        /// </summary>
        Vector3 LocalAnchorB;

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection A to the anchor point.
        /// </summary>
        Vector3 GetAnchorA();
        void SetAnchorA(const Vector3 &value);

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection A to the anchor point.
        /// </summary>
        Vector3 GetAnchorB();
        void SetAnchorB(const Vector3 &value);

        float distance;
        /// <summary>
        /// Gets or sets the distance that the joint connections should be kept from each other.
        /// </summary>
        float GetDistance() const;
        void SetDistance(float value);

        /// <summary>
        /// Constructs a new distance joint.
        /// </summary>
        /// <param name="connectionA">First bone connected by the joint.</param>
        /// <param name="connectionB">Second bone connected by the joint.</param>
        /// <param name="anchorA">Anchor point on the first bone in world space.</param>
        /// <param name="anchorB">Anchor point on the second bone in world space.</param>
        IKDistanceJoint(Bone &connectionA, Bone &connectionB, const Vector3 &anchorA, const Vector3 &anchorB);

        virtual void UpdateJacobiansAndVelocityBias() override;
    };
}
