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

#include "bepuik/limit/IKLimit.hpp"

namespace BEPUik
{
    /// <summary>
    /// Tries to keep the anchor points on two bones within an allowed range of distances.
    /// </summary>
    class IKDistanceLimit : public IKLimit
    {
	public:
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
        Vector3 GetAnchorA() const;
        void SetAnchorA(const Vector3 &value);

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection B to the anchor point.
        /// </summary>
        Vector3 GetAnchorB() const;
        void SetAnchorB(const Vector3 &value);

        float MinimumDistance;
        /// <summary>
        /// Gets or sets the minimum distance that the joint connections should be kept from each other.
        /// </summary>
        float GetMinimumDistance() const;
        void SetMinimumDistance(float value);

        float MaximumDistance;
        /// <summary>
        /// Gets or sets the maximum distance that the joint connections should be kept from each other.
        /// </summary>
        float GetMaximumDistance();
        void SetMaximumDistance(float value);

        /// <summary>
        /// Constructs a new distance joint.
        /// </summary>
        /// <param name="connectionA">First bone connected by the joint.</param>
        /// <param name="connectionB">Second bone connected by the joint.</param>
        /// <param name="anchorA">Anchor point on the first bone in world space.</param>
        /// <param name="anchorB">Anchor point on the second bone in world space.</param>
        /// <param name="minimumDistance">Minimum distance that the joint connections should be kept from each other.</param>
        /// <param name="maximumDistance">Maximum distance that the joint connections should be kept from each other.</param>
        IKDistanceLimit(Bone &connectionA, Bone &connectionB, const Vector3 &anchorA, const Vector3 &anchorB, float minimumDistance, float maximumDistance);

        virtual void UpdateJacobiansAndVelocityBias() override;
    };
}
