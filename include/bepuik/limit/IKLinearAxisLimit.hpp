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
    /// Keeps an anchor point on one bone between planes defined by another bone.
    /// </summary>
    class IKLinearAxisLimit : public IKLimit
    {
        /// <summary>
        /// Gets or sets the offset in connection A's local space from the center of mass to the anchor point of the line.
        /// </summary>
        Vector3 LocalLineAnchor;

        /// <summary>
        /// Gets or sets the direction of the line in connection A's local space.
        /// Must be unit length.
        /// </summary>
        Vector3 LocalLineDirection;

        /// <summary>
        /// Gets or sets the offset in connection B's local space from the center of mass to the anchor point which will be kept on the line.
        /// </summary>
        Vector3 LocalAnchorB;

        /// <summary>
        /// Gets or sets the world space location of the line anchor attached to connection A.
        /// </summary>
        Vector3 GetLineAnchor() const;
        void SetLineAnchor(const Vector3 &value);

        /// <summary>
        /// Gets or sets the world space direction of the line attached to connection A.
        /// Must be unit length.
        /// </summary>
        Vector3 GetLineDirection() const;
        void SetLineDirection(const Vector3 &value);

        /// <summary>
        /// Gets or sets the offset in world space from the center of mass of connection B to the anchor point.
        /// </summary>
        Vector3 GetAnchorB() const;
        void SetAnchorB(const Vector3 &value);

        float minimumDistance;
        /// <summary>
        /// Gets or sets the minimum distance that the joint connections should be kept from each other.
        /// </summary>
        float GetMinimumDistance() const;
        void SetMinimumDistance(float value);

         float maximumDistance;
        /// <summary>
        /// Gets or sets the maximum distance that the joint connections should be kept from each other.
        /// </summary>
        float GetMaximumDistance() const;
        void SetMaximumDistance(float value);

        /// <summary>
        /// Constructs a new axis limit.
        /// </summary>
        /// <param name="connectionA">First bone connected by the joint.</param>
        /// <param name="connectionB">Second bone connected by the joint.</param>
        /// <param name="lineAnchor">Anchor point of the line attached to the first bone in world space.</param>
        /// <param name="lineDirection">Direction of the line attached to the first bone in world space. Must be unit length.</param>
        /// <param name="anchorB">Anchor point on the second bone in world space which is measured against the other connection's anchor.</param>
        /// <param name="minimumDistance">Minimum distance that the joint connections should be kept from each other along the axis.</param>
        /// <param name="maximumDistance">Maximum distance that the joint connections should be kept from each other along the axis.</param>
        IKLinearAxisLimit(Bone connectionA, Bone connectionB, Vector3 lineAnchor, Vector3 lineDirection, Vector3 anchorB, float minimumDistance, float maximumDistance);

        virtual void UpdateJacobiansAndVelocityBias() override;
    };
}
