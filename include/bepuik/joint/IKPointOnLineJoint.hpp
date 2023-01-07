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
    /// Keeps a point on one body on top of a line attached to the other body.
    /// </summary>
    class IKPointOnLineJoint : public IKJoint
    {
        /// <summary>
        /// Gets or sets the offset in connection A's local space from the center of mass to the anchor point of the line.
        /// </summary>
        Vector3 LocalLineAnchor;

        Vector3 localLineDirection;
        /// <summary>
        /// Gets or sets the direction of the line in connection A's local space.
        /// Must be unit length.
        /// </summary>
        const Vector3 &GetLocalLineDirection() const;
        void SetLocalLineDirection(const Vector3 &value);


        /// <summary>
        /// Gets or sets the offset in connection B's local space from the center of mass to the anchor point which will be kept on the line.
        /// </summary>
        Vector3 LocalAnchorB;

        /// <summary>
        /// Gets or sets the world space location of the line anchor attached to connection A.
        /// </summary>
        Vector3 GetLineAnchor();
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
        Vector3 GetAnchorB();
        void SetAnchorB(const Vector3 &value);

        Vector3 localRestrictedAxis1, localRestrictedAxis2;
        void ComputeRestrictedAxes();

        /// <summary>
        /// Constructs a new point on line joint.
        /// </summary>
        /// <param name="connectionA">First bone connected by the joint.</param>
        /// <param name="connectionB">Second bone connected by the joint.</param>
        /// <param name="lineAnchor">Anchor point of the line attached to the first bone in world space.</param>
        /// <param name="lineDirection">Direction of the line attached to the first bone in world space. Must be unit length.</param>
        /// <param name="anchorB">Anchor point on the second bone in world space which tries to stay on connection A's line.</param>
        IKPointOnLineJoint(Bone &connectionA, Bone &connectionB, const Vector3 &lineAnchor, const Vector3 &lineDirection, const Vector3 &anchorB);

        virtual void UpdateJacobiansAndVelocityBias() override;
    };
}
