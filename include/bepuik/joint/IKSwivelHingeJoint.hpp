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
    class IKSwivelHingeJoint : public IKJoint
    {
	public:
        /// <summary>
        /// Gets or sets the free hinge axis attached to connection A in its local space.
        /// </summary>
        Vector3 LocalHingeAxis;
        /// <summary>
        /// Gets or sets the free twist axis attached to connection B in its local space.
        /// </summary>
        Vector3 LocalTwistAxis;


        /// <summary>
        /// Gets or sets the free hinge axis attached to connection A in world space.
        /// </summary>
        Vector3 GetWorldHingeAxis() const;
        void SetWorldHingeAxis(const Vector3 &value);

        /// <summary>
        /// Gets or sets the free twist axis attached to connection B in world space.
        /// </summary>
        Vector3 GetWorldTwistAxis() const;
        void SetWorldTwistAxis(const Vector3 &value);

        /// <summary>
        /// Constructs a new constraint which allows relative angular motion around a hinge axis and a twist axis.
        /// </summary>
        /// <param name="connectionA">First connection of the pair.</param>
        /// <param name="connectionB">Second connection of the pair.</param>
        /// <param name="worldHingeAxis">Hinge axis attached to m_connectionA->
        /// The connected bone will be able to rotate around this axis relative to each other.</param>
        /// <param name="worldTwistAxis">Twist axis attached to m_connectionB->
        /// The connected bones will be able to rotate around this axis relative to each other.</param>
        IKSwivelHingeJoint(Bone &connectionA, Bone &connectionB, const Vector3 &worldHingeAxis, const Vector3 &worldTwistAxis);

        virtual void UpdateJacobiansAndVelocityBias() override;
    };
}
