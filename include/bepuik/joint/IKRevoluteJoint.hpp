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
    class IKRevoluteJoint : public IKJoint
    {
	public:
        Vector3 localFreeAxisA;
        /// <summary>
        /// Gets or sets the free axis in connection A's local space.
        /// Must be unit length.
        /// </summary>
        const Vector3 &GetLocalFreeAxisA() const;
        void SetLocalFreeAxisA(const Vector3 &value);

        Vector3 localFreeAxisB;
        /// <summary>
        /// Gets or sets the free axis in connection B's local space.
        /// Must be unit length.
        /// </summary>
        const Vector3 &GetLocalFreeAxisB() const;
        void SetLocalFreeAxisB(const Vector3 &value);



        /// <summary>
        /// Gets or sets the free axis attached to connection A in world space.
        /// This does not change the other connection's free axis.
        /// </summary>
        Vector3 GetWorldFreeAxisA() const;
        void SetWorldFreeAxisA(const Vector3 &value);

        /// <summary>
        /// Gets or sets the free axis attached to connection B in world space.
        /// This does not change the other connection's free axis.
        /// </summary>
        Vector3 GetWorldFreeAxisB() const;
        void SetWorldFreeAxisB(const Vector3 &value);

        Vector3 localConstrainedAxis1, localConstrainedAxis2;
        void ComputeConstrainedAxes();

        /// <summary>
        /// Constructs a new orientation joint.
        /// Orientation joints can be used to simulate the angular portion of a hinge.
        /// Orientation joints allow rotation around only a single axis.
        /// </summary>
        /// <param name="connectionA">First entity connected in the orientation joint.</param>
        /// <param name="connectionB">Second entity connected in the orientation joint.</param>
        /// <param name="freeAxis">Axis allowed to rotate freely in world space.</param>
        IKRevoluteJoint(Bone &connectionA, Bone &connectionB, const Vector3 &freeAxis);

        virtual void UpdateJacobiansAndVelocityBias() override;
    };
}
