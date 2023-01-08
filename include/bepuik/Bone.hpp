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

#include "bepuik/math.hpp"
#include <vector>

namespace BEPUik
{
	class IKJoint;
    /// <summary>
    /// Piece of a character which is moved by constraints.
    /// </summary>
    class Bone
    {
	public:
		Bone()=default;
		Bone(const Bone&)=delete;
		Bone &operator=(const Bone&)=delete;
        std::vector<IKJoint*> joints;

        /// <summary>
        /// Gets or sets the position of the bone.
        /// </summary>
        Vector3 Position;

        /// <summary>
        /// Gets or sets the orientation of the bone.
        /// </summary>
        Quaternion Orientation = quat_identity;

        /// <summary>
        /// The mid-iteration angular velocity associated with the bone.
        /// This is computed during the velocity subiterations and then applied to the orientation at the end of each position iteration.
        /// </summary>
        Vector3 angularVelocity;

        /// <summary>
        /// The mid-iteration linear velocity associated with the bone.
        /// This is computed during the velocity subiterations and then applied to the position at the end of each position iteration.
        /// </summary>
        Vector3 linearVelocity;


        float inverseMass;

        /// <summary>
        /// Gets or sets the mass of the bone.
        /// High mass bones resist motion more than those of small mass.
        /// Setting the mass updates the inertia tensor of the bone.
        /// </summary>
		float GetMass() const;
		void SetMass(float mass);

        Matrix3x3 inertiaTensorInverse;
        Matrix3x3 localInertiaTensorInverse;

        /// <summary>
        /// An arbitrary scaling factor is applied to the inertia tensor. This tends to improve stability.
        /// </summary>
        float InertiaTensorScaling = 2.5f;

        /// <summary>
        /// Gets the list of joints affecting this bone.
        /// </summary>
		const std::vector<IKJoint*> &GetJoints();


        /// <summary>
        /// Gets or sets whether or not this bone is pinned. Pinned bones cannot be moved by constraints.
        /// </summary>
		bool Pinned = false;
		bool GetPinned() const;
		void SetPinned(bool value);

        /// <summary>
        /// Gets whether or not the bone is a member of the active set as determined by the last IK solver execution.
        /// </summary>
		bool Active = false;
		bool IsActive() const;
		void SetActive(bool value);

        float radius;
        /// <summary>
        /// Gets or sets the radius of the bone.
        /// Setting the radius changes the inertia tensor of the bone.
        /// </summary>
		float GetRadius() const;
		void SetRadius(float value);

        float halfHeight;
        /// <summary>
        /// Gets or sets the height, divided by two, of the bone.
        /// The half height extends both ways from the center position of the bone.
        /// Setting the half height changes the inertia tensor of the bone.
        /// </summary>
        float HalfHeight;
		float GetHalfHeight() const;
		void SetHalfHeight(float value);

        /// <summary>
        /// Gets or sets the height of the bone.
        /// Setting the height changes the inertia tensor of the bone.
        /// </summary>
		float GetHeight() const;
		void SetHeight(float value);

		void SetInertiaTensorScaling(float inertiaTensorScaling);

        /// <summary>
        /// Constructs a new bone.
        /// </summary>
        /// <param name="position">Initial position of the bone.</param>
        /// <param name="orientation">Initial orientation of the bone.</param>
        /// <param name="radius">Radius of the bone.</param>
        /// <param name="height">Height of the bone.</param>
        /// <param name="mass">Mass of the bone.</param>
        Bone(const Vector3 &position, const Quaternion &orientation, float radius, float height, float mass);

        /// <summary>
        /// Constructs a new bone. Assumes the mass will be set later.
        /// </summary>
        /// <param name="position">Initial position of the bone.</param>
        /// <param name="orientation">Initial orientation of the bone.</param>
        /// <param name="radius">Radius of the bone.</param>
        /// <param name="height">Height of the bone.</param>
        Bone(const Vector3 &position, const Quaternion &orientation, float radius, float height);


        void ComputeLocalInertiaTensor();

        /// <summary>
        /// Updates the world inertia tensor based upon the local inertia tensor and current orientation.
        /// </summary>
        void UpdateInertiaTensor();

        /// <summary>
        /// Integrates the position and orientation of the bone forward based upon the current linear and angular velocity.
        /// </summary>
        void UpdatePosition();

        void ApplyLinearImpulse(Vector3 &impulse);

        void ApplyAngularImpulse(Vector3 &impulse);

        /// <summary>
        /// Used by the per-control traversals to find stressed paths.
        /// It has to be separate from the IsActive flag because the IsActive flag is used in the same traversal
        /// to denote all visited bones (including unstressed ones).
        /// Also used in the unstressed traversals; FindCycles uses the IsActive flag and the following DistributeMass phase uses the traversed flag.
        /// </summary>
        bool traversed;

        /// <summary>
        /// The number of stressed paths which use this bone. A stressed path is a possible path between a pin and a control.
        /// </summary>
        int stressCount;

        /// <summary>
        /// The set of parents of a given bone in a traversal. This is like a list of parents; there can be multiple incoming paths and they all need to be kept handy in order to perform some traversal details.
        /// </summary>
        std::vector<Bone*> predecessors;

        /// <summary>
        /// True of the bone is a member of a cycle in an unstressed part of the graph or an unstressed predecessor of an unstressed cycle.
        /// Marking all the predecessors is conceptually simpler than attempting to mark the cycles in isolation.
        /// </summary>
        bool unstressedCycle;
        
        /// <summary>
        /// True if the bone is targeted by a control in the current stress cycle traversal that isn't the current source control.
        /// </summary>
        bool targetedByOtherControl;
    };
}
