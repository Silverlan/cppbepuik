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

#include <vector>
#include <queue>
#include "bepuik/joint/IKJoint.hpp"
#include "bepuik/Bone.hpp"
#include "bepuik/control/Control.hpp"

namespace BEPUik
{
    /// <summary>
    /// Manages the subset of joints which potentially need solving.
    /// The active joint set contains connected components in the joint-bone graph which interact with control constraints.
    /// These connected components can be bounded by pinned bones which do not transfer any motion.
    /// </summary>
    class ActiveSet
    {
	public:
		ActiveSet()=default;
		ActiveSet(const ActiveSet&)=delete;
		ActiveSet &operator=(const ActiveSet&)=delete;
        std::vector<IKJoint*> joints;
		
        std::vector<Bone*> bones;

        /// <summary>
        /// Gets or sets whether or not to automatically configure the masses of bones in the active set based upon their dependencies.
        /// Enabling this makes the solver more responsive and avoids some potential instability.
        /// This will overwrite any existing mass settings.
        /// </summary>
        bool UseAutomass = true;

        /// <summary>
        /// Gets or sets the multiplier applied to the mass of a bone before distributing it to the child bones.
        /// Used only when UseAutomass is set to true.
        /// </summary>
        float AutomassUnstressedFalloff = 0.9f;

		void SetAutomassUnstressedFalloff(float value);

        float AutomassTarget = 1;
        /// <summary>
        /// Gets or sets the mass that the heaviest bones will have when automass is enabled.
        /// </summary>
        void SetAutomassTarget(float value);
		
        void UpdateActiveSet(std::vector<IKJoint*> &joints);

        /// <summary>
        /// Updates the ordered set of active joints.
        /// Joints are ordered according to their graph traversal distance from control constraints.
        /// Joints close to the control constraints are closer to index 0 than joints that are far from control constraints.
        /// The relative ordering of joints from different connected components is irrelevant; the only guarantee is that
        /// constraints further from the source in a particular connected component are later in the list than those that are close.
        /// </summary>
        /// <param name="controls">Currently active control constraints.</param>
        void UpdateActiveSet(std::vector<Control*> &controls);

        ~ActiveSet();
	private:
        //Stores data aban in-process BFS.
        std::queue<Bone*> bonesToVisit;

		bool BonesHaveInteracted(Bone* bone, Bone* childBone);
        void FindStressedPaths(std::vector<Control*> &controls);

        void NotifyPredecessorsOfStress(Bone *bone);
        void FindStressedPaths(Bone *bone);

        void NotifyPredecessorsOfCycle(Bone *bone);

        void FindCycles(Bone *bone);

        std::vector<Bone*> uniqueChildren;
        void DistributeMass(Bone *bone);

        void DistributeMass(std::vector<Control*> &controls);

        /// <summary>
        /// Clears the bone and joint listings and unsets all flags.
        /// </summary>
        void Clear();

        void Dispose();
    };
}
