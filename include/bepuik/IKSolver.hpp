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

#include "bepuik/ActiveSet.hpp"
#include "bepuik/PermutationMapper.hpp"

namespace BEPUik
{
    /// <summary>
    /// <para>
    /// This is a little experimental project designed to iteratively converge to a decent solution
    /// to full body inverse kinematics subject to a variety of constraints.
    /// </para>
    /// <para>
    /// It's currently separated from the rest of BEPUphysics library internals because the immediate goal is to test 
    /// features to be potentially integrated into a blender content pipeline. BEPUphysics interactions with this system
    /// will have to go through the interfaces like everything else for now.
    /// </para>
    /// </summary>
    class IKSolver
    {
	public:
		IKSolver(const IKSolver*)=delete;
		IKSolver &operator=(const IKSolver&)=delete;
        /// <summary>
        /// Gets the active joint set associated with the solver.
        /// </summary>
        ActiveSet activeSet;

        /// <summary>
        /// Gets or sets the number of solver iterations to perform in an attempt to reach specified goals.
        /// </summary>
        int ControlIterationCount = 50;

        /// <summary>
        /// Gets or sets the number of solter iterations to perform after the control iterations in an attempt to minimize
        /// errors introduced by unreachable goals.
        /// </summary>
        int FixerIterationCount = 20;

        /// <summary>
        /// Gets or sets the number of velocity iterations to perform per control or fixer iteration.
        /// </summary>
        int VelocitySubiterationCount = 3;

        /// <summary>
        /// Gets or sets whether or not to scale control impulses such that they fit well with the mass of objects.
        /// </summary>
        bool AutoscaleControlImpulses = true;

        /// <summary>
        /// Gets or sets the maximum impulse the controls will try to push bones with when AutoscaleControlImpulses is enabled.
        /// </summary>
        float AutoscaleControlMaximumForce = FLT_MAX;

        /// <summary>
        /// Gets or sets the time step duration elapsed by each position iteration.
        /// </summary>
        float GetTimeStepDuration() const { return timeStepDuration; }
        void SetTimeStepDuration(float value)
        {
            if (value <= 0)
                throw std::invalid_argument("Time step duration must be positive.");
            timeStepDuration = value;
        }

        /// <summary>
        /// Constructs a new IKSolver.
        /// </summary>
        IKSolver();

        /// <summary>
        /// Updates the positions of bones acted upon by the joints given to this solver.
        /// This variant of the solver can be used when there are no goal-driven controls in the simulation.
        /// This amounts to running just the 'fixer iterations' of a normal control-driven solve.
        /// </summary>
        void Solve(std::vector<IKJoint*> &joints);


        /// <summary>
        /// Updates the positions of bones acted upon by the controls given to this solver.
        /// </summary>
        /// <param name="controls">List of currently active controls.</param>
        void Solve(std::vector<Control*> &controls);


        ~IKSolver();

	private:
        float timeStepDuration = 1.0f;
		PermutationMapper permutationMapper;
    };
}
