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

#include "bepuik/IKSolver.hpp"
#include <cassert>

BEPUik::IKSolver::IKSolver()
{}

void BEPUik::IKSolver::Solve(std::vector<IKJoint*> &joints)
{
    activeSet.UpdateActiveSet(joints);

    //Reset the permutation index; every solve should proceed in exactly the same order.
    permutationMapper.SetPermutationIndex(0);

    float updateRate = 1 / GetTimeStepDuration();
	for(auto *joint : activeSet.joints)
    {
        joint->Preupdate(GetTimeStepDuration(), updateRate);
    }

    for (int i = 0; i < FixerIterationCount; i++)
    {
        //Update the world inertia tensors of objects for the latest position.
		for(auto *bone : activeSet.bones)
        {
            bone->UpdateInertiaTensor();
        }

        //Update the per-constraint jacobians and effective mass for the current bone orientations and positions.
		for(auto *joint : activeSet.joints)
        {
            joint->UpdateJacobiansAndVelocityBias();
            joint->ComputeEffectiveMass();
            joint->WarmStart();
        }

        for (int j = 0; j < VelocitySubiterationCount; j++)
        {
            //A permuted version of the indices is used. The randomization tends to avoid issues with solving order in corner cases.
            for (int jointIndex = 0; jointIndex < activeSet.joints.size(); ++jointIndex)
            {
                auto remappedIndex = permutationMapper.GetMappedIndex(jointIndex, static_cast<int>(activeSet.joints.size()));
                activeSet.joints[remappedIndex]->SolveVelocityIteration();
            }
            //Increment to use the next permutation.
            permutationMapper.SetPermutationIndex(permutationMapper.GetPermutationIndex() +1);
        }

        //Integrate the positions of the bones forward.
		for(auto *bone : activeSet.bones)
        {
            bone->UpdatePosition();
        }
    }

    //Clear accumulated impulses; they should not persist through to another solving round because the state could be arbitrarily different.
    for (int j = 0; j < activeSet.joints.size(); j++)
    {
        activeSet.joints[j]->ClearAccumulatedImpulses();
    }
}

void BEPUik::IKSolver::Solve(std::vector<Control*> &controls)
{
    //Update the list of active joints.
    activeSet.UpdateActiveSet(controls);

    if (AutoscaleControlImpulses)
    {
        //Update the control strengths to match the mass of the target bones and the desired maximum force.
		for(auto *control : controls)
        {
            control->SetMaximumForce(control->GetTargetBone()->GetMass() * AutoscaleControlMaximumForce);
        }
    }

    //Reset the permutation index; every solve should proceed in exactly the same order.
	permutationMapper.SetPermutationIndex(0);

    float updateRate = 1 / GetTimeStepDuration();
	for(auto *joint : activeSet.joints)
    {
        joint->Preupdate(GetTimeStepDuration(), updateRate);
    }
	for(auto *control : controls)
    {
        control->Preupdate(GetTimeStepDuration(), updateRate);
    }
	
    //Go through the set of controls and active joints, updating the state of bones.
    for (int i = 0; i < ControlIterationCount; i++)
    {
        //Update the world inertia tensors of objects for the latest position.
		for(auto *bone : activeSet.bones)
        {
            bone->UpdateInertiaTensor();
        }

        //Update the per-constraint jacobians and effective mass for the current bone orientations and positions.
		for(auto *joint : activeSet.joints)
        {
            joint->UpdateJacobiansAndVelocityBias();
            joint->ComputeEffectiveMass();
            joint->WarmStart();
        }

		for(auto *control : controls)
        {
			assert(!control->GetTargetBone()->Pinned);
            //if (control->GetTargetBone()->Pinned)
            //    throw std::runtime_error("Pinned objects cannot be moved by controls.");
            control->UpdateJacobiansAndVelocityBias();
            control->ComputeEffectiveMass();
            control->WarmStart();
        }

        for (int j = 0; j < VelocitySubiterationCount; j++)
        {
            //Controls are updated first.
			for(auto *control : controls)
            {
                control->SolveVelocityIteration();
            }

            //A permuted version of the indices is used. The randomization tends to avoid issues with solving order in corner cases.
            for (int jointIndex = 0; jointIndex < activeSet.joints.size(); ++jointIndex)
            {
                auto remappedIndex = permutationMapper.GetMappedIndex(jointIndex, static_cast<int>(activeSet.joints.size()));
                activeSet.joints[remappedIndex]->SolveVelocityIteration();
            }
            //Increment to use the next permutation.
            permutationMapper.SetPermutationIndex(permutationMapper.GetPermutationIndex() +1);


        }

        //Integrate the positions of the bones forward.
		for(auto *bone : activeSet.bones)
        {
            bone->UpdatePosition();
        }
    }

    //Clear the control iteration accumulated impulses; they should not persist through to the fixer iterations since the stresses are (potentially) totally different.
    //This just helps stability in some corner cases. Withclearing this, previous high stress would prime the fixer iterations with bad guesses,
    //making the system harder to solve (i.e. introducing instability and requiring more iterations).
    for (int j = 0; j < activeSet.joints.size(); j++)
    {
        activeSet.joints[j]->ClearAccumulatedImpulses();
    }


    //The previous loop may still have significant errors in the active joints due to 
    //unreachable targets. Run a secondary pass withthe influence of the controls to
    //fix the errors withinterference from impossible goals
    //This can potentially cause the bones to move away from the control targets, but with a sufficient
    //number of control iterations, the result is generally a good approximation.
    for (int i = 0; i < FixerIterationCount; i++)
    {
        //Update the world inertia tensors of objects for the latest position.
		for(auto *bone : activeSet.bones)
        {
            bone->UpdateInertiaTensor();
        }

        //Update the per-constraint jacobians and effective mass for the current bone orientations and positions.
		for(auto *joint : activeSet.joints)
        {
            joint->UpdateJacobiansAndVelocityBias();
            joint->ComputeEffectiveMass();
            joint->WarmStart();
        }

        for (int j = 0; j < VelocitySubiterationCount; j++)
        {
            //A permuted version of the indices is used. The randomization tends to avoid issues with solving order in corner cases.
            for (int jointIndex = 0; jointIndex < activeSet.joints.size(); ++jointIndex)
            {
                auto remappedIndex = permutationMapper.GetMappedIndex(jointIndex, static_cast<int>(activeSet.joints.size()));
                activeSet.joints[remappedIndex]->SolveVelocityIteration();
            }
            //Increment to use the next permutation.
            permutationMapper.SetPermutationIndex(permutationMapper.GetPermutationIndex() +1);

        }

        //Integrate the positions of the bones forward.
		for(auto *bone : activeSet.bones)
        {
            bone->UpdatePosition();
        }
    }

    //Clear accumulated impulses; they should not persist through to another solving round because the state could be arbitrarily different.
    for (int j = 0; j < activeSet.joints.size(); j++)
    {
        activeSet.joints[j]->ClearAccumulatedImpulses();
    }

	for(auto *control : controls)
    {
        control->ClearAccumulatedImpulses();
    }
}

BEPUik::IKSolver::~IKSolver()
{
}
