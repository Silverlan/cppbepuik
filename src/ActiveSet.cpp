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

#include "bepuik/ActiveSet.hpp"

void BEPUik::ActiveSet::SetAutomassUnstressedFalloff(float value)
{
	AutomassUnstressedFalloff = std::max(value,0.f);
}

void BEPUik::ActiveSet::SetAutomassTarget(float value)
{
    if (value <= 0)
        throw std::invalid_argument("Mass must be positive.");
    AutomassTarget = value;
}

void BEPUik::ActiveSet::FindStressedPaths(std::vector<Control*> &controls)
{


    //Start a depth first search from each controlled bone to find any pinned bones.
    //All paths from the controlled bone to the pinned bones are 'stressed.'
    //Stressed bones are given greater mass later on.
	for(auto *control : controls)
    {
        //Paths connecting controls should be considered stressed just in case someone tries to pull things apart.
        //Mark bones affected by controls so we can find them in the traversal.
		for(auto *otherControl : controls)
        {
            if (otherControl != control) //Don't include the current control; that could cause false positives for stress cycles.
                otherControl->GetTargetBone()->targetedByOtherControl = true;
        }

        //The control.TargetBone.Parent is null; that's one of the terminating condition for the 'upwards' post-traversal
        //that happens after a pin or stressed path is found.
        FindStressedPaths(control->GetTargetBone());

        //We've analyzed the whole graph for this control. Clean up the bits we used.
		for(auto *bone : bones)
        {
            bone->traversed = false;
            bone->SetActive(false);
            bone->predecessors.clear();
        }
        bones.clear();

        //Get rid of the targetedByOtherControl markings.
		for(auto *otherControl : controls)
        {
            otherControl->GetTargetBone()->targetedByOtherControl = false;
        }
    }

    //All bones in the active set now have their appropriate StressCount values.



}

void BEPUik::ActiveSet::NotifyPredecessorsOfStress(Bone *bone)
{
    //We don't need to tell already-stressed bones abthe fact that they are stressed.
    //Their predecessors are already stressed either by previous notifications like this or
    //through the predecessors being added on after the fact and seeing that the path was stressed.
    if (!bone->traversed)
    {
        bone->traversed = true;
        bone->stressCount++;
		for(auto *predecessor : bone->predecessors)
        {

            NotifyPredecessorsOfStress(predecessor);
        }
    }
}
bool BEPUik::ActiveSet::BonesHaveInteracted(Bone* bone, Bone* childBone)
{
	//Two bones have interacted if one includes the other in its predecessor list.
	return find(bone->predecessors.begin(), bone->predecessors.end(), childBone) != bone->predecessors.end() ||//childBone is a parent of bone. Don't revisit them, that's where we came from!
		find(childBone->predecessors.begin(), childBone->predecessors.end(), bone) != childBone->predecessors.end(); //This bone already explored the childBone; don't do it again.
}
void BEPUik::ActiveSet::FindStressedPaths(Bone *bone)
{
    bone->SetActive(true); //We must keep track of which bones have been visited
    bones.push_back(bone);
	for(auto *joint : bone->joints)
    {
        Bone *boneToAnalyze = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();
        if (BonesHaveInteracted(bone, boneToAnalyze)) //This bone already explored the next bone; don't do it again.
            continue;

        if (!boneToAnalyze->Pinned)
        {
            //The boneToAnalyze is reached by following a path from bone. We record this regardless of whether or not we traverse further.
            //There is one exception: DO NOT create paths to pinned bones!
            boneToAnalyze->predecessors.push_back(bone);
        }

        if (boneToAnalyze->Pinned || boneToAnalyze->traversed)
        {
            //This bone is connected to a pinned bone (or a bone which is directly or indirectly connected to a pinned bone)!
            //This bone and all of its predecessors are a part of a 'stressed path.'
            //This backwards notification is necessary because this depth first search could attempt a deep branch which winds 
            //its way back up to a part of the graph which SHOULD be marked as stressed, but is not yet marked because this path
            //has not popped its way all the way up the stack yet! Left untreated, this would lead to missed stressed paths.
            NotifyPredecessorsOfStress(bone);
            continue;
        }

        if (boneToAnalyze->targetedByOtherControl)
        {
            //We will consider other controls to be sources of stress. This prevents mass ratio issues from allowing multiple controls to tear a structure apart.
            //We do not, however, stop the traversal here. Allow it to continue.         
            NotifyPredecessorsOfStress(bone);
        }
        if (boneToAnalyze->IsActive())
        {
            //The bone has already been visited. We should not proceed.
            //Any bone which is visited but not stressed is either A: not fully explored yet or B: fully explored.
            //Given that we followed an unexplored path to the bone, it must be not fully explored.
            //However, we do not attempt to perform exploration on the bone: any not-yet-fully-explored bones
            //must belong to one of our parents in the DFS! They will take care of it.
            continue;
        }

        //The search hasn't yet found a stressed path or pinned bone yet.
        //Keep on movin' on!
        FindStressedPaths(boneToAnalyze);
        //If a child finds a pin, we will be notified of that fact by the above while loop which traverses the parent pointers.
    }


}

void BEPUik::ActiveSet::NotifyPredecessorsOfCycle(Bone *bone)
{
    //Rather than attempting to only mark cycles, this will simply mark all of the cycle elements and any cycle predecessors up to the unstressed root.
    if (!bone->unstressedCycle && bone->stressCount == 0)
    {
        bone->unstressedCycle = true;
		for(auto *predecessor : bone->predecessors)
        {
            NotifyPredecessorsOfCycle(predecessor);
        }
    }
}

void BEPUik::ActiveSet::FindCycles(Bone *bone)
{
    //The current bone is known to not be stressed.
	for(auto *joint : bone->joints)
    {
        Bone *boneToAnalyze = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();

        if (BonesHaveInteracted(bone, boneToAnalyze)) //Do not attempt to traverse a path which was already traversed *from this bone.*
            continue;
        //We found this bone. Regardless of what happens after, make sure that the bone knows abthis path.
        boneToAnalyze->predecessors.push_back(bone);

        if (boneToAnalyze->IsActive())
        {
            //This bone is butting up against a node which was previously visited.
            //Based on the previous stress path computation, there is only one entry point into an unstressed part of the graph.
            //by the previous condition, we know it's not our immediate parent. We hit an unstressed part of the graph.

            //In other words, this is an unstressed cycle.

            //Rather than attempting to only mark cycles, this will simply mark all of the cycle elements and any cycle predecessors up to the unstressed root.
            NotifyPredecessorsOfCycle(bone);
            continue;
        }
        //Note that no testing for pinned bones is necessary; based on the previous stressed path searches,
        //any unstressed bone is known to not be a path to any pinned bones.

        //The root bone is already added to the active set by the parent breadth-first search.
        //Children are added to the active set.
        boneToAnalyze->SetActive(true);
        bones.push_back(boneToAnalyze);
        FindCycles(boneToAnalyze);
    }
}

void BEPUik::ActiveSet::DistributeMass(Bone *bone)
{
    //Accumulate the number of child joints which we are going to distribute mass to.
	for(auto *joint : bone->joints)
    {
        Bone *boneToAnalyze = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();

        if (boneToAnalyze->traversed || boneToAnalyze->unstressedCycle ||
            find(uniqueChildren.begin(), uniqueChildren.end(), boneToAnalyze) != uniqueChildren.end()) //There could exist multiple joints involved with the same pair of bones; don't continually double count.
        {
            //The bone was already visited or was a member of the stressed path we branched from. Do not proceed.
            continue;
        }
        uniqueChildren.push_back(boneToAnalyze);
    }
    //We distribute a portion of the current bone's total mass to the child bones.
    //By applying a multiplier automassUnstressedFalloff, we guarantee that a chain has a certain maximum weight (excluding cycles).
    //This is thanks to the convergent geometric series sum(automassUnstressedFalloff^n, 1, infinity).
    float massPerChild = uniqueChildren.size() > 0 ? AutomassUnstressedFalloff * bone->GetMass() / uniqueChildren.size() : 0;

    uniqueChildren.clear();
    //(If the number of children is 0, then the only bones which can exist are either bones which were already traversed and will be skipped
    //or bones which are members of unstressed cycles and will inherit the full parent weight. Don't have to worry abthe 0 mass.)

    //The current bone is known to not be stressed.
	for(auto *joint : bone->joints)
    {
        Bone *boneToAnalyze = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();
        //Note that no testing for pinned bones is necessary; based on the previous stressed path searches,
        //any unstressed bone is known to not be a path to any pinned bones.
        if (boneToAnalyze->traversed)// || bone.unstressedCycle)//bone.predecessors.Contains(boneToAnalyze))
        {
            //The bone was already visited or was a member of the stressed path we branched from. Do not proceed.
            continue;
        }

        if (boneToAnalyze->unstressedCycle)
        {
            //This bone is part of a cycle! We cannot give it less mass; that would add in a potential instability.
            //Just give it the current node's full mass.
            boneToAnalyze->SetMass(bone->GetMass());
        }
        else
        {
            //This bone is not a part of a cycle; give it the allotted mass.
            boneToAnalyze->SetMass(massPerChild);
        }
        //The root bone is already added to the traversal set; add the children.
        boneToAnalyze->traversed = true;
        //Note that we do not need to add anything to the bones list here; the previous FindCycles DFS on this unstressed part of the graph did it for us.
        DistributeMass(boneToAnalyze);

    }

}

void BEPUik::ActiveSet::DistributeMass(std::vector<Control*> &controls)
{
    //We assume that all stressed paths have already been marked with nonzero StressCounts.
    //Perform a multi-origin breadth-first search starting at every control. Look for any bones
    //which still have a StressCount of zero.

    //These zero-StressCount bones are the beginnings of isolated 'limbs' in the graph; there is only
    //one bone-to-bone connection (potentially made of multiple constraints, of course, but that does not affect graph connectivity)
    //between the stressed component of the graph and the isolated limb.
    //That means any traversal starting at that first bone and moving away from the stressed graph will never return to the stressed graph
    //(no bone can be revisited).

    //Because these unstressed limbs are not critical weight-carrying paths, they do not need to be as heavy as the stressed paths.
    //In addition, to make the IK more responsive, unstressed bones further from the stressed component of the graph can be made less massive.

    //Care must be taken in determining the masses, though; if the root is light and its children, while individually lighter, are cumulatively much heavier,
    //there could be mass-ratio related instability.

    //To address this, cycles are found and given equal mass and each noncycle branch splits the current object's mass between all noncycle children.


    //Perform a breadth-first search through the graph starting at the bones targeted by each control.
	for(auto *control : controls)
    {
        bonesToVisit.push(control->GetTargetBone());
        //Note that a bone is added to the visited bone set before it is actually processed.
        //This prevents a bone from being put in the queue redundantly.
        control->GetTargetBone()->SetActive(true);
        //A second traversal flag is required for the mass distribution phase on each unstressed part to work efficiently.
        control->GetTargetBone()->traversed = true;
        bones.push_back(control->GetTargetBone());
    }

    //Note that it's technically possible for multiple controls to affect the same bone.
    //The containment tests will stop it from adding in any redundant constraints as a result.
    while (bonesToVisit.size() > 0)
    {
        auto *bone = bonesToVisit.front();
		bonesToVisit.pop();
        if (bone->stressCount == 0)
        {
            bone->SetMass(AutomassUnstressedFalloff);
            //This is an unstressed bone. We should start a DFS to identify any cycles in the unstressed graph.
            FindCycles(bone);
            //Once the cycles are marked, we can proceed through the unstressed graph component and give child bones mass.
            DistributeMass(bone);
            //Do not continue the breadth-first search into the unstressed part of the graph.
            continue;
        }
        else
        {
            //The mass of stressed bones is a multiplier on the number of stressed paths overlapping the bone.
            bone->SetMass(static_cast<float>(bone->stressCount));
        }
        //This bone is not an unstressed branch root. Continue the breadth first search!
		for(auto *joint : bone->joints)
        {
            Bone *boneToAdd = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();
            if (!boneToAdd->Pinned && //Pinned bones act as dead ends! Don't try to traverse them.
                !boneToAdd->IsActive()) //Don't try to add a bone if it's already active.
            {
                boneToAdd->SetActive(true);
                //A second traversal flag is required for the mass distribution phase on each unstressed part to work efficiently.
                boneToAdd->traversed = true;
                boneToAdd->predecessors.push_back(bone);
                //The bone was not already present in the active set. We should visit it!
                //Note that a bone is added to the visited bone set before it is actually processed.
                //This prevents a bone from being put in the queue redundantly.
                bonesToVisit.push(boneToAdd);
                bones.push_back(boneToAdd);
            }
        }
    }

    //Normalize the masses of objects so that the heaviest bones have AutomassTarget mass.
    float lowestInverseMass = std::numeric_limits<float>::max();
	for(auto *bone : bones)
    {
        if (bone->inverseMass < lowestInverseMass)
            lowestInverseMass = bone->inverseMass;
    }

    float inverseMassScale = 1 / (AutomassTarget * lowestInverseMass);

	for(auto *bone : bones)
    {
        //Normalize the mass to the AutomassTarget.
        bone->inverseMass *= inverseMassScale;

        //Also clear the traversal flags while we're at it.
        bone->SetActive(false);
        bone->traversed = false;
        bone->stressCount = 0;
        bone->unstressedCycle = false;
        bone->predecessors.clear();
    }

    bones.clear();
}

void BEPUik::ActiveSet::Clear()
{
    for (int i = 0; i < bones.size(); i++)
    {
        bones[i]->SetActive(false);
        bones[i]->stressCount = 0;
        bones[i]->predecessors.clear();
        bones[i]->SetMass(.01f);
    }
    for (int i = 0; i < joints.size(); i++)
    {
        joints[i]->IsActive = false;
    }
    bones.clear();
    joints.clear();
}

void BEPUik::ActiveSet::UpdateActiveSet(std::vector<IKJoint*> &joints)
{
    //Clear the previous active set to make way for the new active set.   
    //Note that the below flag clearing and usage creates a requirement.
    //Two IKSolvers cannot operate on the same graph; the active set flags could be corrupted.
    Clear();

    for (int i = 0; i < joints.size(); ++i)
    {
        if (joints[i]->GetEnabled())
        {
            if (!joints[i]->m_connectionA->IsActive())
            {
                joints[i]->m_connectionA->SetActive(true);
                bones.push_back(joints[i]->GetConnectionA());
            }

            if (!joints[i]->m_connectionB->IsActive())
            {
                joints[i]->m_connectionB->SetActive(true);
                bones.push_back(joints[i]->GetConnectionB());
            }

            joints.push_back(joints[i]);
        }
    }

    //Use an arbitrary mass for the bones.
    //This could conceivably encounter issues with pathological cases, but we don't have controls to easily guide a better choice.
    if (UseAutomass)
    {
        for (int i = 0; i < bones.size(); ++i)
        {
            bones[i]->SetMass(AutomassTarget);
        }
    }




}

void BEPUik::ActiveSet::UpdateActiveSet(std::vector<Control*> &controls)
{
    //Clear the previous active set to make way for the new active set.
    //Note that the below flag clearing and usage creates a requirement.
    //Two IKSolvers cannot operate on the same graph; the active set flags could be corrupted.
    Clear();

    if (UseAutomass)
    {
        //Identify the stressed bones.
        FindStressedPaths(controls);

        //Compute the dependency graph for all the unstressed bones and assign masses.
        DistributeMass(controls);
    }

    //While we have traversed the whole active set in the previous stressed/unstressed searches, we do not yet have a proper breadth-first constraint ordering available.

    //Perform a breadth-first search through the graph starting at the bones targeted by each control.
	for(auto *control : controls)
    {
        bonesToVisit.push(control->GetTargetBone());
        //Note that a bone is added to the visited bone set before it is actually processed.
        //This prevents a bone from being put in the queue redundantly.
        control->GetTargetBone()->SetActive(true);
        bones.push_back(control->GetTargetBone());
    }

    //Note that it's technically possible for multiple controls to affect the same bone.
    //The containment tests will stop it from adding in any redundant constraints as a result.
    while (bonesToVisit.size() > 0)
    {
        auto *bone = bonesToVisit.front();
		bonesToVisit.pop();
		for(auto *joint : bone->joints)
        {
            if (!joint->IsActive)
            {
                joint->IsActive = true;
                //This is the first time the joint has been visited, so plop it into the list.
                joints.push_back(joint);
            }
            Bone *boneToAdd = joint->GetConnectionA() == bone ? joint->GetConnectionB() : joint->GetConnectionA();
            if (!boneToAdd->Pinned && //Pinned bones act as dead ends! Don't try to traverse them.
                !boneToAdd->IsActive()) //Don't try to add a bone if it's already active.
            {
                boneToAdd->SetActive(true);
                //The bone was not already present in the active set. We should visit it!
                //Note that a bone is added to the visited bone set before it is actually processed.
                //This prevents a bone from being put in the queue redundantly.
                bonesToVisit.push(boneToAdd);
                bones.push_back(boneToAdd);
            }
        }
    }

}

BEPUik::ActiveSet::~ActiveSet()
{
    Dispose();
}

void BEPUik::ActiveSet::Dispose()
{
    Clear();
}
