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

#include "bepuik/control/DragControl.hpp"
#include "bepuik/SingleBoneLinearMotor.hpp"

BEPUik::DragControl::~DragControl()
{}
BEPUik::Bone* BEPUik::DragControl::GetTargetBone() { return LinearMotor->TargetBone; }
void BEPUik::DragControl::SetTargetBone(Bone* value)
{
	LinearMotor->TargetBone = value;
}

BEPUik::SingleBoneLinearMotor* BEPUik::DragControl::GetLinearMotor() { return LinearMotor.get(); }
void BEPUik::DragControl::SetLinearMotor(std::unique_ptr<SingleBoneLinearMotor> value) { LinearMotor = std::move(value); }

BEPUik::DragControl::DragControl()
{
	LinearMotor = std::make_unique<SingleBoneLinearMotor>();
	LinearMotor->Rigidity = 1;
}

void BEPUik::DragControl::Preupdate(float dt, float updateRate)
{
	LinearMotor->Preupdate(dt, updateRate);
}

void BEPUik::DragControl::UpdateJacobiansAndVelocityBias()
{
	LinearMotor->UpdateJacobiansAndVelocityBias();
}

void BEPUik::DragControl::ComputeEffectiveMass()
{
	LinearMotor->ComputeEffectiveMass();
}

void BEPUik::DragControl::WarmStart()
{
	LinearMotor->WarmStart();
}

void BEPUik::DragControl::SolveVelocityIteration()
{
	LinearMotor->SolveVelocityIteration();
}

void BEPUik::DragControl::ClearAccumulatedImpulses()
{
	LinearMotor->ClearAccumulatedImpulses();
}

float BEPUik::DragControl::GetMaximumForce() const { return LinearMotor->MaximumForce; }
void BEPUik::DragControl::SetMaximumForce(float value) { LinearMotor->MaximumForce = value; }

float BEPUik::DragControl::GetRigidity() const { return LinearMotor->GetRigidity(); }
void BEPUik::DragControl::SetRigidity(float rigidity) { LinearMotor->SetRigidity(rigidity); }

void BEPUik::OrientedDragControl::SetTargetOrientation(const Quaternion& orientation) { m_targetOrientation = orientation; }
const BEPUik::Quaternion& BEPUik::OrientedDragControl::GetTargetOrientation() const { return m_targetOrientation; }

void BEPUik::OrientedDragControl::ClearAccumulatedImpulses()
{
	DragControl::ClearAccumulatedImpulses();
	GetTargetBone()->Orientation = m_targetOrientation;
}
