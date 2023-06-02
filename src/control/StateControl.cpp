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

#include "bepuik/control/StateControl.hpp"
#include "bepuik/SingleBoneLinearMotor.hpp"
#include "bepuik/SingleBoneAngularMotor.hpp"

BEPUik::StateControl::~StateControl() {}
BEPUik::Bone* BEPUik::StateControl::GetTargetBone() { return LinearMotor->TargetBone; }

void BEPUik::StateControl::SetTargetBone(Bone* value)
{
	LinearMotor->SetTargetBone(value);
	AngularMotor->SetTargetBone(value);
	if (value != nullptr)
		AngularMotor->TargetOrientation = value->Orientation;
}

BEPUik::SingleBoneLinearMotor* BEPUik::StateControl::GetLinearMotor() { return LinearMotor.get(); }
void BEPUik::StateControl::SetLinearMotor(std::unique_ptr<SingleBoneLinearMotor> value) { LinearMotor = std::move(value); }

BEPUik::SingleBoneAngularMotor* BEPUik::StateControl::GetAngularMotor() { return AngularMotor.get(); }
void BEPUik::StateControl::SetAngularMotor(std::unique_ptr<SingleBoneAngularMotor> value) { AngularMotor = std::move(value); }

BEPUik::StateControl::StateControl()
{
	LinearMotor = std::make_unique<SingleBoneLinearMotor>();
	AngularMotor = std::make_unique<SingleBoneAngularMotor>();
	LinearMotor->Rigidity = 1;
	AngularMotor->Rigidity = 1;
}

void BEPUik::StateControl::Preupdate(float dt, float updateRate)
{
	LinearMotor->Preupdate(dt, updateRate);
	AngularMotor->Preupdate(dt, updateRate);
}


void BEPUik::StateControl::UpdateJacobiansAndVelocityBias()
{
	LinearMotor->UpdateJacobiansAndVelocityBias();
	AngularMotor->UpdateJacobiansAndVelocityBias();
}

void BEPUik::StateControl::ComputeEffectiveMass()
{
	LinearMotor->ComputeEffectiveMass();
	AngularMotor->ComputeEffectiveMass();
}

void BEPUik::StateControl::WarmStart()
{
	LinearMotor->WarmStart();
	AngularMotor->WarmStart();
}

void BEPUik::StateControl::SolveVelocityIteration()
{
	LinearMotor->SolveVelocityIteration();
	AngularMotor->SolveVelocityIteration();
}

void BEPUik::StateControl::ClearAccumulatedImpulses()
{
	LinearMotor->ClearAccumulatedImpulses();
	AngularMotor->ClearAccumulatedImpulses();
}

float BEPUik::StateControl::GetMaximumForce() const { return LinearMotor->MaximumForce; }
void BEPUik::StateControl::SetMaximumForce(float value)
{
	LinearMotor->MaximumForce = value;
	AngularMotor->MaximumForce = value;
}

float BEPUik::StateControl::GetRigidity() const { return LinearMotor->GetRigidity(); }
void BEPUik::StateControl::SetRigidity(float rigidity) { LinearMotor->SetRigidity(rigidity); AngularMotor->SetRigidity(rigidity); }
