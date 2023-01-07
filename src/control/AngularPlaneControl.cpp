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

#include "bepuik/control/AngularPlaneControl.hpp"
#include "bepuik/SingleBoneAngularPlaneConstraint.hpp"

BEPUik::Bone *BEPUik::AngularPlaneControl::GetTargetBone() {return AngularMotor->TargetBone;}
void BEPUik::AngularPlaneControl::SetTargetBone(Bone *value) {
	AngularMotor->TargetBone = value;
}

BEPUik::SingleBoneAngularPlaneConstraint *BEPUik::AngularPlaneControl::GetAngularMotor() {return AngularMotor;}
void BEPUik::AngularPlaneControl::SetAngularMotor(SingleBoneAngularPlaneConstraint *value) {AngularMotor = value;}

BEPUik::AngularPlaneControl::AngularPlaneControl()
{
    AngularMotor = new SingleBoneAngularPlaneConstraint();
    AngularMotor->Rigidity = 1;
}

void BEPUik::AngularPlaneControl::Preupdate(float dt, float updateRate)
{
    AngularMotor->Preupdate(dt, updateRate);
}

void BEPUik::AngularPlaneControl::UpdateJacobiansAndVelocityBias()
{
    AngularMotor->UpdateJacobiansAndVelocityBias();
}

void BEPUik::AngularPlaneControl::ComputeEffectiveMass()
{
    AngularMotor->ComputeEffectiveMass();
}

void BEPUik::AngularPlaneControl::WarmStart()
{
    AngularMotor->WarmStart();
}

void BEPUik::AngularPlaneControl::SolveVelocityIteration()
{
    AngularMotor->SolveVelocityIteration();
}

void BEPUik::AngularPlaneControl::ClearAccumulatedImpulses()
{
    AngularMotor->ClearAccumulatedImpulses();
}

float BEPUik::AngularPlaneControl::GetMaximumForce() const { return AngularMotor->MaximumForce; }
void BEPUik::AngularPlaneControl::SetMaximumForce(float value)
{
    AngularMotor->MaximumForce = value;
}
