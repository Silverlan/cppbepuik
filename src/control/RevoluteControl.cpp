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

#include "bepuik/RevoluteControl.hpp"
#include "bepuik/SingleBoneRevoluteConstraint.hpp"

BEPUik::Bone *BEPUik::RevoluteControl::GetTargetBone() { return AngularMotor->TargetBone; }
void BEPUik::RevoluteControl::SetTargetBone(Bone *value)
{
    AngularMotor->TargetBone = value;
}

BEPUik::SingleBoneRevoluteConstraint *BEPUik::RevoluteControl::GetAngularMotor() {return AngularMotor;}
void BEPUik::RevoluteControl::SetAngularMotor(SingleBoneRevoluteConstraint *value) {AngularMotor = value;}

BEPUik::RevoluteControl::RevoluteControl()
{
    AngularMotor = new SingleBoneRevoluteConstraint();
    AngularMotor->Rigidity = 1;
}

void BEPUik::RevoluteControl::Preupdate(float dt, float updateRate)
{
    AngularMotor->Preupdate(dt, updateRate);
}

void BEPUik::RevoluteControl::UpdateJacobiansAndVelocityBias()
{
    AngularMotor->UpdateJacobiansAndVelocityBias();
}

void BEPUik::RevoluteControl::ComputeEffectiveMass()
{
    AngularMotor->ComputeEffectiveMass();
}

void BEPUik::RevoluteControl::WarmStart()
{
    AngularMotor->WarmStart();
}

void BEPUik::RevoluteControl::SolveVelocityIteration()
{
    AngularMotor->SolveVelocityIteration();
}

void BEPUik::RevoluteControl::ClearAccumulatedImpulses()
{
    AngularMotor->ClearAccumulatedImpulses();
}

float BEPUik::RevoluteControl::GetMaximumForce() const { return AngularMotor->MaximumForce; }
void BEPUik::RevoluteControl::SetMaximumForce(float value)
{
    AngularMotor->MaximumForce = value;
}
