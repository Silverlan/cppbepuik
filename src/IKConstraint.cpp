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

#include "bepuik/IKConstraint.hpp"

float BEPUik::IKConstraint::GetRigidity() const {return Rigidity;}
void BEPUik::IKConstraint::SetRigidity(float value)
{
    if (value <= 0)
        throw std::invalid_argument("Rigidity must be positive.");
    Rigidity = value;
}

float BEPUik::IKConstraint::GetMaximumForce() const {return MaximumForce;}
void BEPUik::IKConstraint::SetMaximumForce(float value)
{
    MaximumForce = std::max(value,0.f);
}

void BEPUik::IKConstraint::Preupdate(float dt, float updateRate)
{
    float stiffness = StiffnessOverDamping * Rigidity;
    float damping = Rigidity;
    float multiplier = 1 / (dt * stiffness + damping);
    errorCorrectionFactor = stiffness * multiplier;
    softness = updateRate * multiplier;
    MaximumImpulse = MaximumForce * dt;
    MaximumImpulseSquared = std::min(std::numeric_limits<float>::max(), MaximumImpulse * MaximumImpulse);

}
