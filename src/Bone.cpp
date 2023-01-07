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

#include "bepuik/Bone.hpp"

float BEPUik::Bone::GetMass() const {return 1 / inverseMass;}
void BEPUik::Bone::SetMass(float value)
{
	//Long chains could produce exceptionally small values.
	//Attempting to invert them would result in NaNs.
	//Clamp the lowest mass to 1e-7f.
	if (value > Epsilon)
		inverseMass = 1.f / value;
	else
		inverseMass = 1e7f;
	ComputeLocalInertiaTensor();
}

const std::vector<BEPUik::IKJoint*> &BEPUik::Bone::GetJoints() {return joints;}

bool BEPUik::Bone::GetPinned() const {return Pinned;}
void BEPUik::Bone::SetPinned(bool value) {Pinned = value;}

bool BEPUik::Bone::IsActive() const {return Active;}
void BEPUik::Bone::SetActive(bool value) {Active = value;}

float BEPUik::Bone::GetRadius() const {return radius;}
void BEPUik::Bone::SetRadius(float value)
{
    radius = value;
    ComputeLocalInertiaTensor();
}

float BEPUik::Bone::GetHalfHeight() const {return HalfHeight;}
void BEPUik::Bone::SetHalfHeight(float value)
{
	HalfHeight = value;
	ComputeLocalInertiaTensor();
}

float BEPUik::Bone::GetHeight() const {return halfHeight * 2;}
void BEPUik::Bone::SetHeight(float value)
{
    halfHeight = value / 2;
    ComputeLocalInertiaTensor();
}

BEPUik::Bone::Bone(const Vector3 &position, const Quaternion &orientation, float radius, float height, float mass)
    :Bone(position, orientation, radius, height)
{
    SetMass(mass);
}

BEPUik::Bone::Bone(const Vector3 &position, const Quaternion &orientation, float radius, float height)
{
    SetMass(1.f);
    Position = position;
    Orientation = orientation;
    SetRadius(radius);
    SetHeight(height);
}


void BEPUik::Bone::ComputeLocalInertiaTensor()
{
    auto localInertiaTensor = matrix::Create();
    auto multiplier = GetMass() * InertiaTensorScaling;
    float diagValue = (.0833333333f * GetHeight() * GetHeight() + .25f * GetRadius() * GetRadius()) * multiplier;
    localInertiaTensor[0][0] = diagValue;
    localInertiaTensor[1][1] = .5f * GetRadius() * GetRadius() * multiplier;
    localInertiaTensor[2][2] = diagValue;
    localInertiaTensorInverse = matrix::Invert(localInertiaTensor);
}

void BEPUik::Bone::UpdateInertiaTensor()
{
    //This is separate from the position update because the orientation can change outside of our iteration loop, so this has to run first.
    //Iworld^-1 = RT * Ilocal^1 * R
    Matrix3x3 orientationMatrix;
    orientationMatrix = matrix::CreateFromQuaternion(Orientation);
    inertiaTensorInverse = matrix::MultiplyTransposed(orientationMatrix, localInertiaTensorInverse);
    inertiaTensorInverse = matrix::Multiply(inertiaTensorInverse, orientationMatrix);
}

void BEPUik::Bone::UpdatePosition()
{
    //Update the position based on the linear velocity.
    Position = vector3::Add(Position, linearVelocity);

    //Update the orientation based on the angular velocity.
    Vector3 increment;
    increment = vector3::Multiply(angularVelocity, .5f);
    Quaternion multiplier = Quaternion(0.f, increment.x, increment.y, increment.z);
    multiplier = BEPUik::quaternion::Multiply(multiplier, Orientation);
    Orientation = quaternion::Add(Orientation, multiplier);
    quaternion::Normalize(Orientation);

    //Eliminate any latent velocity in the bone to prevent unwanted simulation feedback.
    //This is the only thing conceptually separating this "IK" solver from the regular dynamics loop in BEPUphysics.
    //(Well, that and the whole lack of collision detection...)
    linearVelocity = vector3::Create();
    angularVelocity = vector3::Create();

    //Note: Unlike a regular dynamics simulation, we do not include any 'dt' parameter in the above integration.
    //Setting the velocity to 0 every update means that no more than a single iteration's worth of velocity accumulates.
    //Since the softness of constraints already varies with the time step and bones never accelerate for more than one frame,
    //scaling the velocity for position integration actually turns generally worse.
    //This is not a rigorously justifiable approach, but this isn't a regular dynamic simulation anyway.
}

void BEPUik::Bone::ApplyLinearImpulse(Vector3 &impulse)
{
    Vector3 velocityChange;
    velocityChange = vector3::Multiply(impulse, inverseMass);
    linearVelocity = vector3::Add(linearVelocity, velocityChange);
}

void BEPUik::Bone::ApplyAngularImpulse(Vector3 &impulse)
{
    Vector3 velocityChange;
    velocityChange = matrix::Transform(impulse, inertiaTensorInverse);
    angularVelocity = vector3::Add(velocityChange, angularVelocity);
}
