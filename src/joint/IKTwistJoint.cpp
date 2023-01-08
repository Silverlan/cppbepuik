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

#include "bepuik/joint/IKTwistJoint.hpp"

BEPUik::Vector3 BEPUik::IKTwistJoint::GetAxisA() const { return quaternion::Transform(LocalAxisA, m_connectionA->Orientation); }
void BEPUik::IKTwistJoint::SetAxisA(const Vector3 &value) { LocalAxisA = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionA->Orientation)); }

BEPUik::Vector3 BEPUik::IKTwistJoint::GetAxisB() const { return quaternion::Transform(LocalAxisB, m_connectionB->Orientation); }
void BEPUik::IKTwistJoint::SetAxisB(const Vector3 &value) { LocalAxisB = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionB->Orientation)); }

BEPUik::Vector3 BEPUik::IKTwistJoint::GetMeasurementAxisA() const { return quaternion::Transform(LocalMeasurementAxisA, m_connectionA->Orientation); }
void BEPUik::IKTwistJoint::SetMeasurementAxisA(const Vector3 &value) { LocalMeasurementAxisA = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionA->Orientation)); }

BEPUik::Vector3 BEPUik::IKTwistJoint::GetMeasurementAxisB() const { return quaternion::Transform(LocalMeasurementAxisB, m_connectionB->Orientation); }
void BEPUik::IKTwistJoint::SetMeasurementAxisB(const Vector3 &value) { LocalMeasurementAxisB = quaternion::Transform(value, BEPUik::quaternion::Conjugate(m_connectionB->Orientation)); }

void BEPUik::IKTwistJoint::ComputeMeasurementAxes()
{
    Vector3 axisA, axisB;
    axisA = quaternion::Transform(LocalAxisA, m_connectionA->Orientation);
    axisB = quaternion::Transform(LocalAxisB, m_connectionB->Orientation);
    //Pick an axis perpendicular to axisA to use as the measurement axis.
    Vector3 worldMeasurementAxisA;
    worldMeasurementAxisA = vector3::Cross(vector3::Up, axisA);
    float lengthSquared = vector3::LengthSqr(worldMeasurementAxisA);
    if (lengthSquared > Epsilon)
    {
        worldMeasurementAxisA = vector3::Divide(worldMeasurementAxisA, (float)std::sqrt(lengthSquared));
    }
    else
    {
        //Oops! It was parallel to the up vector. Just try again with the right vector.
        worldMeasurementAxisA = vector3::Cross(vector3::Right, axisA);
        vector3::Normalize(worldMeasurementAxisA);
    }
    //Attach the measurement axis to entity B.
    //'Push' A's axis onto B by taking into account the swing transform.
    Quaternion alignmentRotation;
    alignmentRotation = quaternion::GetQuaternionBetweenNormalizedVectors(axisA, axisB);
    Vector3 worldMeasurementAxisB;
    worldMeasurementAxisB = quaternion::Transform(worldMeasurementAxisA, alignmentRotation);
    //Plop them on!
    SetMeasurementAxisA(worldMeasurementAxisA);
    SetMeasurementAxisB(worldMeasurementAxisB);

}

BEPUik::IKTwistJoint::IKTwistJoint(Bone &connectionA, Bone &connectionB, const Vector3 &axisA, const Vector3 &axisB)
    : IKJoint(connectionA, connectionB)
{
    SetAxisA(axisA);
    SetAxisB(axisB);

    ComputeMeasurementAxes();
}

void BEPUik::IKTwistJoint::UpdateJacobiansAndVelocityBias()
{

    //This constraint doesn't consider linear motion.
    linearJacobianA = matrix::Create();
	linearJacobianB = matrix::Create();

    //Compute the world axes.
    Vector3 axisA, axisB;
    axisA = quaternion::Transform(LocalAxisA, m_connectionA->Orientation);
    axisB = quaternion::Transform(LocalAxisB, m_connectionB->Orientation);

    Vector3 twistMeasureAxisA, twistMeasureAxisB;
    twistMeasureAxisA = quaternion::Transform(LocalMeasurementAxisA, m_connectionA->Orientation);
    twistMeasureAxisB = quaternion::Transform(LocalMeasurementAxisB, m_connectionB->Orientation);

    //Compute the shortest rotation to bring axisB into alignment with axisA.
    Quaternion alignmentRotation;
    alignmentRotation = quaternion::GetQuaternionBetweenNormalizedVectors(axisB, axisA);

    //Transform the measurement axis on B by the alignment quaternion.
    twistMeasureAxisB = quaternion::Transform(twistMeasureAxisB, alignmentRotation);

    //We can now compare the angle between the twist axes.
    float error;
    error = vector3::Dot(twistMeasureAxisA, twistMeasureAxisB);
    error = (float)std::acos(std::clamp(error, -1.f, 1.f));
    Vector3 cross;
    cross = vector3::Cross(twistMeasureAxisA, twistMeasureAxisB);
    float dot;
    dot = vector3::Dot(cross, axisA);
    if (dot < 0)
        error = -error;

    //Compute the bias based upon the error.
    velocityBias = Vector3(errorCorrectionFactor * error, 0, 0);

    //We can't just use the axes directly as jacobians. Consider 'cranking' one object around the other.
    Vector3 jacobian;
    jacobian = vector3::Add(axisA, axisB);
    float lengthSquared = vector3::LengthSqr(jacobian);
    if (lengthSquared > Epsilon)
    {
        jacobian = vector3::Divide(jacobian, (float)std::sqrt(lengthSquared));
    }
    else
    {
        //The constraint is in an invalid configuration. Just ignore it.
        jacobian = vector3::Create();
    }

	angularJacobianA = matrix::Create();
	angularJacobianA[0][0] = jacobian.x;
	angularJacobianA[0][1] = jacobian.y;
	angularJacobianA[0][2] = jacobian.z;
	angularJacobianB = matrix::Create();
	angularJacobianB[0][0] = -jacobian.x;
	angularJacobianB[0][1] = -jacobian.y;
	angularJacobianB[0][2] = -jacobian.z;




}
