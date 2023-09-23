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

#include "bepuik/limit/IKLimit.hpp"

namespace BEPUik
{
	/// <summary>
	/// Prevents two bones from rotating beyond a certain angle away from each other as measured by attaching an axis to each connected bone.
	/// </summary>
	class IKEllipseSwingLimit : public IKLimit
	{
	public:
		/// <summary>
		/// Gets or sets the axis attached to ConnectionA in its local space.
		/// </summary>
		Vector3 LocalAxisA;
		/// <summary>
		/// Gets or sets the axis attached to ConnectionB in its local space.
		/// </summary>
		Vector3 LocalAxisB;

		Vector3 LocalXAxis;
		Vector3 LocalAxisBRelToA;

		/// <summary>
		/// Gets or sets the axis attached to ConnectionA in world space.
		/// </summary>
		Vector3 GetAxisA() const;
		void SetAxisA(const Vector3& value);

		/// <summary>
		///  Gets or sets the axis attached to ConnectionB in world space.
		/// </summary>
		Vector3 GetAxisB() const;
		void SetAxisB(const Vector3& value);

		/// <summary>
		/// Gets the maximum angle between the two axes allowed by the constraint.
		/// </summary>
		float GetMaximumAngleX();

		/// <summary>
		/// Sets the maximum angle between the two axes allowed by the constraint.
		/// </summary>
		void SetMaximumAngleX(float angle);

		/// <summary>
		/// Gets the maximum angle between the two axes allowed by the constraint.
		/// </summary>
		float GetMaximumAngleY();

		/// <summary>
		/// Sets the maximum angle between the two axes allowed by the constraint.
		/// </summary>
		void SetMaximumAngleY(float angle);

		void SetupJointTransforms(const Vector3 &twistAxis);

		/// <summary>
		/// Gets the axis attached to connection B in world space.
		/// </summary>
		Vector3 GetXAxis() const;

		/// <summary>
		/// Sets the axis attached to connection B in world space.
		/// </summary>
		void SetXAxis(const Vector3& axis);

		/// <summary>
		/// Builds a new swing limit. Prevents two bones from rotating beyond a certain angle away from each other as measured by attaching an axis to each connected bone.
		/// </summary>
		/// <param name="connectionA">First connection of the limit.</param>
		/// <param name="connectionB">Second connection of the limit.</param>
		/// <param name="axisA">Axis attached to connectionA in world space.</param>
		/// <param name="axisB">Axis attached to connectionB in world space.</param>
		/// <param name="maximumAngle">Maximum angle allowed between connectionA's axis and connectionB's axis.</param>
		IKEllipseSwingLimit(Bone& connectionA, Bone& connectionB, const Vector3& axisA, const Vector3& axisB, float maximumAngleX, float maximumAngleY);

		IKEllipseSwingLimit(Bone& connectionA, Bone& connectionB, const Vector3& axisA, const Vector3& axisB, const Vector3 &axisBRight, const Vector3 &axisBUp, float maximumAngleX, float maximumAngleY);

		virtual void UpdateJacobiansAndVelocityBias() override;

	private:
		float maximumAngleX;
		float maximumAngleY;
		Vector3 xAxis;
		Vector3 yAxis;
	};
}
