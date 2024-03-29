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

#include "bepuik/control/Control.hpp"
#include <memory>
#include <optional>

namespace BEPUik
{
	class SingleBoneLinearMotor;
	/// <summary>
	/// Constrains an individual bone in an attempt to reach some position goal.
	/// </summary>
	class DragControl : public Control
	{
	public:
		virtual ~DragControl() override;
		/// <summary>
		/// Gets or sets the controlled bone.
		/// </summary>
		Bone* GetTargetBone() override;
		void SetTargetBone(Bone* value) override;

		/// <summary>
		/// Gets or sets the linear motor used by the control.
		/// </summary>
		std::unique_ptr<SingleBoneLinearMotor> LinearMotor;
		SingleBoneLinearMotor* GetLinearMotor();
		void SetLinearMotor(std::unique_ptr<SingleBoneLinearMotor> value);

		DragControl();

		virtual void Preupdate(float dt, float updateRate) override;

		virtual void UpdateJacobiansAndVelocityBias() override;

		virtual void ComputeEffectiveMass() override;

		virtual void WarmStart() override;

		virtual void SolveVelocityIteration() override;

		virtual void ClearAccumulatedImpulses() override;

		virtual float GetMaximumForce() const override;
		virtual void SetMaximumForce(float value) override;

		virtual float GetRigidity() const override;
		virtual void SetRigidity(float rigidity) override;
	};

	/// <summary>
	/// Constrains an individual bone in an attempt to reach some position goal.
	/// The control orientation overwrites the bone orientation, but does not affect the actual ik simulation.
	/// </summary>
	class OrientedDragControl : public DragControl
	{
	public:
		virtual void ClearAccumulatedImpulses() override;

		void SetTargetOrientation(const Quaternion& orientation);
		const Quaternion& GetTargetOrientation() const;
	private:
		Quaternion m_targetOrientation{ 1.f,0.f,0.f,0.f };
	};
}