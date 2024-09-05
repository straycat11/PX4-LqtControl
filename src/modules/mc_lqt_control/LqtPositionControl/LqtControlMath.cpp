/****************************************************************************
 *
 *   Copyright (C) 2018 - 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file LqtControlMath.cpp
 */

#include "LqtControlMath.hpp"
#include <px4_platform_common/defines.h>
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

namespace LqtControlMath
{
void toGoToAttitude(matrix::Quatf &to_qo_quaternion, Vector3f angular_velocity, Vector3f &control_torques)
{
	Vector3f inertia =  Vector3f(0.029125f,0.029125f,0.055225f);
	float arm_length = 0.0569f; // 0.13^2 + 0.2^2; from ../Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja
	Vector3f lyapunov_s =  Vector3f((inertia(0)-inertia(2))*angular_velocity(1)*angular_velocity(2)/inertia(0),
				(inertia(2)-inertia(0))*angular_velocity(0)*angular_velocity(2)/inertia(1),
				(inertia(0)-inertia(1))*angular_velocity(0)*angular_velocity(1)/inertia(2));
	Matrix3f lyapunov_g = diag(Vector3f(arm_length/inertia(0),arm_length/inertia(1),1.f/inertia(2)));
	Matrix3f lyapunov_m = diag(Vector3f(1.6,1.6,1.6));
	Matrix3f lyapunov_n = diag(Vector3f(0.15,0.15,0.15));
	control_torques = inv(lyapunov_g)*(inv(diag(inertia))*(lyapunov_m*to_qo_quaternion.imag()-lyapunov_n*angular_velocity)-lyapunov_s);
}

void addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		// No NAN, add to the setpoint
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		// Setpoint NAN, take addition
		setpoint = addition;
	}

	// Addition is NAN or both are NAN, nothing to do
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

void setZeroIfNanVector3f(Vector3f &vector)
{
	// Adding zero vector overwrites elements that are NaN with zero
	addIfNotNanVector3f(vector, Vector3f());
}

} // LqtControlMath
