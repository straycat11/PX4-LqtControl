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
 * @file LqtControlMath.hpp
 *
 * Simple functions for vector manipulation that do not fit into matrix lib.
 * These functions are specific for controls.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>

namespace LqtControlMath
{
/**
 * Creates attitude setpoint from to-go quaternion.
 * @param toGoQuaternion yaw incorporated to-go quaternion
 * @param att_sp attitude setpoint to fill
 */
void toGoToAttitude(matrix::Quatf &to_qo_quaternion, matrix::Vector3f angular_velocity, matrix::Vector3f &control_torques);

/**
 * Adds e.g. feed-forward to the setpoint making sure existing or added NANs have no influence on control.
 * This function is udeful to support all the different setpoint combinations of position, velocity, acceleration with NAN representing an uncommitted value.
 * @param setpoint existing possibly NAN setpoint to add to
 * @param addition value/NAN to add to the setpoint
 */
void addIfNotNan(float &setpoint, const float addition);

/**
 * _addIfNotNan for Vector3f treating each element individually
 * @see _addIfNotNan
 */
void addIfNotNanVector3f(matrix::Vector3f &setpoint, const matrix::Vector3f &addition);

/**
 * Overwrites elements of a Vector3f which are NaN with zero
 * @param vector possibly containing NAN elements
 */
void setZeroIfNanVector3f(matrix::Vector3f &vector);
}
