/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:ß
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
 * Proportional gain for vertical position error (lqt)
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0.1
 * @max 1.5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_Z_P, 1.f);

/**
 * Proportional gain for horizontal position error (lqt)
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_XY_P, 0.95f);

/**
 * Proportional gain for vertical position error (lqt)
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0.1
 * @max 4.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_Z_K, 3.62f);

/**
 * Proportional gain for horizontal position error (lqt)
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0
 * @max 1
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_XY_K, 0.249f);

/**
 * Proportional gain for vertical position error (lqt)
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0.1
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_Z_K_F, 1.35f);

/**
 * Proportional gain for horizontal position error (lqt)
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_XY_K_F, 1.39f);

/**
 * Proportional gain for vertical position error (lqt)
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0.1
 * @max 2.5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_Z_K_Z, 1.99f);

/**
 * Proportional gain for horizontal position error (lqt)
 *
 * Defined as corrective velocity in m/s per m position error
 *
 * @min 0
 * @max 2
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_XY_K_Z, 0.669f);

/**
 * Numerical velocity derivative low pass cutoff frequency (lqt)
 *
 * @unit Hz
 * @min 0
 * @max 10
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_VELD_LP, 5.0f);

/**
 * Arm length of vehicle in meters
 *
 * @unit m
 * @min 0
 * @max 0.5
 * @decimal 4
 * @increment 0.0001
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_ARM_LENGTH, 0.0569f);

/**
 * X component of vehicle moment of inertia.
 *
 * @min 0
 * @max 0.1
 * @decimal 6
 * @increment 0.000005
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_INERTIA_XX, 0.029125f);

/**
 * Y component of vehicle moment of inertia.
 *
 * @min 0
 * @max 0.1
 * @decimal 6
 * @increment 0.000005
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_INERTIA_YY, 0.029125f);

/**
 * Z component of vehicle moment of inertia.
 *
 * @min 0
 * @max 0.1
 * @decimal 6
 * @increment 0.000005
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_INERTIA_ZZ, 0.055225f);

/**
 * Lyapunov gain for first and second components of matrix M
 *
 *
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_LYP_M_XY, 1.6f);

/**
 * Lyapunov gain for third component of matrix M
 *
 *
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_LYP_M_Z, 1.6f);

/**
 * Lyapunov gain for first and second components of matrix N
 *
 *
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_LYP_N_XY, 0.15f);

/**
 * Lyapunov gain for third component of matrix N
 *
 *
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Lqt Position Control
 */
PARAM_DEFINE_FLOAT(MLC_LYP_N_Z, 0.15f);
