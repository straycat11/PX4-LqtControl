/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "LqtControl.h"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include "LqtPositionControl/LqtControlMath.hpp"

using namespace matrix;

LqtControl::LqtControl() :
	SuperBlock(nullptr, "MLC"),ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_update(true);
}

LqtControl::~LqtControl()
{
	perf_free(_cycle_perf);
}

bool LqtControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	ScheduleNow();

	return true;
}

void LqtControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		int num_changed = 0;

		if (num_changed > 0) {
			param_notify_changes();
		}

		_control.setPositionGains(Vector3f(_param_mlc_xy_p.get(), _param_mlc_xy_p.get(), _param_mlc_z_p.get()));
		_control.setVelocityGains(diag(Vector3f(-_param_mlc_xy_k.get(), -_param_mlc_xy_k.get(), -_param_mlc_z_k.get())),
		diag(Vector3f(-_param_mlc_xy_k_f.get(), -_param_mlc_xy_k_f.get(), -_param_mlc_z_k_f.get())),
		diag(Vector3f(_param_mlc_xy_k_z.get(), _param_mlc_xy_k_z.get(), _param_mlc_z_k_z.get())));
		_control.setLyapunovParams(
			diag(Vector3f(_param_mlc_inertia_xx.get(),_param_mlc_inertia_yy.get(),			_param_mlc_inertia_zz.get())),
			_param_mlc_arm_length.get(),
			diag(Vector3f(_param_mlc_lyp_m_xy.get(),_param_mlc_lyp_m_xy.get(),_param_mlc_lyp_m_z.get())),
			diag(Vector3f(_param_mlc_lyp_n_xy.get(),_param_mlc_lyp_n_xy.get(),_param_mlc_lyp_n_z.get())));
	}
}

PositionControlStates LqtControl::set_vehicle_states(const vehicle_local_position_s
		&vehicle_local_position, const vehicle_attitude_s &vehicle_attitude, const vehicle_angular_velocity_s &vehicle_angular_velocity)
{
	PositionControlStates states;

	const Vector2f position_xy(vehicle_local_position.x, vehicle_local_position.y);

	// only set position states if valid and finite
	if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy;

	} else {
		states.position(0) = states.position(1) = NAN;
	}

	if (PX4_ISFINITE(vehicle_local_position.z) && vehicle_local_position.z_valid) {
		states.position(2) = vehicle_local_position.z;

	} else {
		states.position(2) = NAN;
	}

	const Vector2f velocity_xy(vehicle_local_position.vx, vehicle_local_position.vy);

	if (vehicle_local_position.v_xy_valid && velocity_xy.isAllFinite()) {
		states.velocity.xy() = velocity_xy;

	} else {
		states.velocity(0) = states.velocity(1) = NAN;
	}

	if (PX4_ISFINITE(vehicle_local_position.vz) && vehicle_local_position.v_z_valid) {
		states.velocity(2) = vehicle_local_position.vz;

	} else {
		states.velocity(2) = NAN;
	}

	states.yaw = vehicle_local_position.heading;
	states.q = Quatf(vehicle_attitude.q);
	states.angular_velocity = Vector3f(vehicle_angular_velocity.xyz);

	return states;
}

void LqtControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	vehicle_local_position_s vehicle_local_position;
	vehicle_attitude_s vehicle_attitude;
	vehicle_angular_velocity_s vehicle_angular_velocity;

	if (_local_pos_sub.update(&vehicle_local_position)) {

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// clear existing setpoint when controller is no longer active
					_setpoint = LqtPositionControl::empty_trajectory_setpoint;
				}
			}
		}
		_vehicle_attitude_sub.update(&vehicle_attitude);
		_vehicle_angular_velocity_sub.update(&vehicle_angular_velocity);

		PositionControlStates states{set_vehicle_states(vehicle_local_position, vehicle_attitude, vehicle_angular_velocity)};

		_trajectory_setpoint_sub.update(&_setpoint);

		adjustSetpointForEKFResets(vehicle_local_position, _setpoint);

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			// set failsafe setpoint if there hasn't been a new
			// trajectory setpoint since position control started
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (vehicle_local_position.timestamp_sample > _time_position_control_enabled)) {

				_setpoint = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states);
			}
		}

		const float dt = math::constrain(((vehicle_local_position.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = vehicle_local_position.timestamp_sample;
		float addition = _setpoint.yawspeed * dt;

		_man_yaw = PX4_ISFINITE(addition) ? wrap_pi(_man_yaw + _setpoint.yawspeed * dt):_man_yaw;

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {

			_control.setInputSetpoint(_setpoint,_man_yaw);

			_control.setState(states);

			// Run position control
			if (!_control.update()) {

				_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states),states.yaw);
				_control.update();


			}

			// Publish internal position control setpoints lqt
			vehicle_local_position_setpoint_lqt_s local_pos_sp_lqt{};
			_control.getLocalPositionSetpointLqt(local_pos_sp_lqt);
			local_pos_sp_lqt.timestamp = hrt_absolute_time();
			_local_pos_sp_lqt_pub.publish(local_pos_sp_lqt);

		} else {

		}
	}

	perf_end(_cycle_perf);
}

trajectory_setpoint_s LqtControl::generateFailsafeSetpoint(const hrt_abstime &now,
		const PositionControlStates &states)
{
	trajectory_setpoint_s failsafe_setpoint = LqtPositionControl::empty_trajectory_setpoint;
	failsafe_setpoint.timestamp = now;

	return failsafe_setpoint;
}

void LqtControl::adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
		trajectory_setpoint_s &setpoint)
{
	if ((setpoint.timestamp != 0) && (setpoint.timestamp < vehicle_local_position.timestamp)) {
		if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
			setpoint.velocity[0] += vehicle_local_position.delta_vxy[0];
			setpoint.velocity[1] += vehicle_local_position.delta_vxy[1];
		}

		if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
			setpoint.velocity[2] += vehicle_local_position.delta_vz;
		}

		if (vehicle_local_position.xy_reset_counter != _xy_reset_counter) {
			setpoint.position[0] += vehicle_local_position.delta_xy[0];
			setpoint.position[1] += vehicle_local_position.delta_xy[1];
		}

		if (vehicle_local_position.z_reset_counter != _z_reset_counter) {
			setpoint.position[2] += vehicle_local_position.delta_z;
		}

		if (vehicle_local_position.heading_reset_counter != _heading_reset_counter) {
			setpoint.yaw = wrap_pi(setpoint.yaw + vehicle_local_position.delta_heading);
		}
	}

	// save latest reset counters
	_vxy_reset_counter = vehicle_local_position.vxy_reset_counter;
	_vz_reset_counter = vehicle_local_position.vz_reset_counter;
	_xy_reset_counter = vehicle_local_position.xy_reset_counter;
	_z_reset_counter = vehicle_local_position.z_reset_counter;
	_heading_reset_counter = vehicle_local_position.heading_reset_counter;
}

int LqtControl::task_spawn(int argc, char *argv[])
{
	LqtControl *instance = new LqtControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int LqtControl::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}

int LqtControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_lqt_control_main(int argc, char *argv[])
{
	return LqtControl::main(argc, argv);
}