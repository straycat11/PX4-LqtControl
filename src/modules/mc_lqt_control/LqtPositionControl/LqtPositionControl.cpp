#include "LqtPositionControl.hpp"
#include "ControlMath.hpp"
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s LqtPositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void LqtPositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;

	_gain_vel_K = diag(Vector3f(0.2*-1.245,0.2*-1.245,4*-0.905));
	_gain_vel_K_f = diag(Vector3f(-1.3885f,-1.3885f,-1.3507f));
	_gain_vel_K_z = diag(Vector3f(0.5*1.3379,0.5*1.3379,2*0.995));
	// _gain_vel_K = diag(Vector3f(-0.1f*3.245f,-0.1f*3.245f,-0.905f));
	// _gain_vel_K_f = diag(Vector3f(-0.015f,-0.015f,-0.15f));
	// _gain_vel_K_z = diag(Vector3f(3.338f,3.338f,0.955f));
}

void LqtPositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void LqtPositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void LqtPositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void LqtPositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp_lqt(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp_lqt(2);
}

void LqtPositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
	_q = states.q;
	_ang_vel = states.angular_velocity;
}

void LqtPositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool LqtPositionControl::update(const float dt)
{
	bool valid = _inputValid();

	if (valid) {
		_positionControl();
		_velocityControl(dt);

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid;//  && _thr_sp.isAllFinite() && _acc_sp_lqt.isAllFinite();
}

void LqtPositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void LqtPositionControl::_velocityControl(const float dt)
{
	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	Vector3f vel_sp_K = _gain_vel_K * _vel;
	Vector3f vel_sp_K_z = _gain_vel_K_z * _vel_sp;
	Vector3f vel_sp_K_f = _gain_vel_K_f * Vector3f(0.f,0.f,CONSTANTS_ONE_G);
	_acc_sp_lqt = vel_sp_K + vel_sp_K_z + vel_sp_K_f;
	_vel_sp_K_debug = vel_sp_K;
	_vel_sp_K_f_debug = vel_sp_K_f;
	_vel_sp_K_z_debug = vel_sp_K_z;
	_thr_sp_lqt =  math::constrain(-0.0370f*_acc_sp_lqt.norm(),-1.f,0.f);//-0.32f * 0.24f * _acc_sp_lqt.norm(); // This magic number is found by looking at what px4 outputs and what the thr_sp_lqt is without this multiplier.
	// _thr_sp_lqt = math::constrain(_thr_sp_lqt,-1.f,0.f);

	_toGoAccelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);

	// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
	// The ARW loop needs to run if the signal is saturated only.
	const Vector2f acc_sp_xy = _acc_sp.xy();
	const Vector2f acc_limited_xy = (acc_sp_xy.norm_squared() > acc_sp_xy_produced.norm_squared())
					? acc_sp_xy_produced
					: acc_sp_xy;
	vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_limited_xy);

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}


void LqtPositionControl::_toGoAccelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation

	Dcmf ned2body(_q.inversed());
	_acc_sp_lqt(2) = math::constrain(_acc_sp_lqt(2),-1.f,0.f);
	Vector3f acc_sp_body_normalized = ned2body * _acc_sp_lqt.normalized();
	float s_4 = sqrtf(0.5f * (1.f - acc_sp_body_normalized(2)));
	Vector3f s_imag = (Vector3f(0.f, 0.f, -1.f).cross(acc_sp_body_normalized))/(2.f*s_4);
	Quatf s = Quatf(s_4,s_imag(0),s_imag(1),s_imag(2));

	_debug_s = s;
	float yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;
	float delta_yaw = yaw_sp - Eulerf(_q).psi();
	Vector3f y_imag = Vector3f(0.f,0.f,1.f*sinf(delta_yaw/2.f));
	float y_4 = cosf(delta_yaw/2.f);
	_debug_y = Quatf(y_4,y_imag(0),y_imag(1),y_imag(2));
	_debug_acc_sp_body = acc_sp_body_normalized;
	_debug_yaw = y_imag(1);

	_toGoQuaternion = s * Quatf(y_4, y_imag(0),y_imag(1),y_imag(2));
	// Vector3f additionalCommand = Vector3f(0.f,0.f,0.f);
	ControlMath::toGoToAttitude(_toGoQuaternion,_ang_vel,_torque_sp_lqt);
	// ControlMath::addIfNotNanVector3f(_torque_sp_lqt, additionalCommand);
	// _torque_sp_lqt(2) = 0.f;

	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(2);
	}

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool LqtPositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void LqtPositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void LqtPositionControl::getLocalPositionSetpointLqt(vehicle_local_position_setpoint_lqt_s &local_position_setpoint_lqt) const
{
	local_position_setpoint_lqt.x = _pos_sp(0);
	local_position_setpoint_lqt.y = _pos_sp(1);
	local_position_setpoint_lqt.z = _pos_sp(2);
	local_position_setpoint_lqt.yaw = _yaw_sp;
	local_position_setpoint_lqt.yawspeed = _yawspeed_sp;
	local_position_setpoint_lqt.vx = _vel_sp(0);
	local_position_setpoint_lqt.vy = _vel_sp(1);
	local_position_setpoint_lqt.vz = _vel_sp(2);
	_acc_sp_lqt.copyTo(local_position_setpoint_lqt.acceleration);
	_torque_sp_lqt.copyTo(local_position_setpoint_lqt.torque);
	local_position_setpoint_lqt.heave = _thr_sp_lqt;
	_debug_y.copyTo(local_position_setpoint_lqt.y_togo);
	_debug_s.copyTo(local_position_setpoint_lqt.s_togo);
	_vel_sp_K_debug.copyTo(local_position_setpoint_lqt.vel_sp_k);
	_vel_sp_K_f_debug.copyTo(local_position_setpoint_lqt.vel_sp_k_f);
	_vel_sp_K_z_debug.copyTo(local_position_setpoint_lqt.vel_sp_k_z);
	_toGoQuaternion.copyTo(local_position_setpoint_lqt.togo);
}

void LqtPositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}

void LqtPositionControl::getDebug(DebugVars &debug) const
{
	debug.s = _debug_s;
	debug.y = _debug_y;
	debug.acc_sp_body = _debug_acc_sp_body;
	debug.acc_sp = _acc_sp;
	debug.yaw = _debug_yaw;
	debug.toGo = _toGoQuaternion;
}
