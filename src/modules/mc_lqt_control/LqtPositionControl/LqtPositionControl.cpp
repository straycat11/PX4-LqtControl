#include "LqtPositionControl.hpp"
#include "LqtControlMath.hpp"
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s LqtPositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void LqtPositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_q = states.q;
	_ang_vel = states.angular_velocity;
}

void LqtPositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint, float manualYaw)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	_man_yaw = manualYaw;
}

bool LqtPositionControl::update()
{
	bool valid = _inputValid();

	if (valid) {
		_positionControl();
		_velocityControl();

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

		_toGoAccelerationControl();
	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid;//  && _thr_sp.isAllFinite() && _acc_sp_lqt.isAllFinite();
}

void LqtPositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	LqtControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	LqtControlMath::setZeroIfNanVector3f(vel_sp_position);
}

void LqtPositionControl::_velocityControl()
{
	Vector3f vel_sp_K = _gain_vel_K * _vel;
	Vector3f vel_sp_K_z = _gain_vel_K_z * _vel_sp;
	Vector3f vel_sp_K_f = _gain_vel_K_f * Vector3f(0.f,0.f,CONSTANTS_ONE_G);
	_acc_sp_lqt = vel_sp_K + vel_sp_K_z + vel_sp_K_f;
	_thr_sp_lqt =  math::constrain(-0.0370f*_acc_sp_lqt.norm(),-1.f,0.f);
}


void LqtPositionControl::_toGoAccelerationControl()
{
	Dcmf ned2body(_q.inversed());
	_acc_sp_lqt(2) = math::constrain(_acc_sp_lqt(2),-1.f,0.f);
	Vector3f acc_sp_body_normalized = ned2body * _acc_sp_lqt.normalized();
	float s_4 = sqrtf(0.5f * (1.f - acc_sp_body_normalized(2)));
	Vector3f s_imag = (Vector3f(0.f, 0.f, -1.f).cross(acc_sp_body_normalized))/(2.f*s_4);
	Quatf s = Quatf(s_4,s_imag(0),s_imag(1),s_imag(2));

	float yaw_sp = PX4_ISFINITE(_man_yaw) ? _man_yaw : _yaw;
	float delta_yaw = yaw_sp - Eulerf(_q).psi();
	Vector3f y_imag = Vector3f(0.f,0.f,1.f*sinf(delta_yaw/2.f));
	float y_4 = cosf(delta_yaw/2.f);

	_toGoQuaternion = s * Quatf(y_4, y_imag(0),y_imag(1),y_imag(2));

	LqtControlMath::toGoToAttitude(_toGoQuaternion,_ang_vel,_torque_sp_lqt,_vehicle_inertia,_lyapunov_m,_lyapunov_n,_vehicle_arm_length);
}

bool LqtPositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)));
	}

	// Yaw axis needs to have a setpoint
	valid = valid && PX4_ISFINITE(_man_yaw);

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i));
		}

		if (PX4_ISFINITE(_man_yaw)) {
			valid = valid && PX4_ISFINITE(_yaw);
		}
	}

	// Check quaternion validity
	for(int i = 0; i <= 3; i++){
		valid = valid && PX4_ISFINITE(_q(i));
	}

	return valid;
}

void LqtPositionControl::getLocalPositionSetpointLqt(vehicle_local_position_setpoint_lqt_s &local_position_setpoint_lqt) const
{
	local_position_setpoint_lqt.x = _pos_sp(0);
	local_position_setpoint_lqt.y = _pos_sp(1);
	local_position_setpoint_lqt.z = _pos_sp(2);
	local_position_setpoint_lqt.yaw = _man_yaw;
	local_position_setpoint_lqt.yawspeed = _yawspeed_sp;
	local_position_setpoint_lqt.vx = _vel_sp(0);
	local_position_setpoint_lqt.vy = _vel_sp(1);
	local_position_setpoint_lqt.vz = _vel_sp(2);
	_acc_sp_lqt.copyTo(local_position_setpoint_lqt.acceleration);
	_torque_sp_lqt.copyTo(local_position_setpoint_lqt.torque);
	local_position_setpoint_lqt.heave = _thr_sp_lqt;
	_toGoQuaternion.copyTo(local_position_setpoint_lqt.togo);
}