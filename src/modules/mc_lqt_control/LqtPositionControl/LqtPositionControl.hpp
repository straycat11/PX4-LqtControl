/**
 * @file LqtPositionControl.hpp
 *
 * Lqt position controller.
 */
#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <matrix/matrix/Dcm.hpp>
#include <matrix/matrix/Euler.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint_lqt.h>

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
	matrix::Quatf q;
	matrix::Vector3f angular_velocity;
};

struct DebugVars {
	matrix::Quatf s;
	matrix::Quatf y;
	matrix::Quatf toGo;
	matrix::Vector3f acc_sp;
	matrix::Vector3f acc_sp_body;
	float yaw;
};

class LqtPositionControl
{
public:

	LqtPositionControl() = default;
	~LqtPositionControl() = default;

	/**
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }

	/**
	 * Set the lqt velocity control gains
	 */
	void setVelocityGains();

	/**
	 * Pass the current vehicle state to the controller
	 * @param PositionControlStates structure
	 */
	void setState(const PositionControlStates &states);

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint setpoints including feed-forwards to execute in update()
	 */
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt);


	/**
	 * Get the controllers output local position setpoint lqtπ
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpointLqt(vehicle_local_position_setpoint_lqt_s &local_position_setpoint_lqt) const;

	/**
	 * Get debug
	 * @param debug debug struct to fill up
	 */
	void getDebug(DebugVars &debug) const;

	/**
	 * All setpoints are set to NAN (uncontrolled). Timestampt zero.
	 */
	static const trajectory_setpoint_s empty_trajectory_setpoint;


	/**
	 * Get to-go quaternion value
	 */
	float getToGoQuaternionElement(int index) {return _toGoQuaternion(index);}

private:
	bool _inputValid();

	void _positionControl(); ///< Position proportional control
	void _velocityControl(); ///< Velocity control
	void _toGoAccelerationControl(); ///< Acceleration setpoint processing

	// Gains
	matrix::Vector3f _gain_pos_p; ///< Position control proportional gain
	matrix::Matrix3f _gain_vel_K; ///< Velocity lqt control K
	matrix::Matrix3f _gain_vel_K_z; ///< Velocity lqt control K_z
	matrix::Matrix3f _gain_vel_K_f; ///< Velocity lqt control K_f

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	float _yaw{}; /**< current heading */
	matrix::Quatf _q{}; /**< current attitude */
	matrix::Vector3f _ang_vel{}; /**< angular velocity */

	// Setpoints
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */

	matrix::Quatf _toGoQuaternion;

	// Debug
	matrix::Quatf _debug_s;
	matrix::Quatf _debug_y;
	matrix::Vector3f _debug_acc_sp_body;
	matrix::Vector3f _acc_sp_lqt;
	matrix::Vector3f _torque_sp_lqt;
	matrix::Vector3f _vel_sp_K_debug;
	matrix::Vector3f _vel_sp_K_f_debug;
	matrix::Vector3f _vel_sp_K_z_debug;
	float _thr_sp_lqt;
	float _debug_yaw;

};
