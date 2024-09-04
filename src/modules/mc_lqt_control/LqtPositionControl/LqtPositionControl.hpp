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
#include <uORB/topics/debug_array.h> // Debug

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
	 * Set the velocity control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set margin that is kept for horizontal control when prioritizing vertical thrust
	 * @param margin of normalized thrust that is kept for horizontal control e.g. 0.3
	 */
	void setHorizontalThrustMargin(const float margin);

	/**
	 * Set the maximum tilt angle in radians the output attitude is allowed to have
	 * @param tilt angle in radians from level orientation
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_up upwards velocity limit
	 * @param vel_down downwards velocity limit
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

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
	 * Set the integral term in xy to 0.
	 * @see _vel_int
	 */
	void resetIntegral() { _vel_int.setZero(); }

	/**
	 * If set, the tilt setpoint is computed by assuming no vertical acceleration
	 */
	void decoupleHorizontalAndVecticalAcceleration(bool val) { _decouple_horizontal_and_vertical_acceleration = val; }


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

	// Limits
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_thr_xy_margin{}; ///< Margin to keep for horizontal control when saturating prioritized vertical thrust
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have

	bool _decouple_horizontal_and_vertical_acceleration{true}; ///< Ignore vertical acceleration setpoint to remove its effect on the tilt setpoint

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	float _yaw{}; /**< current heading */
	matrix::Quatf _q{}; /**< current attitude */
	matrix::Vector3f _ang_vel{}; /**< angular velocity */

	// Setpoints
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	matrix::Vector3f _acc_sp; /**< desired acceleration */
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
