# Local position setpoint in NED frame using LQT


uint64 timestamp	# time since system start (microseconds)

float32 x		# in meters NED
float32 y		# in meters NED
float32 z		# in meters NED

float32 vx		# in meters/sec
float32 vy		# in meters/sec
float32 vz		# in meters/sec

float32[3] acceleration # in meters/sec^2
float32[3] torque	# normalized thrust vector in NED

float32 yaw		# in radians NED -PI..+PI
float32 yawspeed	# in radians/sec

float32 heave		# in normalized thrust

float32[4] y_togo	# yaw component of to-go quaternion
float32[4] s_togo	# yaw-free component of to-go quaternion

float32[3] vel_sp_k	# K component of velocity setpoint (debug)
float32[3] vel_sp_k_f	# K_f component of velocity setpoint (debug)
float32[3] vel_sp_k_z	# K_z component of velocity setpoint (debug)
