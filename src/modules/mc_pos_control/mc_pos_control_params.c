/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_params.c
 * Multicopter position controller parameters.
 *
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Minimum collective thrust in auto thrust control
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 * Note: Without airmode zero thrust leads to zero roll/pitch control authority. (see MC_AIRMODE)
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MIN, 0.12f);

/**
 * Hover thrust
 *
 * Vertical thrust required to hover.
 * This value is mapped to center stick for manual throttle control.
 * With this value set to the thrust required to hover, transition
 * from manual to Altitude or Position mode while hovering will occur with the
 * throttle stick near center, which is then interpreted as (near)
 * zero demand for vertical speed.
 *
 * This parameter is also important for the landing detection to work correctly.
 *
 * @unit norm
 * @min 0.1
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_HOVER, 0.5f);

/**
 * Hover thrust source selector
 *
 * Set false to use the fixed parameter MPC_THR_HOVER
 * Set true to use the value computed by the hover thrust estimator
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_USE_HTE, 1);

/**
 * Thrust curve in Manual Mode
 *
 * This parameter defines how the throttle stick input is mapped to commanded thrust
 * in Manual/Stabilized flight mode.
 *
 * In case the default is used ('Rescale to hover thrust'), the stick input is linearly
 * rescaled, such that a centered stick corresponds to the hover throttle (see MPC_THR_HOVER).
 *
 * Select 'No Rescale' to directly map the stick 1:1 to the output. This can be useful
 * in case the hover thrust is very low and the default would lead to too much distortion
 * (e.g. if hover thrust is set to 20%, 80% of the upper thrust range is squeezed into the
 * upper half of the stick range).
 *
 * Note: In case MPC_THR_HOVER is set to 50%, the modes 0 and 1 are the same.
 *
 * @value 0 Rescale to hover thrust
 * @value 1 No Rescale
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_THR_CURVE, 0);

/**
 * Horizontal thrust margin
 *
 * Margin that is kept for horizontal control when prioritizing vertical thrust.
 * To avoid completely starving horizontal control with high vertical error.
 *
 * @unit norm
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_XY_MARG, 0.3f);

/**
 * Maximum thrust in auto thrust control
 *
 * Limit max allowed thrust
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MAX, 1.0f);

/**
 * Minimum manual thrust
 *
 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 * With MC_AIRMODE set to 1, this can safely be set to 0.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MANTHR_MIN, 0.08f);

/**
 * Proportional gain for vertical position error
 *
 * @min 0.0
 * @max 1.5
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_P, 1.0f);

/**
 * Proportional gain for vertical velocity error
 *
 * defined as correction acceleration in m/s^2 per m/s velocity error
 *
 * @min 2.0
 * @max 15.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_P_ACC, 4.0f);

/**
 * Integral gain for vertical velocity error
 *
 * defined as correction acceleration in m/s^2 per m velocity integral
 *
 * Non zero value allows hovering thrust estimation on stabilized or autonomous takeoff.
 *
 * @min 0.2
 * @max 3.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_I_ACC, 2.0f);

/**
 * Differential gain for vertical velocity error
 *
 * defined as correction acceleration in m/s^2 per m/s^2 velocity derivative
 *
 * @min 0.0
 * @max 2.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_D_ACC, 0.0f);

/**
 * Automatic ascent velocity
 *
 * Ascent velocity in auto modes.
 * For manual modes and offboard, see MPC_Z_VEL_MAX_UP
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_V_AUTO_UP, 3.f);

/**
 * Maximum ascent velocity
 *
 * Ascent velocity in manual modes and offboard.
 * For auto modes, see MPC_Z_V_AUTO_UP
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_UP, 3.f);

/**
 * Automatic descent velocity
 *
 * Descent velocity in auto modes.
 * For manual modes and offboard, see MPC_Z_VEL_MAX_DN
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_V_AUTO_DN, 1.5f);

/**
 * Maximum descent velocity
 *
 * Descent velocity in manual modes and offboard.
 * For auto modes, see MPC_Z_V_AUTO_DN
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_DN, 1.5f);

/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_P, 0.95f);

/**
 * Proportional gain for horizontal velocity error
 *
 * defined as correction acceleration in m/s^2 per m/s velocity error
 *
 * @min 1.2
 * @max 5.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_P_ACC, 1.8f);

/**
 * Integral gain for horizontal velocity error
 *
 * defined as correction acceleration in m/s^2 per m velocity integral
 * Non-zero value allows to eliminate steady state errors in the presence of disturbances like wind.
 *
 * @min 0.0
 * @max 60.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_I_ACC, 0.4f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * defined as correction acceleration in m/s^2 per m/s^2 velocity derivative
 *
 * @min 0.1
 * @max 2.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_D_ACC, 0.2f);

/**
 * Default horizontal velocity in mission
 *
 * Horizontal velocity used when flying autonomously in e.g. Missions, RTL, Goto.
 *
 * @unit m/s
 * @min 3.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_CRUISE, 5.0f);

/**
 * Proportional gain for horizontal trajectory position error
 *
 * @min 0.1
 * @max 1.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_TRAJ_P, 0.5f);

/**
 * Maximum horizontal error allowed by the trajectory generator
 *
 * The integration speed of the trajectory setpoint is linearly
 * reduced with the horizontal position tracking error. When the
 * error is above this parameter, the integration of the
 * trajectory is stopped to wait for the drone.
 *
 * This value can be adjusted depending on the tracking
 * capabilities of the vehicle.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_ERR_MAX, 2.0f);

/**
 * Maximum horizontal velocity setpoint in Position mode
 *
 * If velocity setpoint larger than MPC_XY_VEL_MAX is set, then
 * the setpoint will be capped to MPC_XY_VEL_MAX
 *
 * The maximum sideways and backward speed can be set differently
 * using MPC_VEL_MAN_SIDE and MPC_VEL_MAN_BACK, respectively.
 *
 * @unit m/s
 * @min 3.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MANUAL, 10.0f);

/**
 * Maximum sideways velocity in Position mode
 *
 * If set to a negative value or larger than
 * MPC_VEL_MANUAL then MPC_VEL_MANUAL is used.
 *
 * @unit m/s
 * @min -1.0
 * @max 20.0
 * @increment 0.1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MAN_SIDE, -1.0f);

/**
 * Maximum backward velocity in Position mode
 *
 * If set to a negative value or larger than
 * MPC_VEL_MANUAL then MPC_VEL_MANUAL is used.
 *
 * @unit m/s
 * @min -1.0
 * @max 20.0
 * @increment 0.1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MAN_BACK, -1.0f);

/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode. If higher speeds
 * are commanded in a mission they will be capped to this velocity.
 *
 * @unit m/s
 * @min 0.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX, 12.0f);

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit deg
 * @min 20.0
 * @max 89.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_AIR, 45.0f);

/**
 * Maximum tilt during landing
 *
 * Limits maximum tilt angle on landing.
 *
 * @unit deg
 * @min 10.0
 * @max 89.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_LND, 12.0f);

/**
 * Landing descend rate
 *
 * @unit m/s
 * @min 0.6
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_SPEED, 0.7f);

/**
 * Land crawl descend rate
 *
 * Used below MPC_LAND_ALT3 if distance sensor data is availabe.
 *
 * @unit m/s
 * @min 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_CRWL, 0.3f);

/**
 * Enable user assisted descent for autonomous land routine
 *
 * When enabled, descent speed will be:
 * stick full up - 0
 * stick centered - MPC_LAND_SPEED
 * stick full down - 2 * MPC_LAND_SPEED
 *
 * Additionally, the vehicle can be yawed and moved laterally using the other sticks.
 * Manual override during auto modes has to be disabled to use this feature (see COM_RC_OVERRIDE).
 *
 * @min 0
 * @max 1
 * @value 0 Fixed descent speed of MPC_LAND_SPEED
 * @value 1 User assisted descent speed
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_LAND_RC_HELP, 0);

/**
 * User assisted landing radius
 *
 * When user assisted descent is enabled (see MPC_LAND_RC_HELP),
 * this parameter controls the maximum position adjustment
 * allowed from the original landing point.
 *
 * @unit m
 * @min 0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_RADIUS, 1000.f);

/**
 * Takeoff climb rate
 *
 * @unit m/s
 * @min 1
 * @max 5
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TKO_SPEED, 1.5f);

/**
 * Maximal tilt angle in manual or altitude mode
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_TILT_MAX, 35.0f);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 400
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_MAX, 150.0f);

/**
 * Manual yaw rate input filter time constant
 *
 * Setting this parameter to 0 disables the filter
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_TAU, 0.08f);

/**
 * Deadzone of sticks where position hold is enabled
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_DZ, 0.1f);

/**
 * Maximum horizontal velocity for which position hold is enabled (use 0 to disable check)
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_XY, 0.8f);

/**
 * Maximum vertical velocity for which position hold is enabled (use 0 to disable check)
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_Z, 0.6f);

/**
 * Low pass filter cut freq. for numerical velocity derivative
 *
 * @unit Hz
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VELD_LP, 5.0f);

/**
 * Maximum horizontal acceleration for auto mode and for manual mode
 *
 * MPC_POS_MODE
 * 1 just deceleration
 * 3 acceleration and deceleration
 * 4 just acceleration
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_HOR_MAX, 5.0f);

/**
 * Acceleration for auto and for manual
 *
 * Note: In manual, this parameter is only used in MPC_POS_MODE 4.
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */

PARAM_DEFINE_FLOAT(MPC_ACC_HOR, 3.0f);

/**
 * Maximum vertical acceleration in velocity controlled modes upward
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_UP_MAX, 4.0f);

/**
 * Maximum vertical acceleration in velocity controlled modes down
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_DOWN_MAX, 3.0f);

/**
 * Maximum jerk limit
 *
 * Limit the maximum jerk of the vehicle (how fast the acceleration can change).
 * A lower value leads to smoother vehicle motions, but it also limits its
 * agility (how fast it can change directions or break).
 *
 * Setting this to the maximum value essentially disables the limit.
 *
 * Note: This is only used when MPC_POS_MODE is set to a smoothing mode 3 or 4.
 *
 * @unit m/s^3
 * @min 0.5
 * @max 500.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_JERK_MAX, 8.0f);

/**
 * Jerk limit in auto mode
 *
 * Limit the maximum jerk of the vehicle (how fast the acceleration can change).
 * A lower value leads to smoother vehicle motions, but it also limits its
 * agility.
 *
 * @unit m/s^3
 * @min 1.0
 * @max 80.0
 * @increment 1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_JERK_AUTO, 4.0f);

/**
 * Altitude control mode.
 *
 * Set to 0 to control height relative to the earth frame origin. This origin may move up and down in
 * flight due to sensor drift.
 * Set to 1 to control height relative to estimated distance to ground. The vehicle will move up and down
 * with terrain height variation. Requires a distance to ground sensor. The height controller will
 * revert to using height above origin if the distance to ground estimate becomes invalid as indicated
 * by the local_position.distance_bottom_valid message being false.
 * Set to 2 to control height relative to ground (requires a distance sensor) when stationary and relative
 * to earth frame origin when moving horizontally.
 * The speed threshold is controlled by the MPC_HOLD_MAX_XY parameter.
 *
 * @min 0
 * @max 2
 * @value 0 Altitude following
 * @value 1 Terrain following
 * @value 2 Terrain hold
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_ALT_MODE, 0);

/**
 * Manual position control stick exponential curve sensitivity
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_MAN_EXPO, 0.6f);

/**
 * Manual control stick vertical exponential curve
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_MAN_EXPO, 0.6f);

/**
 * Manual control stick yaw rotation exponential curve
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_YAW_EXPO, 0.6f);

/**
 * Max yaw rate in auto mode
 *
 * Limit the rate of change of the yaw setpoint in autonomous mode
 * to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MPC_YAWRAUTO_MAX, 45.0f);

/**
 * Altitude for 1. step of slow landing (descend)
 *
 * Below this altitude descending velocity gets limited to a value
 * between "MPC_Z_VEL_MAX_DN" (or "MPC_Z_V_AUTO_DN") and "MPC_LAND_SPEED"
 * Value needs to be higher than "MPC_LAND_ALT2"
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_ALT1, 10.0f);

/**
 * Altitude for 2. step of slow landing (landing)
 *
 * Below this altitude descending velocity gets
 * limited to "MPC_LAND_SPEED"
 * Value needs to be lower than "MPC_LAND_ALT1"
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_ALT2, 5.0f);

/**
 * Altitude for 3. step of slow landing
 *
 * Below this altitude descending velocity gets
 * limited to "MPC_LAND_CRWL", if LIDAR available.
 * No effect if LIDAR not available
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_ALT3, 1.0f);

/**
 * Position control smooth takeoff ramp time constant
 *
 * Increasing this value will make automatic and manual takeoff slower.
 * If it's too slow the drone might scratch the ground and tip over.
 * A time constant of 0 disables the ramp
 *
 * @min 0
 * @max 5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TKO_RAMP_T, 3.0f);

/**
 * Manual-Position control sub-mode
 *
 * The supported sub-modes are:
 * 0 Simple position control where sticks map directly to velocity setpoints
 *   without smoothing. Useful for velocity control tuning.
 * 3 Smooth position control with maximum acceleration and jerk limits based on
 *   jerk optimized trajectory generator (different algorithm than 1).
 * 4 Smooth position control where sticks map to acceleration and there's a virtual brake drag
 *
 * @value 0 Simple position control
 * @value 3 Smooth position control (Jerk optimized)
 * @value 4 Acceleration based input
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_POS_MODE, 4);

/**
 * Yaw mode.
 *
 * Specifies the heading in Auto.
 *
 * @min 0
 * @max 4
 * @value 0 towards waypoint
 * @value 1 towards home
 * @value 2 away from home
 * @value 3 along trajectory
 * @value 4 towards waypoint (yaw first)
 * @group Mission
 */
PARAM_DEFINE_INT32(MPC_YAW_MODE, 0);

/**
 * Responsiveness
 *
 * Changes the overall responsiveness of the vehicle.
 * The higher the value, the faster the vehicle will react.
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * the acceleration or jerk limits).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -1
 * @max 1
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(SYS_VEHICLE_RESP, -0.4f);

/**
 * Overall Horizontal Velocity Limit
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * MPC_XY_VEL_MAX or MPC_VEL_MANUAL).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -20
 * @max 20
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_ALL, -10.0f);

/**
 * Overall Vertical Velocity Limit
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * MPC_Z_VEL_MAX_UP or MPC_LAND_SPEED).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -3
 * @max 8
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_ALL, -3.0f);






/****************************************************************
 ******************************************************************
 ******************************************************************
 ******************************************************************
 ******************************************************************
 *  PARAMETERS FOR REMOVED MODULES BELOW
 ******************************************************************
 ******************************************************************
 ******************************************************************
 * *****************************************************************
 * *****************************************************************
 * *****************************************************************
 * *****************************************************************
 * *****************************************************************
 * *****************************************************************
 * *****************************************************************
 * *****************************************************************
 * *****************************************************************
*/

/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file fw_att_control_params.c
 *
 * Parameters defined by the fixed-wing attitude control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Attitude Roll Time Constant
 *
 * This defines the latency between a roll step input and the achieved setpoint
 * (inverse to a P gain). Half a second is a good start value and fits for
 * most average systems. Smaller systems may require smaller values, but as
 * this will wear out servos faster, the value should only be decreased as
 * needed.
 *
 * @unit s
 * @min 0.2
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_R_TC, 0.4f);

/**
 * Attitude pitch time constant
 *
 * This defines the latency between a pitch step input and the achieved setpoint
 * (inverse to a P gain). Half a second is a good start value and fits for
 * most average systems. Smaller systems may require smaller values, but as
 * this will wear out servos faster, the value should only be decreased as
 * needed.
 *
 * @unit s
 * @min 0.2
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_TC, 0.4f);

/**
 * Maximum positive / up pitch rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_RMAX_POS, 60.0f);

/**
 * Maximum negative / down pitch rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_RMAX_NEG, 60.0f);

/**
 * Maximum roll rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_R_RMAX, 70.0f);

/**
 * Maximum yaw rate setpoint
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_Y_RMAX, 50.0f);

/**
 * Enable wheel steering controller
 *
 * Only enabled during automatic runway takeoff and landing.
 * In all manual modes the wheel is directly controlled with yaw stick.
 *
 * @boolean
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_W_EN, 0);


/**
 * Wheel steering rate proportional gain
 *
 * This defines how much the wheel steering input will be commanded depending on the
 * current body angular rate error.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_P, 0.5f);

/**
 * Wheel steering rate integrator gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.005
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_I, 0.1f);

/**
 * Wheel steering rate integrator limit
 *
 * The portion of the integrator part in the control surface deflection is
 * limited to this value
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_IMAX, 0.4f);

/**
 * Maximum wheel steering rate
 *
 * This limits the maximum wheel steering rate the controller will output (in degrees per
 * second).
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_W_RMAX, 30.0f);

/**
 * Wheel steering rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_FF, 0.2f);

/**
 * Pitch setpoint offset (pitch at level flight)
 *
 * An airframe specific offset of the pitch setpoint in degrees, the value is
 * added to the pitch setpoint and should correspond to the pitch at
 * typical cruise speed of the airframe.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_PSP_OFF, 0.0f);

/**
 * Maximum manually added yaw rate
 *
 * This is the maximally added yaw rate setpoint from the yaw stick in any attitude controlled flight mode.
 * The controller already generates a yaw rate setpoint to coordinate a turn, and this value is added to it.
 * This is an absolute value, which is applied symmetrically to the negative and positive side.
 *
 * @unit deg/s
 * @min 0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_YR_MAX, 30.f);

/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file fw_autotune_attitude_control_params.c
 *
 * Parameters used by the attitude auto-tuner
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

/**
 * Start the autotuning sequence
 *
 * WARNING: this will inject steps to the rate controller
 * and can be dangerous. Only activate if you know what you
 * are doing, and in a safe environment.
 *
 * Any motion of the remote stick will abort the signal
 * injection and reset this parameter
 * Best is to perform the identification in position or
 * hold mode.
 * Increase the amplitude of the injected signal using
 * FW_AT_SYSID_AMP for more signal/noise ratio
 *
 * @boolean
 * @group Autotune
 */
PARAM_DEFINE_INT32(FW_AT_START, 0);

/**
 * Amplitude of the injected signal
 *
 * This parameter scales the signal sent to the
 * rate controller during system identification.
 *
 * @min 0.1
 * @max 6.0
 * @decimal 1
 * @group Autotune
 */
PARAM_DEFINE_FLOAT(FW_AT_SYSID_AMP, 1.0);

/**
 * Controls when to apply the new gains
 *
 * After the auto-tuning sequence is completed,
 * a new set of gains is available and can be applied
 * immediately or after landing.
 *
 * @value 0 Do not apply the new gains (logging only)
 * @value 1 Apply the new gains after disarm
 * @value 2 Apply the new gains in air
 * @group Autotune
 */
PARAM_DEFINE_INT32(FW_AT_APPLY, 2);

/**
 * Tuning axes selection
 *
 * Defines which axes will be tuned during the auto-tuning sequence
 *
 * Set bits in the following positions to enable:
 * 0 : Roll
 * 1 : Pitch
 * 2 : Yaw
 *
 * @bit 0 roll
 * @bit 1 pitch
 * @bit 2 yaw
 * @min 1
 * @max 7
 * @group Autotune
 */
PARAM_DEFINE_INT32(FW_AT_AXES, 3);

/**
 * Enable/disable auto tuning using an RC AUX input
 *
 * Defines which RC_MAP_AUXn parameter maps the RC channel used to enable/disable auto tuning.
 *
 * @value 0 Disable
 * @value 1 Aux1
 * @value 2 Aux2
 * @value 3 Aux3
 * @value 4 Aux4
 * @value 5 Aux5
 * @value 6 Aux6
 * @min 0
 * @max 6
 * @group Autotune
 */
PARAM_DEFINE_INT32(FW_AT_MAN_AUX, 0);
/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * Path navigation roll slew rate limit.
 *
 * The maximum change in roll angle setpoint per second.
 *
 * @unit deg/s
 * @min 0
 * @decimal 0
 * @increment 1
 * @group FW Path Control
 */
PARAM_DEFINE_FLOAT(FW_PN_R_SLEW_MAX, 90.0f);

/**
 * NPFG period
 *
 * Period of the NPFG control law.
 *
 * @unit s
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_PERIOD, 10.0f);

/**
 * NPFG damping ratio
 *
 * Damping ratio of the NPFG control law.
 *
 * @min 0.10
 * @max 1.00
 * @decimal 2
 * @increment 0.01
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_DAMPING, 0.7f);

/**
 * Enable automatic lower bound on the NPFG period
 *
 * Avoids limit cycling from a too aggressively tuned period/damping combination.
 * If set to false, also disables the upper bound NPFG_PERIOD_UB.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_LB_PERIOD, 1);

/**
 * Enable automatic upper bound on the NPFG period
 *
 * Adapts period to maintain track keeping in variable winds and path curvature.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_UB_PERIOD, 1);

/**
 * Enable track keeping excess wind handling logic.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_TRACK_KEEP, 1);

/**
 * Enable minimum forward ground speed maintaining excess wind handling logic
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_EN_MIN_GSP, 1);

/**
 * Enable wind excess regulation.
 *
 * Disabling this parameter further disables all other airspeed incrementation options.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_WIND_REG, 1);

/**
 * Maximum, minimum forward ground speed for track keeping in excess wind
 *
 * The maximum value of the minimum forward ground speed that may be commanded
 * by the track keeping excess wind handling logic. Commanded in full at the normalized
 * track error fraction of the track error boundary and reduced to zero on track.
 *
 * @unit m/s
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_GSP_MAX_TK, 5.0f);

/**
 * Roll time constant
 *
 * Time constant of roll controller command / response, modeled as first order delay.
 * Used to determine lower period bound. Setting zero disables automatic period bounding.
 *
 * @unit s
 * @min 0.00
 * @max 2.00
 * @decimal 2
 * @increment 0.05
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_ROLL_TC, 0.5f);

/**
 * NPFG switch distance multiplier
 *
 * Multiplied by the track error boundary to determine when the aircraft switches
 * to the next waypoint and/or path segment. Should be less than 1.
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_SW_DST_MLT, 0.32f);

/**
 * Period safety factor
 *
 * Multiplied by period for conservative minimum period bounding (when period lower
 * bounding is enabled). 1.0 bounds at marginal stability.
 *
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_PERIOD_SF, 1.5f);

/**
 * Trim throttle
 *
 * This is the throttle setting required to achieve FW_AIRSPD_TRIM during level flight.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_THR_TRIM, 0.6f);

/**
 * Throttle max slew rate
 *
 * Maximum slew rate for the commanded throttle
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_THR_SLEW_MAX, 0.0f);

/**
 * Minimum pitch angle
 *
 * The minimum pitch angle setpoint for a height-rate or altitude controlled mode.
 *
 * @unit deg
 * @min -60.0
 * @max 0.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MIN, -30.0f);

/**
 * Maximum pitch angle
 *
 * The maximum pitch angle setpoint setpoint for a height-rate or altitude controlled mode.
 *
 * @unit deg
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MAX, 30.0f);

/**
 * Maximum roll angle
 *
 * The maximum roll angle setpoint for setpoint for a height-rate or altitude controlled mode.
 *
 * @unit deg
 * @min 35.0
 * @max 65.0
 * @decimal 1
 * @increment 0.5
 * @group FW Path Control
 */
PARAM_DEFINE_FLOAT(FW_R_LIM, 50.0f);

/**
 * Throttle limit max
 *
 * This is the maximum throttle % that can be used by the controller.
 * For overpowered aircraft, this should be reduced to a value that
 * provides sufficient thrust to climb at the maximum pitch angle PTCH_MAX.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_THR_MAX, 1.0f);

/**
 * Throttle limit min
 *
 * This is the minimum throttle % that can be used by the controller.
 * For electric aircraft this will normally be set to zero, but can be set
 * to a small non-zero value if a folding prop is fitted to prevent the
 * prop from folding and unfolding repeatedly in-flight or to provide
 * some aerodynamic drag from a turning prop to improve the descent rate.
 *
 * For aircraft with internal combustion engine this parameter should be set
 * for desired idle rpm.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_THR_MIN, 0.0f);

/**
 * Idle throttle
 *
 * This is the minimum throttle while on the ground
 *
 * For aircraft with internal combustion engines, this parameter should be set
 * above the desired idle rpm. For electric motors, idle should typically be set
 * to zero.
 *
 * Note that in automatic modes, "landed" conditions will engage idle throttle.
 *
 * @unit norm
 * @min 0.0
 * @max 0.4
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_THR_IDLE, 0.0f);

/**
 * Maximum landing slope angle
 *
 * Typically the desired landing slope angle when landing configuration (flaps, airspeed) is enabled.
 * Set this value within the vehicle's performance limits.
 *
 * @unit deg
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_ANG, 5.0f);

/**
 * Minimum pitch during takeoff.
 *
 * @unit deg
 * @min -5.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group FW Path Control
 */
PARAM_DEFINE_FLOAT(FW_TKO_PITCH_MIN, 10.0f);

/**
 * Takeoff Airspeed
 *
 * The calibrated airspeed setpoint TECS will stabilize to during the takeoff climbout.
 *
 * If set <= 0.0, FW_AIRSPD_MIN will be set by default.
 *
 * @unit m/s
 * @min -1.0
 * @decimal 1
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_TKO_AIRSPD, -1.0f);

/**
 * Landing flare altitude (relative to landing altitude)
 *
 * NOTE: max(FW_LND_FLALT, FW_LND_FL_TIME * |z-velocity|) is taken as the flare altitude
 *
 * @unit m
 * @min 0.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FLALT, 0.5f);

/**
 * Use terrain estimation during landing. This is critical for detecting when to flare, and should be enabled if possible.
 *
 * NOTE: terrain estimate is currently solely derived from a distance sensor.
 *
 * If enabled and no measurement is found within a given timeout, the landing waypoint altitude will be used OR the landing
 * will be aborted, depending on the criteria set in FW_LND_ABORT.
 *
 * If disabled, FW_LND_ABORT terrain based criteria are ignored.
 *
 * @min 0
 * @max 2
 * @value 0 Disable the terrain estimate
 * @value 1 Use the terrain estimate to trigger the flare (only)
 * @value 2 Calculate landing glide slope relative to the terrain estimate
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_USETER, 1);

/**
 * Early landing configuration deployment
 *
 * When disabled, the landing configuration (flaps, landing airspeed, etc.) is only activated
 * on the final approach to landing. When enabled, it is already activated when entering the
 * final loiter-down (loiter-to-alt) waypoint before the landing approach. This shifts the (often large)
 * altitude and airspeed errors caused by the configuration change away from the ground such that
 * these are not so critical. It also gives the controller enough time to adapt to the new
 * configuration such that the landing approach starts with a cleaner initial state.
 *
 * @boolean
 *
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_EARLYCFG, 0);

/**
 * Flare, minimum pitch
 *
 * Minimum pitch during flare, a positive sign means nose up
 * Applied once flaring is triggered
 *
 * @unit deg
 * @min -5
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMIN, 2.5f);

/**
 * Flare, maximum pitch
 *
 * Maximum pitch during flare, a positive sign means nose up
 * Applied once flaring is triggered
 *
 * @unit deg
 * @min 0
 * @max 45.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMAX, 15.0f);

/**
 * Landing airspeed
 *
 * The calibrated airspeed setpoint during landing.
 *
 * If set <= 0.0, landing airspeed = FW_AIRSPD_MIN by default.
 *
 * @unit m/s
 * @min -1.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_AIRSPD, -1.f);

/**
 * Altitude time constant factor for landing
 *
 * Set this parameter to less than 1.0 to make TECS react faster to altitude errors during
 * landing than during normal flight. During landing, the TECS
 * altitude time constant (FW_T_ALT_TC) is multiplied by this value.
 *
 * @unit
 * @min 0.2
 * @max 1.0
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_THRTC_SC, 1.0f);



/*
 * TECS parameters
 *
 */

/**
 * Maximum climb rate
 *
 * This is the maximum climb rate that the aircraft can achieve with
 * the throttle set to THR_MAX and the airspeed set to the
 * trim value. For electric aircraft make sure this number can be
 * achieved towards the end of flight when the battery voltage has reduced.
 *
 * @unit m/s
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_CLMB_MAX, 5.0f);

/**
 * Minimum descent rate
 *
 * This is the sink rate of the aircraft with the throttle
 * set to THR_MIN and flown at the same airspeed as used
 * to measure FW_T_CLMB_MAX.
 *
 * @unit m/s
 * @min 1.0
 * @max 5.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_MIN, 2.0f);

/**
 * Maximum descent rate
 *
 * This sets the maximum descent rate that the controller will use.
 * If this value is too large, the aircraft can over-speed on descent.
 * This should be set to a value that can be achieved without
 * exceeding the lower pitch angle limit and without over-speeding
 * the aircraft.
 *
 * @unit m/s
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_MAX, 5.0f);

/**
 * Throttle damping factor
 *
 * This is the damping gain for the throttle demand loop.
 * Increase to add damping to correct for oscillations in speed and height.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_THR_DAMP, 0.1f);

/**
 * Integrator gain throttle
 *
 * This is the integrator gain on the throttle part of the control loop.
 * Increasing this gain increases the speed at which speed
 * and height offsets are trimmed out, but reduces damping and
 * increases overshoot. Set this value to zero to completely
 * disable all integrator action.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_I_GAIN_THR, 0.3f);

/**
 * Integrator gain pitch
 *
 * This is the integrator gain on the pitch part of the control loop.
 * Increasing this gain increases the speed at which speed
 * and height offsets are trimmed out, but reduces damping and
 * increases overshoot. Set this value to zero to completely
 * disable all integrator action.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_I_GAIN_PIT, 0.1f);

/**
 * Maximum vertical acceleration
 *
 * This is the maximum vertical acceleration (in m/s/s)
 * either up or down that the controller will use to correct speed
 * or height errors. The default value of 7 m/s/s (equivalent to +- 0.7 g)
 * allows for reasonably aggressive pitch changes if required to recover
 * from under-speed conditions.
 *
 * @unit m/s^2
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_VERT_ACC, 7.0f);

/**
 * Airspeed measurement standard deviation for airspeed filter.
 *
 * This is the measurement standard deviation for the airspeed used in the airspeed filter in TECS.
 *
 * @unit m/s
 * @min 0.01
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SPD_STD, 0.2f);

/**
 * Airspeed rate measurement standard deviation for airspeed filter.
 *
 * This is the measurement standard deviation for the airspeed rate used in the airspeed filter in TECS.
 *
 * @unit m/s^2
 * @min 0.01
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SPD_DEV_STD, 0.05f);

/**
 * Process noise standard deviation for the airspeed rate in the airspeed filter.
 *
 * This is the process noise standard deviation in the airspeed filter filter defining the noise in the
 * airspeed rate for the constant airspeed rate model. This is used to define how much the airspeed and
 * the airspeed rate are filtered. The smaller the value the more the measurements are smoothed with the
 * drawback for delays.
 *
 * @unit m/s^2
 * @min 0.01
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SPD_PRC_STD, 0.2f);


/**
 * Roll -> Throttle feedforward
 *
 * Increasing this gain turn increases the amount of throttle that will
 * be used to compensate for the additional drag created by turning.
 * Ideally this should be set to  approximately 10 x the extra sink rate
 * in m/s created by a 45 degree bank turn. Increase this gain if
 * the aircraft initially loses energy in turns and reduce if the
 * aircraft initially gains energy in turns. Efficient high aspect-ratio
 * aircraft (eg powered sailplanes) can use a lower value, whereas
 * inefficient low aspect-ratio models (eg delta wings) can use a higher value.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_RLL2THR, 15.0f);

/**
 * Speed <--> Altitude priority
 *
 * This parameter adjusts the amount of weighting that the pitch control
 * applies to speed vs height errors. Setting it to 0.0 will cause the
 * pitch control to control height and ignore speed errors. This will
 * normally improve height accuracy but give larger airspeed errors.
 * Setting it to 2.0 will cause the pitch control loop to control speed
 * and ignore height errors. This will normally reduce airspeed errors,
 * but give larger height errors. The default value of 1.0 allows the pitch
 * control to simultaneously control height and speed.
 * Set to 2 for gliders.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @increment 1.0
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SPDWEIGHT, 1.0f);

/**
 * Pitch damping factor
 *
 * This is the damping gain for the pitch demand loop. Increase to add
 * damping to correct for oscillations in height. The default value of 0.0
 * will work well provided the pitch to servo controller has been tuned
 * properly.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_PTCH_DAMP, 0.1f);

/**
 * Altitude error time constant.
 *
 * @min 2.0
 * @decimal 2
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_ALT_TC, 5.0f);

/**
 * Height rate feed forward
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_HRATE_FF, 0.3f);

/**
 * True airspeed error time constant.
 *
 * @min 2.0
 * @decimal 2
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_TAS_TC, 5.0f);

/**
 * Minimum groundspeed
 *
 * The controller will increase the commanded airspeed to maintain
 * this minimum groundspeed to the next waypoint.
 *
 * @unit m/s
 * @min 0.0
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_GND_SPD_MIN, 5.0f);

/**
 * RC stick configuration fixed-wing.
 *
 * Set RC/joystick configuration for fixed-wing manual position and altitude controlled flight.
 *
 * @min 0
 * @max 3
 * @bit 0 Alternative stick configuration (height rate on throttle stick, airspeed on pitch stick)
 * @bit 1 Enable airspeed setpoint via sticks in altitude and position flight mode
 * @group FW Path Control
 */
PARAM_DEFINE_INT32(FW_POS_STK_CONF, 2);

/**
 * Specific total energy rate first order filter time constant.
 *
 * This filter is applied to the specific total energy rate used for throttle damping.
 *
 * @min 0.0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_STE_R_TC, 0.4f);

/**
 * Specific total energy balance rate feedforward gain.
 *
 *
 * @min 0.5
 * @max 3
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SEB_R_FF, 1.0f);

/**
 * Default target climbrate.
 *
 * The default rate at which the vehicle will climb in autonomous modes to achieve altitude setpoints.
 * In manual modes this defines the maximum rate at which the altitude setpoint can be increased.
 *
 *
 * @unit m/s
 * @min 0.5
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_CLMB_R_SP, 3.0f);

/**
 * Default target sinkrate.
 *
 *
 * The default rate at which the vehicle will sink in autonomous modes to achieve altitude setpoints.
 * In manual modes this defines the maximum rate at which the altitude setpoint can be decreased.
 *
 * @unit m/s
 * @min 0.5
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_R_SP, 2.0f);

/**
 * GPS failure loiter time
 *
 * The time in seconds the system should do open loop loiter and wait for GPS recovery
 * before it starts descending. Set to 0 to disable. Roll angle is set to FW_GPSF_R.
 * Does only apply for fixed-wing vehicles or VTOLs with NAV_FORCE_VT set to 0.
 *
 * @unit s
 * @min 0
 * @max 3600
 * @group Mission
 */
PARAM_DEFINE_INT32(FW_GPSF_LT, 30);

/**
 * GPS failure fixed roll angle
 *
 * Roll in degrees during the loiter after the vehicle has lost GPS in an auto mode (e.g. mission or loiter).
 *
 * @unit deg
 * @min 0.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group Mission
 */
PARAM_DEFINE_FLOAT(FW_GPSF_R, 15.0f);

/**
 * Vehicle base weight.
 *
 * This is the weight of the vehicle at which it's performance limits were derived. A zero or negative value
 * disables trim throttle and minimum airspeed compensation based on weight.
 *
 * @unit kg
 * @decimal 1
 * @increment 0.5
 * @group Mission
 */
PARAM_DEFINE_FLOAT(WEIGHT_BASE, -1.0f);

/**
 * Vehicle gross weight.
 *
 * This is the actual weight of the vehicle at any time. This value will differ from WEIGHT_BASE in case weight was added
 * or removed from the base weight. Examples are the addition of payloads or larger batteries. A zero or negative value
 * disables trim throttle and minimum airspeed compensation based on weight.
 *
 * @unit kg
 * @decimal 1
 * @increment 0.1
 * @group Mission
 */
PARAM_DEFINE_FLOAT(WEIGHT_GROSS, -1.0f);

/**
 * The aircraft's wing span (length from tip to tip).
 *
 * This is used for limiting the roll setpoint near the ground. (if multiple wings, take the longest span)
 *
 * @unit m
 * @min 0.1
 * @decimal 1
 * @increment 0.1
 * @group FW Geometry
 */
PARAM_DEFINE_FLOAT(FW_WING_SPAN, 3.0);

/**
 * Height (AGL) of the wings when the aircraft is on the ground.
 *
 * This is used to constrain a minimum altitude below which we keep wings level to avoid wing tip strike. It's safer
 * to give a slight margin here (> 0m)
 *
 * @unit m
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group FW Geometry
 */
PARAM_DEFINE_FLOAT(FW_WING_HEIGHT, 0.5);

/**
 * Landing flare time
 *
 * Multiplied by the descent rate to calculate a dynamic altitude at which
 * to trigger the flare.
 *
 * NOTE: max(FW_LND_FLALT, FW_LND_FL_TIME * descent rate) is taken as the flare altitude
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_TIME, 1.0f);

/**
 * Landing touchdown time (since flare start)
 *
 * This is the time after the start of flaring that we expect the vehicle to touch the runway.
 * At this time, a 0.5s clamp down ramp will engage, constraining the pitch setpoint to RWTO_PSP.
 * If enabled, ensure that RWTO_PSP is configured appropriately for full gear contact on ground roll.
 *
 * Set to -1.0 to disable touchdown clamping. E.g. it may not be desirable to clamp on belly landings.
 *
 * The touchdown time will be constrained to be greater than or equal to the flare time (FW_LND_FL_TIME).
 *
 * @unit s
 * @min -1.0
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_TD_TIME, -1.0f);

/**
 * Landing flare sink rate
 *
 * TECS will attempt to control the aircraft to this sink rate via pitch angle (throttle killed during flare)
 *
 * @unit m/s
 * @min 0.0
 * @max 2
 * @decimal 2
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_SINK, 0.25f);

/**
 * Maximum lateral position offset for the touchdown point
 *
 * @unit m
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_TD_OFF, 3.0);

/**
 * Landing touchdown nudging option.
 *
 * Approach angle nudging: shifts the touchdown point laterally while keeping the approach entrance point constant
 * Approach path nudging: shifts the touchdown point laterally along with the entire approach path
 *
 * This is useful for manually adjusting the landing point in real time when map or GNSS errors cause an offset from the
 * desired landing vector. Nuding is done with yaw stick, constrained to FW_LND_TD_OFF (in meters) and the direction is
 * relative to the vehicle heading (stick deflection to the right = land point moves to the right as seen by the vehicle).
 *
 * @min 0
 * @max 2
 * @value 0 Disable nudging
 * @value 1 Nudge approach angle
 * @value 2 Nudge approach path
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_NUDGE, 2);

/**
 * Bit mask to set the automatic landing abort conditions.
 *
 * Terrain estimation:
 * bit 0: Abort if terrain is not found
 * bit 1: Abort if terrain times out (after a first successful measurement)
 *
 * The last estimate is always used as ground, whether the last valid measurement or the land waypoint, depending on the
 * selected abort criteria, until an abort condition is entered. If FW_LND_USETER == 0, these bits are ignored.
 *
 * TODO: Extend automatic abort conditions
 * e.g. glide slope tracking error (horizontal and vertical)
 *
 * @min 0
 * @max 3
 * @bit 0 Abort if terrain is not found (only applies to mission landings)
 * @bit 1 Abort if terrain times out (after a first successful measurement)
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_ABORT, 3);

/**
 * Wind-based airspeed scaling factor
 *
 * Multiplying this factor with the current absolute wind estimate gives the airspeed offset
 * added to the minimum airspeed setpoint limit. This helps to make the
 * system more robust against disturbances (turbulence) in high wind.
 * Only applies to AUTO flight mode.
 *
 * airspeed_min_adjusted = FW_AIRSPD_MIN + FW_WIND_ARSP_SC * wind.length()
 *
 * @min 0
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_WIND_ARSP_SC, 0.f);

/**
 * FW Launch detection
 *
 * Enables automatic launch detection based on measured acceleration. Use for hand- or catapult-launched vehicles.
 * Only available for fixed-wing vehicles.
 * Not compatible with runway takeoff.
 *
 * @boolean
 * @group FW Launch detection
 */
PARAM_DEFINE_INT32(FW_LAUN_DETCN_ON, 0);

/**
 * Flaps setting during take-off
 *
 * Sets a fraction of full flaps during take-off.
 * Also applies to flaperons if enabled in the mixer/allocation.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_TO_SCL, 0.0f);

/**
 * Flaps setting during landing
 *
 * Sets a fraction of full flaps during landing.
 * Also applies to flaperons if enabled in the mixer/allocation.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_LND_SCL, 1.0f);

/**
 * Spoiler landing setting
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_SPOILERS_LND, 0.f);

/**
 * Spoiler descend setting
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_SPOILERS_DESC, 0.f);
/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file fw_rate_control_params.c
 *
 * Parameters defined by the fixed-wing attitude control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Minimum Airspeed (CAS)
 *
 * The minimal airspeed (calibrated airspeed) the user is able to command.
 * Further, if the airspeed falls below this value, the TECS controller will try to
 * increase airspeed more aggressively.
 * Has to be set according to the vehicle's stall speed (which should be set in FW_AIRSPD_STALL),
 * with some margin between the stall speed and minimum airspeed.
 * This value corresponds to the desired minimum speed with the default load factor (level flight, default weight),
 * and is automatically adpated to the current load factor (calculated from roll setpoint and WEIGHT_GROSS/WEIGHT_BASE).
 *
 * @unit m/s
 * @min 0.5
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MIN, 10.0f);

/**
 * Maximum Airspeed (CAS)
 *
 * The maximal airspeed (calibrated airspeed) the user is able to command.
 *
 * @unit m/s
 * @min 0.5
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MAX, 20.0f);

/**
 * Airspeed mode
 *
 * On vehicles without airspeed sensor this parameter can be used to
 * enable flying without an airspeed reading
 *
 * @value 0 Use airspeed in controller
 * @value 1 Do not use airspeed in controller
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_ARSP_MODE, 0);

/**
 * Trim (Cruise) Airspeed
 *
 * The trim CAS (calibrated airspeed) of the vehicle. If an airspeed controller is active,
 * this is the default airspeed setpoint that the controller will try to achieve.
 *
 * @unit m/s
 * @min 0.5
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_TRIM, 15.0f);

/**
 * Stall Airspeed (CAS)
 *
 * The stall airspeed (calibrated airspeed) of the vehicle.
 * It is used for airspeed sensor failure detection and for the control
 * surface scaling airspeed limits.
 *
 * @unit m/s
 * @min 0.5
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_STALL, 7.0f);

/**
 * Pitch rate proportional gain.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_P, 0.08f);

/**
 * Pitch rate derivative gain.
 *
 * Pitch rate differential gain.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_D, 0.f);

/**
 * Pitch rate integrator gain.
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_I, 0.1f);

/**
 * Pitch rate integrator limit
 *
 * The portion of the integrator part in the control surface deflection is
 * limited to this value
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_IMAX, 0.4f);

/**
 * Roll rate proportional Gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_P, 0.05f);

/**
 * Roll rate derivative Gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations.
 * If value is too big oscillations will appear again.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_D, 0.00f);

/**
 * Roll rate integrator Gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 0.2
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_I, 0.1f);

/**
 * Roll integrator anti-windup
 *
 * The portion of the integrator part in the control surface deflection is limited to this value.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_IMAX, 0.2f);

/**
 * Yaw rate proportional gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_P, 0.05f);

/**
 * Yaw rate derivative gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations.
 * If value is too big oscillations will appear again.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_D, 0.0f);

/**
 * Yaw rate integrator gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 0.5
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * The portion of the integrator part in the control surface deflection is
 * limited to this value
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_IMAX, 0.2f);

/**
 * Roll rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output. Use this
 * to obtain a tigher response of the controller without introducing
 * noise amplification.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_FF, 0.5f);

/**
 * Pitch rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_FF, 0.5f);

/**
 * Yaw rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_FF, 0.3f);

/**
 * Acro body x max rate.
 *
 * This is the rate the controller is trying to achieve if the user applies full roll
 * stick input in acro mode.
 *
 * @min 45
 * @max 720
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_X_MAX, 90);

/**
 * Acro body pitch max rate setpoint.
 *
 * @min 45
 * @max 720
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_Y_MAX, 90);

/**
 * Acro body yaw max rate setpoint.
 *
 * @min 10
 * @max 180
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_Z_MAX, 45);

/**
 * Enable throttle scale by battery level
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery.
 *
 * @boolean
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_BAT_SCALE_EN, 0);

/**
 * Enable airspeed scaling
 *
 * This enables a logic that automatically adjusts the output of the rate controller to take
 * into account the real torque produced by an aerodynamic control surface given
 * the current deviation from the trim airspeed (FW_AIRSPD_TRIM).
 *
 * Enable when using aerodynamic control surfaces (e.g.: plane)
 * Disable when using rotor wings (e.g.: autogyro)
 *
 * @boolean
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_ARSP_SCALE_EN, 1);

/**
* Roll trim increment at minimum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_R_VMIN, 0.0f);

/**
* Pitch trim increment at minimum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_P_VMIN, 0.0f);

/**
* Yaw trim increment at minimum airspeed
*
* This increment is added to TRIM_YAW when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_Y_VMIN, 0.0f);

/**
* Roll trim increment at maximum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_R_VMAX, 0.0f);

/**
* Pitch trim increment at maximum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_P_VMAX, 0.0f);

/**
* Yaw trim increment at maximum airspeed
*
* This increment is added to TRIM_YAW when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.25
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_Y_VMAX, 0.0f);

/**
 * Manual roll scale
 *
 * Scale factor applied to the desired roll actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_R_SC, 1.0f);

/**
 * Maximum manual pitch angle
 *
 * Maximum manual pitch angle setpoint (positive & negative) in manual attitude-only stabilized mode
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_P_MAX, 30.0f);

/**
 * Manual pitch scale
 *
 * Scale factor applied to the desired pitch actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_P_SC, 1.0f);

/**
 * Maximum manual roll angle
 *
 * Maximum manual roll angle setpoint (positive & negative) in manual attitude-only stabilized mode
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_R_MAX, 45.0f);

/**
 * Manual yaw scale
 *
 * Scale factor applied to the desired yaw actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_Y_SC, 1.0f);

/**
 * Roll control to yaw control feedforward gain.
 *
 * This gain can be used to counteract the "adverse yaw" effect for fixed wings.
 * When the plane enters a roll it will tend to yaw the nose out of the turn.
 * This gain enables the use of a yaw actuator to counteract this effect.
 *
 * @min 0.0
 * @decimal 1
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RLL_TO_YAW_FF, 0.0f);

/**
 * Spoiler input in manual flight
 *
 * Chose source for manual setting of spoilers in manual flight modes.
 *
 * @value 0 Disabled
 * @value 1 Flaps channel
 * @value 2 Aux1
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_SPOILERS_MAN, 0);
/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file rover_pos_control_params.c
 *
 * Parameters defined by the position control task for ground rovers
 *
 * This is a modification of the fixed wing params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

/*
 * Controller parameters, accessible via MAVLink
 */

/**
 * Distance from front axle to rear axle
 *
 * A value of 0.31 is typical for 1/10 RC cars.
 *
 * @unit m
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_WHEEL_BASE, 0.31f);

/**
 * L1 distance
 *
 * This is the distance at which the next waypoint is activated. This should be set
 * to about 2-4x of GND_WHEEL_BASE and not smaller than one meter (due to GPS accuracy).
 *
 *
 * @unit m
 * @min 1.0
 * @max 50.0
 * @decimal 1
 * @increment 0.1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_L1_DIST, 1.0f);

/**
 * L1 period
 *
 * This is the L1 distance and defines the tracking
 * point ahead of the rover it's following.
 * Use values around 2-5m for a 0.3m wheel base. Tuning instructions: Shorten
 * slowly during tuning until response is sharp without oscillation.
 *
 * @unit m
 * @min 0.5
 * @max 50.0
 * @decimal 1
 * @increment 0.5
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_L1_PERIOD, 5.0f);

/**
 * L1 damping
 *
 * Damping factor for L1 control.
 *
 * @min 0.6
 * @max 0.9
 * @decimal 2
 * @increment 0.05
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_L1_DAMPING, 0.75f);

/**
 * Cruise throttle
 *
 * This is the throttle setting required to achieve the desired cruise speed.
 * 10% is ok for a traxxas stampede vxl with ESC set to training mode
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_THR_CRUISE, 0.1f);

/**
 * Throttle limit max
 *
 * This is the maximum throttle % that can be used by the controller.
 * For a Traxxas stampede vxl with the ESC set to training, 30 % is enough
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_THR_MAX, 0.3f);

/**
 * Throttle limit min
 *
 * This is the minimum throttle % that can be used by the controller.
 * Set to 0 for rover
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_THR_MIN, 0.0f);

/**
 * Control mode for speed
 *
 * This allows the user to choose between closed loop gps speed or open loop cruise throttle speed
 * @min 0
 * @max 1
 * @value 0 open loop control
 * @value 1 close the loop with gps speed
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_SP_CTRL_MODE, 1);

/**
 * Speed proportional gain
 *
 * This is the proportional gain for the speed closed loop controller
 *
 * @unit %m/s
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_P, 2.0f);

/**
 * Speed Integral gain
 *
 * This is the integral gain for the speed closed loop controller
 *
 * @unit %m/s
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_I, 3.0f);

/**
 * Speed proportional gain
 *
 * This is the derivative gain for the speed closed loop controller
 *
 * @unit %m/s
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_D, 0.001f);

/**
 * Speed integral maximum value
 *
 * This is the maxim value the integral can reach to prevent wind-up.
 *
 * @unit %m/s
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_IMAX, 1.0f);

/**
 * Speed to throttle scaler
 *
 * This is a gain to map the speed control output to the throttle linearly.
 *
 * @unit %m/s
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_THR_SC, 1.0f);

/**
 * Trim ground speed
 *
 *
 * @unit m/s
 * @min 0.0
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_TRIM, 3.0f);

/**
 * Maximum ground speed
 *
 *
 * @unit m/s
 * @min 0.0
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_MAX, 10.0f);

/**
 * Maximum turn angle for Ackerman steering.
 *
 * At a control output of 0, the steering wheels are at 0 radians.
 * At a control output of 1, the steering wheels are at GND_MAX_ANG radians.
 *
 * @unit rad
 * @min 0.0
 * @max 3.14159
 * @decimal 3
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_MAX_ANG, 0.7854f);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 400
 * @decimal 1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_MAN_Y_MAX, 150.0f);
/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file uuv_att_control_params.c
 *
 * Parameters defined by the attitude control task for unmanned underwater vehicles (UUVs)
 *
 * This is a modification of the fixed wing/ground rover params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing/rover app are reported in those files.
 *
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

// Roll gains
/**
 * Roll proportional gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_P, 4.0f);

/**
 * Roll differential gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_ROLL_D, 1.5f);


// Pitch gains
/**
 * Pitch proportional gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_P, 4.0f);

/**
 * Pitch differential gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_PITCH_D, 2.0f);


// Yaw gains
/**
 * Yawh proportional gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_YAW_P, 4.0f);

/**
 * Yaw differential gain
 *
 * @group UUV Attitude Control
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(UUV_YAW_D, 2.0f);


// Input Modes
/**
 * Select Input Mode
 *
 * @value 0 use Attitude Setpoints
 * @value 1 Direct Feedthrough
 * @group UUV Attitude Control
 */
PARAM_DEFINE_INT32(UUV_INPUT_MODE, 0);

/**
 * Skip the controller
 *
 * @value 0 use the module's controller
 * @value 1 skip the controller and feedthrough the setpoints
 */
PARAM_DEFINE_INT32(UUV_SKIP_CTRL, 0);

/**
 * Direct roll input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_DIRCT_ROLL, 0.0f);

/**
 * Direct pitch input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_DIRCT_PITCH, 0.0f);

/**
 * Direct yaw input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_DIRCT_YAW, 0.0f);

/**
 * Direct thrust input
 *
 * @group UUV Attitude Control
 */
PARAM_DEFINE_FLOAT(UUV_DIRCT_THRUST, 0.0f);
/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file uuv_pos_control_params.c
 *
 * Parameters defined by the position control task for unmanned underwater vehicles (UUVs)
 *
 * This is a modification of the fixed wing/ground rover params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing/rover app are reported in those files.
 *
 * @author Tim Hansen <t.hansen@jacobs-university.de>
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */
/**
 * Gain of P controller X
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_X_P, 1.0f);
/**
 * Gain of P controller Y
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_Y_P, 1.0f);
/**
 * Gain of P controller Z
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_Z_P, 1.0f);

/**
 * Gain of D controller X
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_X_D, 0.2f);
/**
 * Gain of D controller Y
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_Y_D, 0.2f);
/**
 * Gain of D controller Z
 *
 * @group UUV Position Control
 */
PARAM_DEFINE_FLOAT(UUV_GAIN_Z_D, 0.2f);

/**
 * Stabilization mode(1) or Position Control(0)
 *
 * @value 0 Position Control
 * @value 1 Stabilization Mode
 * @group UUV Position Control
 */
PARAM_DEFINE_INT32(UUV_STAB_MODE, 1);
/****************************************************************************
 *
 *   Copyright (c) 2014-2022 PX4 Development Team. All rights reserved.
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
 * @file vtol_att_control_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <roman@px4.io>
 * @author Sander Smeets <sander@droneslab.com>
 */

/**
 * VTOL Type (Tailsitter=0, Tiltrotor=1, Standard=2)
 *
 * @value 0 Tailsitter
 * @value 1 Tiltrotor
 * @value 2 Standard
 * @min 0
 * @max 2
 * @reboot_required true
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_TYPE, 0);

/**
 * Lock control surfaces in hover
 *
 * If set to 1 the control surfaces are locked at the disarmed value in multicopter mode.
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_ELEV_MC_LOCK, 1);

/**
 * Duration of a front transition
 *
 * Time in seconds used for a transition
 *
 * @unit s
 * @min 0.1
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TRANS_DUR, 5.0f);

/**
 * Duration of a back transition
 *
 * Time in seconds used for a back transition
 *
 * @unit s
 * @min 0.1
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TRANS_DUR, 4.0f);

/**
 * Target throttle value for the transition to fixed wing flight.
 *
 * standard vtol: pusher
 *
 * tailsitter, tiltrotor: main throttle
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TRANS_THR, 1.0f);

/**
 * Target throttle value for the transition to hover flight.
 *
 * standard vtol: pusher
 *
 * tailsitter, tiltrotor: main throttle
 *
 *
 * @min -1
 * @max 1
 * @increment 0.01
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TRANS_THR, 0.0f);

/**
 * Approximate deceleration during back transition
 *
 * The approximate deceleration during a back transition in m/s/s
 * Used to calculate back transition distance in an auto mode.
 * For standard vtol and tiltrotors a controller is used to track this value during the transition.
 *
 * @unit m/s^2
 * @min 0.5
 * @max 10
 * @increment 0.1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_DEC_MSS, 2.0f);

/**
 * Transition blending airspeed
 *
 * Airspeed at which we can start blending both fw and mc controls. Set to 0 to disable.
 *
 * @unit m/s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_BLEND, 8.0f);

/**
 * Transition airspeed
 *
 * Airspeed at which we can switch to fw mode
 *
 * @unit m/s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_TRANS, 10.0f);

/**
 * Front transition timeout
 *
 * Time in seconds after which transition will be cancelled. Disabled if set to 0.
 *
 * @unit s
 * @min 0.1
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_TIMEOUT, 15.0f);

/**
 * Front transition minimum time
 *
 * Minimum time in seconds for front transition.
 *
 * @unit s
 * @min 0.0
 * @max 20.0
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_MIN_TM, 2.0f);

/**
 * QuadChute Altitude
 *
 * Minimum altitude for fixed wing flight, when in fixed wing the altitude drops below this altitude
 * the vehicle will transition back to MC mode and enter failsafe RTL
 *
 * @unit m
 * @min 0.0
 * @max 200.0
 * @increment 1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_MIN_ALT, 0.0f);

/**
 * Quad-chute uncommanded descent threshold
 *
 * Threshold for integrated height rate error to trigger a uncommanded-descent quad-chute.
 * Only checked in altitude-controlled fixed-wing flight.
 * Additional conditions that have to be met for uncommanded descent detection are a positive (climbing) height
 * rate setpoint and a negative (sinking) current height rate estimate.
 *
 * Set to 0 do disable this threshold.
 *
 * @unit m
 * @min 0.0
 * @max 200.0
 * @increment 1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_QC_HR_ERROR_I, 0.0f);

/**
 * Quad-chute transition altitude loss threshold
 *
 * Altitude loss threshold for quad-chute triggering during VTOL transition to fixed-wing flight.
 * If the current altitude is more than this value below the altitude at the beginning of the
 * transition, it will instantly switch back to MC mode and execute behavior defined in COM_QC_ACT.
 *
 * Set to 0 do disable this threshold.
 *
 * @unit m
 * @min 0
 * @max 50
 * @increment 1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_QC_T_ALT_LOSS, 10.0f);

/**
 * Quad-chute max pitch threshold
 *
 * Absolute pitch threshold for quad-chute triggering in FW mode.
 * Above this the vehicle will transition back to MC mode and execute behavior defined in COM_QC_ACT.
 * Set to 0 do disable this threshold.
 *
 * @unit deg
 * @min 0
 * @max 180
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_P, 0);

/**
 * Quad-chute max roll threshold
 *
 * Absolute roll threshold for quad-chute triggering in FW mode.
 * Above this the vehicle will transition back to MC mode and execute behavior defined in COM_QC_ACT.
 * Set to 0 do disable this threshold.
 *
 * @unit deg
 * @min 0
 * @max 180
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_R, 0);

/**
 * Quad-chute maximum height
 *
 * Maximum height above the ground (if available, otherwhise above
 * home if available, otherwise above the local origin) where triggering a quadchute is possible.
 * At high altitudes there is a big risk to deplete the battery and therefore crash if quad-chuting there.
 *
 * @unit m
 * @min 0
 * @increment 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_HMAX, 0);

/**
 * Airspeed less front transition time (open loop)
 *
 * The duration of the front transition when there is no airspeed feedback available.
 *
 * @unit s
 * @min 1.0
 * @max 30.0
 * @increment 0.5
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TR_OL_TM, 6.0f);

/**
 * Differential thrust in forwards flight.
 *
 * Enable differential thrust seperately for roll, pitch, yaw in forward (fixed-wing) mode.
 * The effectiveness of differential thrust around the corresponding axis can be
 * tuned by setting VT_FW_DIFTHR_S_R / VT_FW_DIFTHR_S_P / VT_FW_DIFTHR_S_Y.
 *
 * @min 0
 * @max 7
 * @bit 0 Yaw
 * @bit 1 Roll
 * @bit 2 Pitch
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_DIFTHR_EN, 0);

/**
 * Roll differential thrust factor in forward flight
 *
 * Differential thrust in forward flight is enabled via VT_FW_DIFTHR_EN.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_DIFTHR_S_R, 1.f);

/**
 * Pitch differential thrust factor in forward flight
 *
 * Differential thrust in forward flight is enabled via VT_FW_DIFTHR_EN.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_DIFTHR_S_P, 1.f);

/**
 * Yaw differential thrust factor in forward flight
 *
 * Differential thrust in forward flight is enabled via VT_FW_DIFTHR_EN.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_DIFTHR_S_Y, 0.1f);

/**
 * Backtransition deceleration setpoint to pitch feedforward gain.
 *
 *
 * @unit rad s^2/m
 * @min 0
 * @max 0.2
 * @decimal 2
 * @increment 0.01
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_DEC_FF, 0.f);

/**
 * Backtransition deceleration setpoint to pitch I gain.
 *
 *
 * @unit rad s/m
 * @min 0
 * @max 0.3
 * @decimal 2
 * @increment 0.05
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_DEC_I, 0.1f);

/**
 * Minimum pitch angle during hover.
 *
 * Minimum pitch angle during hover flight. If the desired pitch angle is is lower than this value
 * then the fixed-wing forward actuation can be used to compensate for the missing thrust in forward direction
 * (see VT_FW_TRHUST_EN)
 *
 * @unit deg
 * @min -10.0
 * @max 45.0
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_PITCH_MIN, -5.0f);

/**
 * Minimum pitch angle during hover landing.
 *
 * Overrides VT_PITCH_MIN when the vehicle is in LAND mode (hovering).
 * During landing it can be beneficial to allow lower minimum pitch angles as it can avoid the wings
 * generating too much lift and preventing the vehicle from sinking at the desired rate.
 *
 * @unit deg
 * @min -10.0
 * @max 45.0
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_LND_PITCH_MIN, -5.0f);

/**
 * Spoiler setting while landing (hover)
 *
 * @unit norm
 * @min -1
 * @max 1
 * @decimal 1
 * @increment 0.1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_SPOILER_MC_LD, 0.f);
