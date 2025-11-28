/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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

#pragma once

#include "launchdetection/LaunchDetector.h"
#include "runway_takeoff/RunwayTakeoff.h"

#include <float.h>

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/npfg/npfg.hpp>
#include <lib/pid/PID.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <modules/flight_mode_manager/tasks/Utility/Sticks.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/fixed_wing_lateral_guidance_status.h>
#include <uORB/topics/fixed_wing_lateral_setpoint.h>
#include <uORB/topics/fixed_wing_longitudinal_setpoint.h>
#include <uORB/topics/fixed_wing_runway_control.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/launch_detection_status.h>
#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/orbit_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_controller_landing_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/longitudinal_control_configuration.h>
#include <uORB/uORB.h>

#ifdef CONFIG_FIGURE_OF_EIGHT
#include "figure_eight/FigureEight.hpp"
#include <uORB/topics/figure_eight_status.h>
#endif // CONFIG_FIGURE_OF_EIGHT

using namespace launchdetection;
using namespace runwaytakeoff;
using namespace time_literals;

using matrix::Vector2d;
using matrix::Vector2f;

static constexpr float WIND_EST_TIMEOUT = 10_s;
static constexpr float MIN_AUTO_TIMESTEP = 0.01f;
static constexpr float MAX_AUTO_TIMESTEP = 0.05f;
static constexpr float HDG_HOLD_MAN_INPUT_THRESH = 0.01f;
static constexpr float MANUAL_TOUCHDOWN_NUDGE_INPUT_DEADZONE = 0.15f;
static constexpr uint64_t ROLL_WARNING_TIMEOUT = 2_s;
static constexpr float ROLL_WARNING_CAN_RUN_THRESHOLD = 0.9f;

class FixedWingModeManager final : public ModuleBase<FixedWingModeManager>, public ModuleParams,
	public px4::WorkItem
{
public:
	FixedWingModeManager();
	~FixedWingModeManager() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);
	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	void parameters_update();
	void airspeed_poll();
	void wind_poll(const hrt_abstime now);
	void manual_control_setpoint_poll();
	void vehicle_attitude_poll();
	void vehicle_command_poll();
	void vehicle_control_mode_poll();

	void reset_takeoff_state();
	void reset_landing_state();
	void set_control_mode_current(const hrt_abstime &now);
	void update_in_air_states(const hrt_abstime now);
	void move_position_setpoint_for_vtol_transition(position_setpoint_s &current_sp);

	void control_auto(const float control_interval, const Vector2d &curr_pos,
			  const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev,
			  const position_setpoint_s &pos_sp_curr, const position_setpoint_s &pos_sp_next);
	void control_auto_fixed_bank_alt_hold();
	void control_auto_descend();
	uint8_t handle_setpoint_type(const position_setpoint_s &pos_sp_curr, const position_setpoint_s &pos_sp_next);
	void control_auto_position(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				   const position_setpoint_s &pos_sp_prev, const position_setpoint_s &pos_sp_curr);
	void control_auto_velocity(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				   const position_setpoint_s &pos_sp_curr);
	void control_auto_loiter(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				 const position_setpoint_s &pos_sp_curr, const position_setpoint_s &pos_sp_next);
	void control_auto_path(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
			       const position_setpoint_s &pos_sp_curr);
	void control_auto_takeoff(const hrt_abstime &now, const float control_interval, const Vector2d &global_position,
				  const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr);
	void control_auto_takeoff_no_nav(const hrt_abstime &now, const float control_interval,
					 const float current_setpoint_altitude_amsl);
	void control_auto_landing_straight(const hrt_abstime &now, const float control_interval,
					   const Vector2f &ground_speed, const position_setpoint_s &pos_sp_prev,
					   const position_setpoint_s &pos_sp_curr);
	void control_auto_landing_circular(const hrt_abstime &now, const float control_interval,
					   const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr);
	void control_manual_altitude(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed);
	void control_manual_position(const hrt_abstime now, const float control_interval, const Vector2d &curr_pos,
				     const Vector2f &ground_speed);
	void control_backtransition_heading_hold();
	void control_backtransition_line_follow(const Vector2f &ground_speed, const position_setpoint_s &pos_sp_curr);

	float rollAngleToLateralAccel(float roll_body) const;
	float get_manual_airspeed_setpoint();

	void landing_status_publish();
	void updateLandingAbortStatus(const uint8_t new_abort_status = position_controller_landing_status_s::NOT_ABORTED);
	bool checkLandingAbortBitMask(const uint8_t automatic_abort_criteria_bitmask, uint8_t landing_abort_criterion);
	void publishLocalPositionSetpoint(const position_setpoint_s &current_waypoint);
	void publishOrbitStatus(const position_setpoint_s pos_sp);

	DirectionalGuidanceOutput navigateWaypoints(const Vector2f &start_waypoint, const Vector2f &end_waypoint,
			const Vector2f &vehicle_pos, const Vector2f &ground_vel,
			const Vector2f &wind_vel);
	DirectionalGuidanceOutput navigateWaypoint(const Vector2f &waypoint_pos, const Vector2f &vehicle_pos,
			const Vector2f &ground_vel, const Vector2f &wind_vel);
	DirectionalGuidanceOutput navigateLine(const Vector2f &point_on_line_1, const Vector2f &point_on_line_2,
					       const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel);
	DirectionalGuidanceOutput navigateLine(const Vector2f &point_on_line, const float line_bearing,
					       const Vector2f &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel);
	DirectionalGuidanceOutput navigateLoiter(const Vector2f &loiter_center, const Vector2f &vehicle_pos,
			float radius, bool loiter_direction_counter_clockwise,
			const Vector2f &ground_vel, const Vector2f &wind_vel);
	DirectionalGuidanceOutput navigatePathTangent(const Vector2f &vehicle_pos, const Vector2f &position_setpoint,
			const Vector2f &tangent_setpoint, const Vector2f &ground_vel,
			const Vector2f &wind_vel, const float &curvature);
	DirectionalGuidanceOutput navigateBearing(const Vector2f &vehicle_pos, float bearing, const Vector2f &ground_vel,
			const Vector2f &wind_vel);
	DirectionalGuidanceOutput navigateL1(const Vector2f &vehicle_pos, const Vector2f &ground_vel,
					     const Vector2f &wind_vel, const Vector2f &unit_path_tangent,
					     const Vector2f &closest_point_on_path, const float &path_curvature);
	DirectionalGuidanceOutput navigateL1Conservative(const Vector2f &vehicle_pos, const Vector2f &ground_vel,
			const Vector2f &wind_vel, const Vector2f &unit_path_tangent,
			const Vector2f &closest_point_on_path, const float &path_curvature);
	DirectionalGuidanceOutput navigatePID(const Vector2f &vehicle_pos, const Vector2f &ground_vel,
					      const Vector2f &wind_vel, const Vector2f &unit_path_tangent,
					      const Vector2f &closest_point_on_path, const float &path_curvature);
	void updateGuidanceTelemetry(int guidance_mode, const DirectionalGuidanceOutput &sp, float track_error,
				     float bearing_feas = NAN, float bearing_feas_on_track = NAN,
				     float track_error_bound = NAN, float adapted_period = NAN);
	void publish_lateral_guidance_status(const hrt_abstime now);

#ifdef CONFIG_FIGURE_OF_EIGHT
	void controlAutoFigureEight(const float control_interval, const Vector2d &curr_pos, const Vector2f &ground_speed,
				    const position_setpoint_s &pos_sp_curr);
	void publishFigureEightStatus(const position_setpoint_s pos_sp);
#endif // CONFIG_FIGURE_OF_EIGHT

	void initializeAutoLanding(const hrt_abstime &now, const position_setpoint_s &pos_sp_prev,
				   const float land_point_altitude, const Vector2f &local_position,
				   const Vector2f &local_land_point);
	Vector2f calculateTouchdownPosition(const float control_interval, const Vector2f &local_land_position);
	Vector2f calculateLandingApproachVector() const;
	float getLandingTerrainAltitudeEstimate(const hrt_abstime &now, const float land_point_altitude,
						const bool abort_on_terrain_measurement_timeout,
						const bool abort_on_terrain_timeout);
	float getMaxRollAngleNearGround(const float altitude, const float terrain_altitude) const;

	class ControlConfigurationHandler
	{
	public:
		void setThrottleMax(float v) { _config.throttle_max = v; _dirty = true; }
		void setThrottleMin(float v) { _config.throttle_min = v; _dirty = true; }
		void setSpeedWeight(float v) { _config.speed_weight = v; _dirty = true; }
		void setLateralAccelMax(float v) { _config.lateral_accel_max = v; _dirty = true; }
		void setEnforceLowHeightCondition(bool v) { _config.enforce_low_height_condition = v; _dirty = true; }
		void setPitchMin(float v) { _config.pitch_min = v; _dirty = true; }
		void setPitchMax(float v) { _config.pitch_max = v; _dirty = true; }
		void setClimbRateTarget(float v) { _config.climb_rate_target = v; _dirty = true; }
		void setSinkRateTarget(float v) { _config.sink_rate_target = v; _dirty = true; }
		void setDisableUnderspeedProtection(bool v) { _config.disable_underspeed_protection = v; _dirty = true; }
		void resetLastPublishTime() { _last_publish_time = 0; }

		void update(const hrt_abstime now)
		{
			if (_dirty || (now - _last_publish_time) > 1_s) {
				_config.timestamp = now;
				_configuration_pub.publish(_config);
				_last_publish_time = now;
				_dirty = false;
			}
		}

	private:
		uORB::Publication<longitudinal_control_configuration_s> _configuration_pub{ORB_ID(longitudinal_control_configuration)};
		longitudinal_control_configuration_s _config{};
		hrt_abstime _last_publish_time{0};
		bool _dirty{true};
	};

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _wind_sub{ORB_ID(wind)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<position_controller_landing_status_s> _pos_ctrl_landing_status_pub{ORB_ID(position_controller_landing_status)};
	uORB::Publication<launch_detection_status_s> _launch_detection_status_pub{ORB_ID(launch_detection_status)};
	uORB::Publication<landing_gear_s> _landing_gear_pub{ORB_ID(landing_gear)};
	uORB::Publication<normalized_unsigned_setpoint_s> _flaps_setpoint_pub{ORB_ID(flaps_setpoint)};
	uORB::Publication<normalized_unsigned_setpoint_s> _spoilers_setpoint_pub{ORB_ID(spoilers_setpoint)};
	uORB::Publication<fixed_wing_lateral_setpoint_s> _lateral_ctrl_sp_pub{ORB_ID(fixed_wing_lateral_setpoint)};
	uORB::Publication<fixed_wing_longitudinal_setpoint_s> _longitudinal_ctrl_sp_pub{ORB_ID(fixed_wing_longitudinal_setpoint)};
	uORB::Publication<fixed_wing_lateral_guidance_status_s> _fixed_wing_lateral_guidance_status_pub{ORB_ID(fixed_wing_lateral_guidance_status)};
	uORB::Publication<fixed_wing_runway_control_s> _fixed_wing_runway_control_pub{ORB_ID(fixed_wing_runway_control)};
	uORB::PublicationMulti<orbit_status_s> _orbit_status_pub{ORB_ID(orbit_status)};

#ifdef CONFIG_FIGURE_OF_EIGHT
	FigureEight _figure_eight;
	uORB::Publication<figure_eight_status_s> _figure_eight_status_pub {ORB_ID(figure_eight_status)};
#endif // CONFIG_FIGURE_OF_EIGHT

	position_setpoint_triplet_s _pos_sp_triplet{};
	vehicle_control_mode_s _control_mode{};
	vehicle_local_position_s _local_pos{};
	vehicle_status_s _vehicle_status{};

	Vector2f _lpos_where_backtrans_started;
	Vector2f _closest_point_on_path;

	bool _position_setpoint_previous_valid{false};
	bool _position_setpoint_current_valid{false};
	bool _position_setpoint_next_valid{false};

	perf_counter_t _loop_perf{nullptr};
	hrt_abstime _last_time_position_control_called{0};

	uint8_t _position_sp_type{0};

	enum FW_POSCTRL_MODE {
		FW_POSCTRL_MODE_AUTO,
		FW_POSCTRL_MODE_AUTO_ALTITUDE,
		FW_POSCTRL_MODE_AUTO_CLIMBRATE,
		FW_POSCTRL_MODE_AUTO_TAKEOFF,
		FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT,
		FW_POSCTRL_MODE_AUTO_LANDING_CIRCULAR,
		FW_POSCTRL_MODE_AUTO_PATH,
		FW_POSCTRL_MODE_MANUAL_POSITION,
		FW_POSCTRL_MODE_MANUAL_ALTITUDE,
		FW_POSCTRL_MODE_TRANSITION_TO_HOVER_LINE_FOLLOW,
		FW_POSCTRL_MODE_TRANSITION_TO_HOVER_HEADING_HOLD,
		FW_POSCTRL_MODE_OTHER
	} _control_mode_current{FW_POSCTRL_MODE_OTHER};

	enum StickConfig {
		STICK_CONFIG_SWAP_STICKS_BIT = (1 << 0),
		STICK_CONFIG_ENABLE_AIRSPEED_SP_MANUAL_BIT = (1 << 1)
	};

	double _current_latitude{0};
	double _current_longitude{0};
	float _current_altitude{0.f};

	float _yaw{0.0f};
	float _yawrate{0.0f};
	float _body_acceleration_x{0.f};
	float _body_velocity_x{0.f};
	float _reference_altitude{NAN};
	bool _landed{true};

	bool _completed_manual_takeoff{false};
	float _hdg_hold_yaw{0.0f};
	bool _hdg_hold_enabled{false};
	bool _yaw_lock_engaged{false};
	Vector2f _hdg_hold_position{};
	float _manual_control_setpoint_for_height_rate{0.0f};
	float _manual_control_setpoint_for_airspeed{0.0f};
	float _commanded_manual_airspeed_setpoint{NAN};

	float _takeoff_ground_alt{0.0f};
	LaunchDetector _launchDetector;
	bool _launch_detected{false};
	float _launch_current_yaw{0.f};
	RunwayTakeoff _runway_takeoff;
	bool _skipping_takeoff_detection{false};

	struct FlareStates {
		bool flaring{false};
		hrt_abstime start_time{0};
		float initial_height_rate_setpoint{0.0f};
		float initial_throttle_setpoint{0.0f};
	} _flare_states;

	float _last_valid_terrain_alt_estimate{0.0f};
	hrt_abstime _last_time_terrain_alt_was_valid{0};
	uint8_t _landing_abort_status{position_controller_landing_status_s::NOT_ABORTED};
	float _lateral_touchdown_position_offset{0.0f};
	Vector2f _landing_approach_entrance_offset_vector{};
	float _landing_approach_entrance_rel_alt{0.0f};

	float _airspeed_eas{0.f};
	bool _airspeed_valid{false};
	hrt_abstime _time_airspeed_last_valid{0};

	Vector2f _wind_vel{0.0f, 0.0f};
	bool _wind_valid{false};
	hrt_abstime _time_wind_last_received{0};

	float _backtrans_heading{NAN};
	uint8_t _xy_reset_counter{0};
	uint64_t _time_last_xy_reset{0};
	float _min_current_sp_distance_xy{FLT_MAX};
	float _target_bearing{0.0f};
	int8_t _new_landing_gear_position{landing_gear_s::GEAR_KEEP};
	float _flaps_setpoint{0.f};
	float _spoilers_setpoint{0.f};
	float _time_in_fixed_bank_loiter{0};
	float _body_roll_limit{0.0f};

	NPFG _directional_guidance;
	PID _pid_xte;
	float _pid_kd{0.0f};
	float _pid_last_course{0.0f};
	float _pid_last_error{NAN};
	hrt_abstime _pid_last_update_time{0};
	float _pid_update_freq{0.0f};
	float _pid_update_freq_sum{0.0f};
	int _pid_update_count{0};

	DirectionalGuidanceOutput _latest_guidance_output{};
	float _latest_track_error{NAN};
	float _latest_bearing_feas{NAN};
	float _latest_bearing_feas_on_track{NAN};
	float _latest_track_error_bound{NAN};
	float _latest_adapted_period{NAN};
	int _latest_guidance_mode{-1};

	SlewRate<float> _airspeed_slew_rate_controller;
	ControlConfigurationHandler _ctrl_configuration_handler;
	Sticks _sticks{this};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,

		(ParamInt<px4::params::FW_GUIDANCE_MODE>) _param_fw_guidance_mode,
		(ParamFloat<px4::params::FW_L1_PERIOD>) _param_fw_l1_period,
		(ParamFloat<px4::params::FW_L1_DAMPING>) _param_fw_l1_damping,

		(ParamFloat<px4::params::FW_PID_XTE_KP>) _param_pid_xte_kp,
		(ParamFloat<px4::params::FW_PID_XTE_KI>) _param_pid_xte_ki,
		(ParamFloat<px4::params::FW_PID_XTE_KD>) _param_pid_xte_kd,
		(ParamFloat<px4::params::FW_PID_XTE_MAXA>) _param_pid_xte_maxa,
		(ParamFloat<px4::params::FW_PID_XTE_ILIM>) _param_pid_xte_ilim,

		(ParamFloat<px4::params::NPFG_PERIOD>) _param_npfg_period,
		(ParamFloat<px4::params::NPFG_DAMPING>) _param_npfg_damping,
		(ParamBool<px4::params::NPFG_LB_PERIOD>) _param_npfg_en_period_lb,
		(ParamBool<px4::params::NPFG_UB_PERIOD>) _param_npfg_en_period_ub,
		(ParamFloat<px4::params::NPFG_ROLL_TC>) _param_npfg_roll_time_const,
		(ParamFloat<px4::params::NPFG_SW_DST_MLT>) _param_npfg_switch_distance_multiplier,
		(ParamFloat<px4::params::NPFG_PERIOD_SF>) _param_npfg_period_safety_factor,

		(ParamFloat<px4::params::FW_LND_AIRSPD>) _param_fw_lnd_airspd,
		(ParamFloat<px4::params::FW_LND_ANG>) _param_fw_lnd_ang,
		(ParamFloat<px4::params::FW_LND_FL_PMAX>) _param_fw_lnd_fl_pmax,
		(ParamFloat<px4::params::FW_LND_FL_PMIN>) _param_fw_lnd_fl_pmin,
		(ParamFloat<px4::params::FW_LND_FLALT>) _param_fw_lnd_flalt,
		(ParamBool<px4::params::FW_LND_EARLYCFG>) _param_fw_lnd_earlycfg,
		(ParamInt<px4::params::FW_LND_USETER>) _param_fw_lnd_useter,
		(ParamFloat<px4::params::FW_LND_FL_SINK>) _param_fw_lnd_fl_sink,
		(ParamFloat<px4::params::FW_LND_FL_TIME>) _param_fw_lnd_fl_time,
		(ParamFloat<px4::params::FW_LND_TD_TIME>) _param_fw_lnd_td_time,
		(ParamFloat<px4::params::FW_LND_TD_OFF>) _param_fw_lnd_td_off,
		(ParamInt<px4::params::FW_LND_NUDGE>) _param_fw_lnd_nudge,
		(ParamInt<px4::params::FW_LND_ABORT>) _param_fw_lnd_abort,

		(ParamFloat<px4::params::FW_P_LIM_MAX>) _param_fw_p_lim_max,
		(ParamFloat<px4::params::FW_P_LIM_MIN>) _param_fw_p_lim_min,
		(ParamFloat<px4::params::FW_R_LIM>) _param_fw_r_lim,

		(ParamFloat<px4::params::FW_T_CLMB_MAX>) _param_fw_t_clmb_max,
		(ParamFloat<px4::params::FW_T_CLMB_R_SP>) _param_climbrate_target,
		(ParamFloat<px4::params::FW_T_SINK_R_SP>) _param_sinkrate_target,
		(ParamFloat<px4::params::FW_T_SPDWEIGHT>) _param_t_spdweight,
		(ParamFloat<px4::params::FW_THR_IDLE>) _param_fw_thr_idle,
		(ParamFloat<px4::params::FW_THR_MAX>) _param_fw_thr_max,
		(ParamFloat<px4::params::FW_THR_MIN>) _param_fw_thr_min,

		(ParamFloat<px4::params::FW_FLAPS_LND_SCL>) _param_fw_flaps_lnd_scl,
		(ParamFloat<px4::params::FW_FLAPS_TO_SCL>) _param_fw_flaps_to_scl,
		(ParamFloat<px4::params::FW_SPOILERS_LND>) _param_fw_spoilers_lnd,

		(ParamInt<px4::params::FW_POS_STK_CONF>) _param_fw_pos_stk_conf,
		(ParamInt<px4::params::FW_GPSF_LT>) _param_nav_gpsf_lt,
		(ParamFloat<px4::params::FW_GPSF_R>) _param_nav_gpsf_r,

		(ParamBool<px4::params::FW_USE_AIRSPD>) _param_fw_use_airspd,
		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad,
		(ParamFloat<px4::params::NAV_FW_ALT_RAD>) _param_nav_fw_alt_rad,
		(ParamFloat<px4::params::FW_WING_SPAN>) _param_fw_wing_span,
		(ParamFloat<px4::params::FW_WING_HEIGHT>) _param_fw_wing_height,
		(ParamFloat<px4::params::RWTO_NUDGE>) _param_rwto_nudge,
		(ParamFloat<px4::params::RWTO_PSP>) _param_rwto_psp,
		(ParamFloat<px4::params::FW_TKO_AIRSPD>) _param_fw_tko_airspd,
		(ParamBool<px4::params::FW_LAUN_DETCN_ON>) _param_fw_laun_detcn_on
	)
};

