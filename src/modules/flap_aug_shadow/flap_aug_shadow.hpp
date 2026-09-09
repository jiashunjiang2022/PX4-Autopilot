#pragma once

#include "fast_predictor.hpp"
#include "slow_engineering_shadow.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/flap_aug_shadow.h>
#include <uORB/topics/flap_frequency.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/rate_ctrl_terms.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>

namespace flap_aug_shadow
{

class CausalLowpassDerivative
{
public:
	float update(float input, bool valid);
	bool valid() const { return _output_valid; }
	void reset();

private:
	float _x1{0.f};
	float _x2{0.f};
	float _y1{0.f};
	float _y2{0.f};
	float _previous_filtered{0.f};
	float _replacement{0.f};
	bool _initialized{false};
	bool _output_valid{false};
};

class FlapAugShadow final : public ModuleBase<FlapAugShadow>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FlapAugShadow();
	~FlapAugShadow() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	void update_subscriptions();
	void update_legacy_integrator(hrt_abstime now);
	bool build_base_features(hrt_abstime now, float (&features)[FastPredictor::BaseFeatureCount],
			 bool &roll_valid, bool &pitch_valid, float &roll, float &pitch, float &yaw,
			 float &roll_sp, float &pitch_sp, float &ground_track_rate, bool &rtk_valid);
	bool fresh(hrt_abstime now, hrt_abstime timestamp, hrt_abstime maximum_age) const;
	void reset_estimators();

	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _flap_frequency_sub{ORB_ID(flap_frequency)};
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000000};
	uORB::Subscription _rate_status_sub{ORB_ID(rate_ctrl_status)};
	uORB::Subscription _rate_terms_sub{ORB_ID(rate_ctrl_terms)};
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _wind_sub{ORB_ID(wind)};
	uORB::Publication<flap_aug_shadow_s> _shadow_pub{ORB_ID(flap_aug_shadow)};

	actuator_motors_s _actuator_motors{};
	actuator_servos_s _actuator_servos{};
	airspeed_validated_s _airspeed{};
	battery_status_s _battery{};
	flap_frequency_s _flap_frequency{};
	manual_control_setpoint_s _manual{};
	rate_ctrl_status_s _rate_status{};
	rate_ctrl_terms_s _rate_terms{};
	sensor_accel_s _sensor_accel{};
	vehicle_angular_velocity_s _angular_velocity{};
	vehicle_attitude_s _attitude{};
	vehicle_attitude_setpoint_s _attitude_setpoint{};
	vehicle_land_detected_s _land{};
	vehicle_local_position_s _local_position{};
	vehicle_rates_setpoint_s _rates_setpoint{};
	vehicle_status_s _vehicle_status{};
	wind_s _wind{};

	FastPredictor _fast{};
	PhaseClassifier _phase{};
	SlowEngineeringShadow _slow{};
	CausalLowpassDerivative _rate_derivative[3]{};
	CausalLowpassDerivative _flap_derivative{};
	CausalLowpassDerivative _propulsion_derivative{};

	float _legacy_integrator[3]{};
	hrt_abstime _legacy_integrator_timestamp{0};
	uint64_t _legacy_integrator_bucket{UINT64_MAX};
	float _previous_track{0.f};
	float _unwrapped_track{0.f};
	float _track_rate_state{0.f};
	bool _previous_track_valid{false};
	bool _track_rate_initialized{false};
	hrt_abstime _invalid_since{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_RR_IMAX>) _param_fw_rr_imax,
		(ParamFloat<px4::params::FW_PR_IMAX>) _param_fw_pr_imax
	)
};

} // namespace flap_aug_shadow
