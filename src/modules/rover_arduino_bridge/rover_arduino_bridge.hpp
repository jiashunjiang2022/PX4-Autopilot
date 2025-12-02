#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/fixed_wing_lateral_guidance_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>

class RoverArduinoBridge : public ModuleBase<RoverArduinoBridge>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	RoverArduinoBridge();
	~RoverArduinoBridge() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	int open_serial();
	void close_serial();
	void send_cmd(float v_mps, float omega_rad_s);

	uORB::Subscription _fw_lat_guidance_status_sub{ORB_ID(fixed_wing_lateral_guidance_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	vehicle_control_mode_s _vehicle_control_mode{};

	int _uart_fd{-1};

	// Cached last values in case topics are not updated every cycle
	float _last_lat_accel{0.f};     // [m/s^2] lateral acceleration from NPFG
	float _last_ground_speed{0.f};  // [m/s] ground speed magnitude from local position
};
