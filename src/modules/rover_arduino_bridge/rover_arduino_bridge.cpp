#include "rover_arduino_bridge.hpp"

#include <px4_platform_common/log.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <mathlib/mathlib.h>

using namespace time_literals;

// Minimum forward speed command [m/s] to avoid division by zero
static constexpr float V_CMD_MIN = 0.3f;

// Maximum yaw rate command magnitude [rad/s]
static constexpr float OMEGA_MAX = 2.0f;

// NOTE: Adjust this device path to match the UART connected to the Arduino.
// For example on many Pixhawk-style boards this could be "/dev/ttyS4".
#ifndef ROVER_ARDUINO_BRIDGE_DEVICE
#define ROVER_ARDUINO_BRIDGE_DEVICE "/dev/ttyS4"
#endif

// Default baud rate for the Arduino link.
static constexpr speed_t ROVER_ARDUINO_BRIDGE_BAUD = B57600;

RoverArduinoBridge::RoverArduinoBridge() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

RoverArduinoBridge::~RoverArduinoBridge()
{
	ScheduleClear();
	close_serial();
}

bool RoverArduinoBridge::init()
{
	if (open_serial() < 0) {
		PX4_ERR("failed to open Arduino UART " ROVER_ARDUINO_BRIDGE_DEVICE);
		// Still schedule; we will keep trying to reopen in Run()
	}

	ScheduleOnInterval(20_ms);
	return true;
}

int RoverArduinoBridge::open_serial()
{
	if (_uart_fd >= 0) {
		return _uart_fd;
	}

	int fd = ::open(ROVER_ARDUINO_BRIDGE_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		return -1;
	}

	struct termios uart_config {};

	if (tcgetattr(fd, &uart_config) < 0) {
		::close(fd);
		return -1;
	}

	// Raw mode, 8N1
	cfmakeraw(&uart_config);

	if (cfsetispeed(&uart_config, ROVER_ARDUINO_BRIDGE_BAUD) < 0 ||
	    cfsetospeed(&uart_config, ROVER_ARDUINO_BRIDGE_BAUD) < 0) {
		::close(fd);
		return -1;
	}

	if (tcsetattr(fd, TCSANOW, &uart_config) < 0) {
		::close(fd);
		return -1;
	}

	_uart_fd = fd;
	return _uart_fd;
}

void RoverArduinoBridge::close_serial()
{
	if (_uart_fd >= 0) {
		::close(_uart_fd);
		_uart_fd = -1;
	}
}

void RoverArduinoBridge::send_cmd(float v_mps, float omega_rad_s)
{
	if (_uart_fd < 0) {
		return;
	}

	char buf[64];
	const int len = snprintf(buf, sizeof(buf), "CMD,%.3f,%.3f\n",
				 static_cast<double>(v_mps),
				 static_cast<double>(omega_rad_s));

	if (len <= 0 || len >= static_cast<int>(sizeof(buf))) {
		return;
	}

	// Non-blocking write; ignore errors for now
	(void)::write(_uart_fd, buf, len);
}

void RoverArduinoBridge::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// Keep trying to open the UART if it is not available yet
	if (_uart_fd < 0) {
		open_serial();
	}

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	// Only send commands when position control (fixed-wing navigation) is active and the vehicle is armed
	const bool active = _vehicle_control_mode.flag_control_position_enabled && _vehicle_control_mode.flag_armed;

	if (active) {
		fixed_wing_lateral_guidance_status_s lat_status{};
		vehicle_local_position_s lpos{};

		if (_fw_lat_guidance_status_sub.update(&lat_status)) {
			if (PX4_ISFINITE(lat_status.lateral_acceleration_ff)) {
				_last_lat_accel = lat_status.lateral_acceleration_ff;
			}
		}

		if (_vehicle_local_position_sub.update(&lpos)) {
			const float vx = lpos.vx;
			const float vy = lpos.vy;
			const float gsp = sqrtf(vx * vx + vy * vy);

			if (PX4_ISFINITE(gsp)) {
				_last_ground_speed = gsp;
			}
		}

		// Compute forward speed command
		float v_for_cmd = _last_ground_speed;

		if (!PX4_ISFINITE(v_for_cmd) || (fabsf(v_for_cmd) < V_CMD_MIN)) {
			v_for_cmd = V_CMD_MIN;
		}

		// Compute yaw rate command from lateral acceleration: a_y = v * omega
		float omega_cmd = 0.f;

		if (PX4_ISFINITE(_last_lat_accel) && (fabsf(v_for_cmd) > 1e-3f)) {
			omega_cmd = _last_lat_accel / v_for_cmd;
		}

		// Limit yaw rate
		if (!PX4_ISFINITE(omega_cmd)) {
			omega_cmd = 0.f;
		}

		omega_cmd = math::constrain(omega_cmd, -OMEGA_MAX, OMEGA_MAX);

		send_cmd(v_for_cmd, omega_cmd);

	} else {
		send_cmd(0.f, 0.f);
	}
}

int RoverArduinoBridge::task_spawn(int argc, char *argv[])
{
	RoverArduinoBridge *instance = new RoverArduinoBridge();

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

int RoverArduinoBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverArduinoBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Bridge fixed-wing lateral guidance (NPFG / L1 / PID) to an Arduino over UART.

It listens to fixed_wing_lateral_guidance_status (lateral acceleration demand)
and vehicle_local_position (ground speed) and emits a simple ASCII line:

  CMD,v_mps,omega_rad_s

at 50 Hz on the configured UART. The Arduino then drives the steering
servo and motor using v and omega.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_arduino_bridge", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_arduino_bridge_main(int argc, char *argv[])
{
	return RoverArduinoBridge::main(argc, argv);
}
