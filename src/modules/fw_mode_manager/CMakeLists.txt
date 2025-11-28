px4_add_module(
	MODULE modules__fw_mode_manager
	MAIN fw_mode_manager
	SRCS
		FixedWingModeManager.cpp
		FixedWingModeManager.hpp
	DEPENDS
		launchdetection
		npfg
		runway_takeoff
		SlewRate
		pid
)

