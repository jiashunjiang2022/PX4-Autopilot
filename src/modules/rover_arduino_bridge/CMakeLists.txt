############################################################################
#
#   Custom bridge from PX4 rover controllers to an Arduino over UART
#
############################################################################

px4_add_module(
	MODULE modules__rover_arduino_bridge
	MAIN rover_arduino_bridge
	SRCS
		rover_arduino_bridge.cpp
		rover_arduino_bridge.hpp
	DEPENDS
	)

