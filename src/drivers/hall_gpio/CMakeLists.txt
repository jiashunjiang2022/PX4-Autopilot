px4_add_module(
	MODULE drivers__hall_gpio
	MAIN hall_gpio
	COMPILE_FLAGS
		-Wno-cast-align
	SRCS
		HallGPIO.cpp
	DEPENDS
		px4_work_queue
	)
