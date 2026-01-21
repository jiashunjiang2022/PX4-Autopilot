px4_add_module(
	MODULE modules__wing_phase
	MAIN wing_phase
	COMPILE_FLAGS
		-Wno-cast-align
	SRCS
		WingPhase.cpp
	DEPENDS
		px4_work_queue
	)
