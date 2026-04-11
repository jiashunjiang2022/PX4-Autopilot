px4_add_library(wing_phase_math
	WingPhaseMath.cpp
)

px4_add_module(
	MODULE modules__wing_phase
	MAIN wing_phase
	COMPILE_FLAGS
		-Wno-cast-align
	SRCS
		WingPhase.cpp
	DEPENDS
		px4_work_queue
		wing_phase_math
)

px4_add_unit_gtest(SRC WingPhaseMathTest.cpp LINKLIBS wing_phase_math)
