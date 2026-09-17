############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_module(
	MODULE modules__fw_rate_control
	MAIN fw_rate_control
	SRCS
		FixedwingRateControl.cpp
		FixedwingRateControl.hpp
		AdaptiveTailTrimShadow.cpp
	DEPENDS
		px4_work_queue
		RateControl
		SlewRate
	)

px4_add_unit_gtest(
	SRC tests/FixedB0ExperimentTest.cpp
)

px4_add_unit_gtest(
	SRC tests/BumplessRollITransferTest.cpp
	LINKLIBS RateControl
)

option(ADAPTIVE_TAIL_TRIM_CONTRACT_RED "Opt in to intentionally RED future contracts" OFF)
px4_add_unit_gtest(SRC tests/AdaptiveTailTrimCoreTest.cpp LINKLIBS RateControl)
px4_add_unit_gtest(SRC tests/AdaptiveTailTrimShadowTest.cpp LINKLIBS RateControl)
if(BUILD_TESTING)
	target_sources(unit-AdaptiveTailTrimShadow PRIVATE AdaptiveTailTrimShadow.cpp)
endif()
px4_add_functional_gtest(SRC tests/AllocatorSaturationFeedbackTest.cpp LINKLIBS RateControl)
if(BUILD_TESTING)
	# Compile the actual routing block, not a separately maintained copy.
	set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS FixedwingRateControl.cpp)
	file(READ "${CMAKE_CURRENT_SOURCE_DIR}/FixedwingRateControl.cpp" routing_source)
	string(FIND "${routing_source}" "// saturation handling for axis controlled by differential thrust" routing_start)
	string(FIND "${routing_source}" "const float airspeed_scale_squared" routing_end)
	if(routing_start LESS 0 OR routing_end LESS routing_start)
		message(FATAL_ERROR "Allocator routing test extraction anchors missing")
	endif()
	math(EXPR routing_length "${routing_end} - ${routing_start}")
	string(SUBSTRING "${routing_source}" ${routing_start} ${routing_length} ALLOCATOR_ROUTING_BODY)
	configure_file(tests/AllocatorRouting.inc.in AllocatorRouting.inc @ONLY)
	target_include_directories(functional-AllocatorSaturationFeedback PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
	string(FIND "${routing_source}" "const Vector3f gains_before_update" trim_start)
	string(FIND "${routing_source}" "/* throttle passed through" trim_end)
	if(trim_start LESS 0 OR trim_end LESS trim_start)
		message(FATAL_ERROR "Shadow isolation test production extraction anchors missing")
	endif()
	math(EXPR trim_length "${trim_end} - ${trim_start}")
	string(SUBSTRING "${routing_source}" ${trim_start} ${trim_length} TAIL_TRIM_ACTUAL_COMPOSITION)
	configure_file(tests/TailTrimActualComposition.inc.in TailTrimActualComposition.inc @ONLY)
	target_include_directories(unit-AdaptiveTailTrimShadow PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
endif()
if(ADAPTIVE_TAIL_TRIM_CONTRACT_RED)
	px4_add_unit_gtest(SRC tests/AdaptiveTailTrimContractTest.cpp LINKLIBS RateControl)
endif()
