px4_add_module(
	MODULE modules__flap_aug_shadow
	MAIN flap_aug_shadow
	SRCS
		flap_aug_shadow.cpp
		fast_predictor.cpp
		slow_engineering_shadow.cpp
	DEPENDS
		px4_work_queue
)

px4_add_unit_gtest(
	SRC tests/FlapAugShadowTest.cpp
	EXTRA_SRCS
		fast_predictor.cpp
		slow_engineering_shadow.cpp
)
