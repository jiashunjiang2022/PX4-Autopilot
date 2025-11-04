############################################################################
#
#   PX4 flap_rpm_controller example module
#
############################################################################

px4_add_module(
    MODULE examples__flap_rpm_controller
    MAIN flap_rpm_controller
    SRCS
        flap_rpm_controller.cpp
)

