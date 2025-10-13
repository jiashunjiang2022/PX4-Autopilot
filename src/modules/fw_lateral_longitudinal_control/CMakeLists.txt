set(CONTROL_DEPENDENCIES
        npfg
        tecs
        guidance_interface  # 新增
)


px4_add_module(
        MODULE modules__fw_lateral_longitudinal_control
        MAIN fw_lat_lon_control
        SRCS
        FwLateralLongitudinalControl.cpp
        FwLateralLongitudinalControl.hpp
        fw_lat_long_params.c
        DEPENDS
        ${CONTROL_DEPENDENCIES}
)
