#pragma once

#include "GuidanceInterface.hpp"
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>
#include <stdint.h>

class PIDAdapter : public GuidanceInterface
{
public:
    PIDAdapter();

    // 实现GuidanceInterface接口
    GuidanceOutput guideToPath(const matrix::Vector2f &curr_pos_local,
                             const matrix::Vector2f &ground_vel,
                             const matrix::Vector2f &wind_vel,
                             const matrix::Vector2f &unit_path_tangent,
                             const matrix::Vector2f &closest_point_on_path,
                             const float path_curvature) override;

    float controlHeading(float heading_setpoint, float current_heading, float airspeed) override;
    float mapCourseSetpointToHeadingSetpoint(float course_setpoint,
                                           const matrix::Vector2f &wind_speed,
                                           float airspeed_eas) override;
    float getMinAirspeedForCurrentBearing(float course_setpoint,
                                        const matrix::Vector2f &wind_speed,
                                        float max_true_airspeed,
                                        float min_ground_speed) override;

    // 状态获取函数
    float getCourseSetpoint() const override;
    float getLateralAccelerationSetpoint() const override;
    float getBearingFeasibility() const override;
    float getBearingFeasibilityOnTrack() const override;
    float getSignedTrackError() const override;
    float getTrackErrorBound() const override;
    float getAdaptedPeriod() const override;
    float switchDistance(float wp_radius) const override;

    // PID参数设置
    void setCoursePIDParams(float kp, float ki, float kd);
    void setHeadingPIDParams(float kp, float ki, float kd);
    void reset();

private:
    // PID控制器参数
    float _course_kp{2.0f};
    float _course_ki{0.1f};
    float _course_kd{0.5f};
    float _course_integral{0.0f};
    float _course_error_prev{0.0f};

    float _heading_kp{1.5f};
    float _heading_ki{0.05f};
    float _heading_kd{0.3f};
    float _heading_integral{0.0f};
    float _heading_error_prev{0.0f};

    // 状态变量
    float _current_course_setpoint{NAN};
    float _current_lateral_acceleration{NAN};
    float _current_track_error{NAN};
    float _current_bearing_feasibility{1.0f};

    hrt_abstime _last_time{0};

    // 辅助函数
    float normalizeAngle(float angle);
    float calculateWindCompensation(float course_setpoint, const matrix::Vector2f &wind_speed, float airspeed);
};
