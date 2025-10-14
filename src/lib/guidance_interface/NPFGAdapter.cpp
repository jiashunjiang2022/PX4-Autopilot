#include "NPFGAdapter.hpp"
#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <math.h>
#include <px4_platform_common/defines.h>

NPFGAdapter::NPFGAdapter()
{
    // NPFG对象使用默认构造函数
}

GuidanceOutput NPFGAdapter::guideToPath(const matrix::Vector2f &curr_pos_local,
                                      const matrix::Vector2f &ground_vel,
                                      const matrix::Vector2f &wind_vel,
                                      const matrix::Vector2f &unit_path_tangent,
                                      const matrix::Vector2f &closest_point_on_path,
                                      const float path_curvature)
{
    GuidanceOutput output;

    // 检查输入有效性
    if (!PX4_ISFINITE(curr_pos_local(0)) || !PX4_ISFINITE(curr_pos_local(1)) ||
        !PX4_ISFINITE(ground_vel(0)) || !PX4_ISFINITE(ground_vel(1)) ||
        ground_vel.length() < 0.1f) {
        // 无效输入，返回安全的默认值
        output.course_setpoint = atan2f(unit_path_tangent(1), unit_path_tangent(0));
        output.lateral_acceleration_feedforward = 0.0f;
        return output;
    }

    // 使用NPFG进行路径制导
    DirectionalGuidanceOutput npfg_output = _directional_guidance.guideToPath(curr_pos_local, ground_vel, wind_vel,
                                    unit_path_tangent, closest_point_on_path, path_curvature);

    // 获取制导输出并限制
    output.course_setpoint = npfg_output.course_setpoint;
    output.lateral_acceleration_feedforward = math::constrain(npfg_output.lateral_acceleration_feedforward, -5.0f, 5.0f);

    // 检查输出有效性
    if (!PX4_ISFINITE(output.course_setpoint)) {
        output.course_setpoint = atan2f(unit_path_tangent(1), unit_path_tangent(0));
    }
    if (!PX4_ISFINITE(output.lateral_acceleration_feedforward)) {
        output.lateral_acceleration_feedforward = 0.0f;
    }

    return output;
}

float NPFGAdapter::controlHeading(float heading_setpoint, float current_heading, float airspeed)
{
    // 简化的航向控制实现
    float heading_error = heading_setpoint - current_heading;
    // 归一化角度
    while (heading_error > M_PI_F) heading_error -= 2.0f * M_PI_F;
    while (heading_error < -M_PI_F) heading_error += 2.0f * M_PI_F;

    // 简单的比例控制
    return heading_error * 0.5f; // 返回航向角速度
}

float NPFGAdapter::mapCourseSetpointToHeadingSetpoint(float course_setpoint,
                                                     const matrix::Vector2f &wind_speed,
                                                     float airspeed_eas)
{
    // 简化的航向映射，考虑风的影响
    float wind_angle = atan2f(wind_speed(1), wind_speed(0));
    float wind_effect = wind_speed.length() * sinf(course_setpoint - wind_angle) / airspeed_eas;
    return course_setpoint + wind_effect;
}

float NPFGAdapter::getMinAirspeedForCurrentBearing(float course_setpoint,
                                                  const matrix::Vector2f &wind_speed,
                                                  float max_true_airspeed,
                                                  float min_ground_speed)
{
    // 简化的最小空速计算
    float wind_effect = wind_speed.length() * cosf(course_setpoint - atan2f(wind_speed(1), wind_speed(0)));
    return fmaxf(min_ground_speed - wind_effect, 5.0f); // 最小5m/s
}

float NPFGAdapter::getCourseSetpoint() const
{
    return _directional_guidance.getCourseSetpoint();
}

float NPFGAdapter::getLateralAccelerationSetpoint() const
{
    return _directional_guidance.getLateralAccelerationSetpoint();
}

float NPFGAdapter::getBearingFeasibility() const
{
    return _directional_guidance.getBearingFeasibility();
}

float NPFGAdapter::getBearingFeasibilityOnTrack() const
{
    return _directional_guidance.getBearingFeasibilityOnTrack();
}

float NPFGAdapter::getSignedTrackError() const
{
    return _directional_guidance.getSignedTrackError();
}

float NPFGAdapter::getTrackErrorBound() const
{
    return _directional_guidance.getTrackErrorBound();
}

float NPFGAdapter::getAdaptedPeriod() const
{
    return _directional_guidance.getAdaptedPeriod();
}

float NPFGAdapter::switchDistance(float wp_radius) const
{
    return _directional_guidance.switchDistance(wp_radius);
}
