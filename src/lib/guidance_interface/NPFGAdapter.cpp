#include "NPFGAdapter.hpp"
#include <matrix/math.hpp>
#include <math.h>

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
    
    // 使用NPFG进行路径制导
    matrix::Vector2f path_tangent = unit_path_tangent;
    matrix::Vector2f path_curvature_vector = path_curvature * matrix::Vector2f(-unit_path_tangent(1), unit_path_tangent(0));
    
    // 调用NPFG的路径制导
    _directional_guidance.guideToPath(curr_pos_local, ground_vel, wind_vel, 
                                    path_tangent, path_curvature_vector);
    
    // 获取制导输出
    output.course_setpoint = _directional_guidance.getCourseSetpoint();
    output.lateral_acceleration_feedforward = _directional_guidance.getLateralAccelerationSetpoint();
    
    return output;
}

float NPFGAdapter::controlHeading(float heading_setpoint, float current_heading, float airspeed)
{
    // 使用NPFG的航向控制
    return _directional_guidance.controlHeading(heading_setpoint, current_heading, airspeed);
}

float NPFGAdapter::mapCourseSetpointToHeadingSetpoint(float course_setpoint,
                                                     const matrix::Vector2f &wind_speed,
                                                     float airspeed_eas)
{
    // 使用NPFG的航向映射
    return _course_to_airspeed.mapCourseSetpointToHeadingSetpoint(course_setpoint, wind_speed, airspeed_eas);
}

float NPFGAdapter::getMinAirspeedForCurrentBearing(float course_setpoint,
                                                  const matrix::Vector2f &wind_speed,
                                                  float max_true_airspeed,
                                                  float min_ground_speed)
{
    // 使用NPFG的最小空速计算
    return _course_to_airspeed.getMinAirspeedForCurrentBearing(course_setpoint, wind_speed, 
                                                              max_true_airspeed, min_ground_speed);
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
