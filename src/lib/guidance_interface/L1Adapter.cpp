#include "L1Adapter.hpp"
#include <matrix/math.hpp>
#include <math.h>
#include <stdint.h>

L1Adapter::L1Adapter()
{
    // 使用L1控制器的默认参数
    _l1_period = 25.0f;
    _l1_damping = 0.75f;
    _l1_ratio = 1.0f / M_PI_F * _l1_damping * _l1_period;
    _k_l1 = 4.0f * _l1_damping * _l1_damping;
    _roll_lim_rad = math::radians(30.0f);
}

GuidanceOutput L1Adapter::guideToPath(const matrix::Vector2f &curr_pos_local,
                                     const matrix::Vector2f &ground_vel,
                                     const matrix::Vector2f &wind_vel,
                                     const matrix::Vector2f &unit_path_tangent,
                                     const matrix::Vector2f &closest_point_on_path,
                                     const float path_curvature)
{
    GuidanceOutput output;
    
    // 简化的L1算法实现
    float ground_speed = math::max(ground_vel.length(), 0.1f);
    
    // 计算L1距离
    _l1_distance = _l1_ratio * ground_speed;
    
    // 计算航向误差
    float current_heading = atan2f(ground_vel(1), ground_vel(0));
    float target_heading = atan2f(unit_path_tangent(1), unit_path_tangent(0));
    float heading_error = target_heading - current_heading;
    
    // 归一化角度
    while (heading_error > M_PI_F) heading_error -= 2.0f * M_PI_F;
    while (heading_error < -M_PI_F) heading_error += 2.0f * M_PI_F;
    
    // 限制航向误差到±90度
    heading_error = math::constrain(heading_error, -M_PI_F / 2.0f, M_PI_F / 2.0f);
    
    // 计算横向加速度
    _lateral_accel = _k_l1 * ground_speed * ground_speed / _l1_distance * sinf(heading_error);
    
    // 更新状态
    _nav_bearing = target_heading;
    _bearing_error = heading_error;
    _target_bearing = target_heading;
    _crosstrack_error = (curr_pos_local - closest_point_on_path).length();
    
    // 计算滚转角设定
    _roll_setpoint = atanf(_lateral_accel / 9.81f);
    _roll_setpoint = math::constrain(_roll_setpoint, -_roll_lim_rad, _roll_lim_rad);
    
    // 设置输出
    output.course_setpoint = _nav_bearing;
    output.lateral_acceleration_feedforward = _lateral_accel;
    
    return output;
}

float L1Adapter::controlHeading(float heading_setpoint, float current_heading, float airspeed)
{
    // 简化的L1航向控制
    float heading_error = heading_setpoint - current_heading;
    
    // 归一化角度
    while (heading_error > M_PI_F) heading_error -= 2.0f * M_PI_F;
    while (heading_error < -M_PI_F) heading_error += 2.0f * M_PI_F;
    
    // 限制航向误差到±90度
    heading_error = math::constrain(heading_error, -M_PI_F / 2.0f, M_PI_F / 2.0f);
    
    // 计算横向加速度
    float lateral_accel = _k_l1 * airspeed * airspeed / _l1_distance * sinf(heading_error);
    
    return lateral_accel;
}

float L1Adapter::mapCourseSetpointToHeadingSetpoint(float course_setpoint,
                                                   const matrix::Vector2f &wind_speed,
                                                   float airspeed_eas)
{
    // L1控制器通常不需要风补偿，直接返回航向设定
    return course_setpoint;
}

float L1Adapter::getMinAirspeedForCurrentBearing(float course_setpoint,
                                                const matrix::Vector2f &wind_speed,
                                                float max_true_airspeed,
                                                float min_ground_speed)
{
    // L1控制器使用固定的最小空速
    return fmaxf(min_ground_speed, 5.0f);
}

float L1Adapter::getCourseSetpoint() const
{
    return _nav_bearing;
}

float L1Adapter::getLateralAccelerationSetpoint() const
{
    return _lateral_accel;
}

float L1Adapter::getBearingFeasibility() const
{
    // L1控制器总是可行的
    return 1.0f;
}

float L1Adapter::getBearingFeasibilityOnTrack() const
{
    return 1.0f;
}

float L1Adapter::getSignedTrackError() const
{
    return _crosstrack_error;
}

float L1Adapter::getTrackErrorBound() const
{
    // L1控制器的航向误差边界
    return math::radians(90.0f);
}

float L1Adapter::getAdaptedPeriod() const
{
    return _l1_period;
}

float L1Adapter::switchDistance(float wp_radius) const
{
    return math::min(wp_radius, _l1_distance);
}

void L1Adapter::navigateWaypoints(const matrix::Vector2f &waypoint_A, 
                                  const matrix::Vector2f &waypoint_B,
                                  const matrix::Vector2f &current_position,
                                  const matrix::Vector2f &ground_speed)
{
    _current_waypoint_A = waypoint_A;
    _current_waypoint_B = waypoint_B;
    _waypoint_mode_active = true;
    _loiter_mode_active = false;
    
    // 简化的航点导航实现
    matrix::Vector2f path_vector = waypoint_B - waypoint_A;
    matrix::Vector2f to_current = current_position - waypoint_A;
    
    // 计算航向设定点
    _target_bearing = atan2f(path_vector(1), path_vector(0));
    _nav_bearing = _target_bearing;
}

void L1Adapter::navigateLoiter(const matrix::Vector2f &center,
                               const matrix::Vector2f &current_position,
                               float radius,
                               int8_t loiter_direction,
                               const matrix::Vector2f &ground_speed)
{
    _current_waypoint_A = center;
    _loiter_radius = radius;
    _loiter_direction = loiter_direction;
    _loiter_mode_active = true;
    _waypoint_mode_active = false;
    
    // 简化的盘旋导航实现
    matrix::Vector2f to_center = current_position - center;
    _target_bearing = atan2f(-to_center(1), -to_center(0));
    _nav_bearing = _target_bearing;
}

void L1Adapter::navigateHeading(float navigation_heading, 
                                float current_heading,
                                const matrix::Vector2f &ground_speed)
{
    _waypoint_mode_active = false;
    _loiter_mode_active = false;
    
    _target_bearing = navigation_heading;
    _nav_bearing = navigation_heading;
}

void L1Adapter::navigateLevelFlight(float current_heading)
{
    _waypoint_mode_active = false;
    _loiter_mode_active = false;
    
    _target_bearing = current_heading;
    _nav_bearing = current_heading;
    _lateral_accel = 0.0f;
}

void L1Adapter::setL1Period(float period)
{
    _l1_period = period;
    _l1_ratio = 1.0f / M_PI_F * _l1_damping * _l1_period;
}

void L1Adapter::setL1Damping(float damping)
{
    _l1_damping = damping;
    _l1_ratio = 1.0f / M_PI_F * _l1_damping * _l1_period;
    _k_l1 = 4.0f * _l1_damping * _l1_damping;
}

void L1Adapter::setL1RollLimit(float roll_lim_rad)
{
    _roll_lim_rad = roll_lim_rad;
}

void L1Adapter::setRollSlewRate(float roll_slew_rate)
{
    // 简化的L1实现中暂不支持滚转角变化率限制
    (void)roll_slew_rate;
}

void L1Adapter::setDt(float dt)
{
    // 简化的L1实现中暂不支持时间步长设置
    (void)dt;
}

float L1Adapter::getNavBearing() const
{
    return _nav_bearing;
}

float L1Adapter::getLateralAccelerationDemand() const
{
    return _lateral_accel;
}

float L1Adapter::getBearingError() const
{
    return _bearing_error;
}

float L1Adapter::getTargetBearing() const
{
    return _target_bearing;
}

float L1Adapter::getRollSetpoint() const
{
    return _roll_setpoint;
}

float L1Adapter::getCrosstrackError() const
{
    return _crosstrack_error;
}

bool L1Adapter::reachedLoiterTarget() const
{
    return _circle_mode;
}

bool L1Adapter::circleMode() const
{
    return _circle_mode;
}

matrix::Vector2f L1Adapter::calculatePathTangent(const matrix::Vector2f &waypoint_A,
                                                const matrix::Vector2f &waypoint_B)
{
    matrix::Vector2f path_vector = waypoint_B - waypoint_A;
    return path_vector.normalized();
}

matrix::Vector2f L1Adapter::calculateClosestPointOnPath(const matrix::Vector2f &current_pos,
                                                       const matrix::Vector2f &waypoint_A,
                                                       const matrix::Vector2f &waypoint_B)
{
    matrix::Vector2f path_vector = waypoint_B - waypoint_A;
    matrix::Vector2f to_current = current_pos - waypoint_A;

    float t = (to_current * path_vector) / (path_vector * path_vector);
    t = math::constrain(t, 0.0f, 1.0f);

    return waypoint_A + t * path_vector;
}

float L1Adapter::calculatePathCurvature(const matrix::Vector2f &waypoint_A,
                                       const matrix::Vector2f &waypoint_B)
{
    // 简化的路径曲率计算，对于直线路径返回0
    return 0.0f;
}
