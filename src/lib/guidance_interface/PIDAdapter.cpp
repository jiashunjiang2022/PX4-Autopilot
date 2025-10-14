#include "PIDAdapter.hpp"
#include <px4_platform_common/time.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <math.h>
#include <stdint.h>

PIDAdapter::PIDAdapter()
{
    _last_time = hrt_absolute_time();
}

GuidanceOutput PIDAdapter::guideToPath(const matrix::Vector2f &curr_pos_local,
                                     const matrix::Vector2f &ground_vel,
                                     const matrix::Vector2f &wind_vel,
                                     const matrix::Vector2f &unit_path_tangent,
                                     const matrix::Vector2f &closest_point_on_path,
                                     const float path_curvature)
{
    GuidanceOutput output;

    float ground_speed = ground_vel.length();

    // 低速保护
    if (ground_speed < 2.0f) {
        output.course_setpoint = atan2f(unit_path_tangent(1), unit_path_tangent(0));
        output.lateral_acceleration_feedforward = 0.0f;
        return output;
    }

    // 计算轨迹误差（垂直于路径的距离）
    matrix::Vector2f position_error = curr_pos_local - closest_point_on_path;

    // 计算轨迹误差的符号（左侧为正，右侧为负）
    matrix::Vector2f path_normal(-unit_path_tangent(1), unit_path_tangent(0));
    float signed_track_error = position_error.dot(path_normal);

    // 当前航向
    float current_course = atan2f(ground_vel(1), ground_vel(0));

    hrt_abstime current_time = hrt_absolute_time();
    float dt = (current_time - _last_time) / 1e6f; // 转换为秒

    // 初始化或时间跳变保护
    if (_last_time == 0 || dt <= 0.0f || dt > 1.0f) {
        _last_time = current_time;
        output.course_setpoint = atan2f(unit_path_tangent(1), unit_path_tangent(0));
        output.lateral_acceleration_feedforward = 0.0f;
        return output;
    }

    _last_time = current_time;

    // 计算到最近点的距离
    float distance_to_path = (curr_pos_local - closest_point_on_path).length();
    
    // 自适应前视距离：基于地速和轨迹误差
    // 当靠近路径时减小前视距离，远离时增大
    float base_lookahead = 20.0f; // 基础前视距离
    float lookahead_distance = math::constrain(
        base_lookahead + distance_to_path * 0.5f,
        10.0f,  // 最小前视距离
        50.0f   // 最大前视距离
    );
    
    // 计算前视点
    matrix::Vector2f lookahead_point = closest_point_on_path + unit_path_tangent * lookahead_distance;
    matrix::Vector2f vehicle_to_lookahead = lookahead_point - curr_pos_local;
    
    // 如果距离太近，直接使用路径切线方向
    if (vehicle_to_lookahead.length() < 1.0f) {
        output.course_setpoint = atan2f(unit_path_tangent(1), unit_path_tangent(0));
        output.lateral_acceleration_feedforward = 0.0f;
        _current_course_setpoint = output.course_setpoint;
        _current_lateral_acceleration = 0.0f;
        _current_track_error = signed_track_error;
        return output;
    }
    
    float bearing_to_lookahead = atan2f(vehicle_to_lookahead(1), vehicle_to_lookahead(0));
    
    // 计算方位角误差（eta）
    float eta = normalizeAngle(bearing_to_lookahead - current_course);
    
    // 限制eta到±90度（与L1相同）
    eta = math::constrain(eta, -M_PI_F / 2.0f, M_PI_F / 2.0f);
    
    // 使用L1类似的公式计算横向加速度
    // a_lat = K * v^2 / L * sin(eta)
    float K_gain = 2.0f;
    float lateral_acceleration = K_gain * ground_speed * ground_speed / lookahead_distance * sinf(eta);
    
    // 更保守的横向加速度限制
    lateral_acceleration = math::constrain(lateral_acceleration, -3.0f, 3.0f);

    // 输出目标航向（方位角）
    output.course_setpoint = bearing_to_lookahead;
    output.lateral_acceleration_feedforward = lateral_acceleration;

    _current_course_setpoint = bearing_to_lookahead;
    _current_lateral_acceleration = lateral_acceleration;
    _current_track_error = signed_track_error;

    return output;
}

float PIDAdapter::controlHeading(float heading_setpoint, float current_heading, float airspeed)
{
    float heading_error = normalizeAngle(heading_setpoint - current_heading);

    hrt_abstime current_time = hrt_absolute_time();
    float dt = (current_time - _last_time) / 1e6f;

    if (dt > 0.0f && dt < 1.0f) {
        _heading_integral += heading_error * dt;
        float heading_derivative = (heading_error - _heading_error_prev) / dt;

        float heading_rate = _heading_kp * heading_error +
                           _heading_ki * _heading_integral +
                           _heading_kd * heading_derivative;

        _heading_error_prev = heading_error;

        return heading_rate;
    }

    return 0.0f;
}

float PIDAdapter::mapCourseSetpointToHeadingSetpoint(float course_setpoint,
                                                   const matrix::Vector2f &wind_speed,
                                                   float airspeed_eas)
{
    // 简单的风补偿
    return course_setpoint + calculateWindCompensation(course_setpoint, wind_speed, airspeed_eas);
}

float PIDAdapter::getMinAirspeedForCurrentBearing(float course_setpoint,
                                                const matrix::Vector2f &wind_speed,
                                                float max_true_airspeed,
                                                float min_ground_speed)
{
    // 简化的最小空速计算
    float wind_effect = wind_speed.length() * cosf(course_setpoint - atan2f(wind_speed(1), wind_speed(0)));
    return fmaxf(min_ground_speed - wind_effect, 5.0f); // 最小5m/s
}

float PIDAdapter::getCourseSetpoint() const
{
    return _current_course_setpoint;
}

float PIDAdapter::getLateralAccelerationSetpoint() const
{
    return _current_lateral_acceleration;
}

float PIDAdapter::getBearingFeasibility() const
{
    return _current_bearing_feasibility;
}

float PIDAdapter::getBearingFeasibilityOnTrack() const
{
    return _current_bearing_feasibility;
}

float PIDAdapter::getSignedTrackError() const
{
    return _current_track_error;
}

float PIDAdapter::getTrackErrorBound() const
{
    return 10.0f; // 固定值
}

float PIDAdapter::getAdaptedPeriod() const
{
    return 1.0f; // 固定值
}

float PIDAdapter::switchDistance(float wp_radius) const
{
    return wp_radius * 2.0f; // 简化的切换距离
}

void PIDAdapter::setCoursePIDParams(float kp, float ki, float kd)
{
    _course_kp = kp;
    _course_ki = ki;
    _course_kd = kd;
}

void PIDAdapter::setHeadingPIDParams(float kp, float ki, float kd)
{
    _heading_kp = kp;
    _heading_ki = ki;
    _heading_kd = kd;
}

void PIDAdapter::reset()
{
    _course_integral = 0.0f;
    _course_error_prev = 0.0f;
    _heading_integral = 0.0f;
    _heading_error_prev = 0.0f;
    _current_course_setpoint = NAN;
    _current_lateral_acceleration = NAN;
    _current_track_error = NAN;
    _current_bearing_feasibility = 1.0f;
}

float PIDAdapter::normalizeAngle(float angle)
{
    while (angle > M_PI_F) angle -= 2.0f * M_PI_F;
    while (angle < -M_PI_F) angle += 2.0f * M_PI_F;
    return angle;
}

float PIDAdapter::calculateWindCompensation(float course_setpoint, const matrix::Vector2f &wind_speed, float airspeed)
{
    // 简化的风补偿计算
    float wind_angle = atan2f(wind_speed(1), wind_speed(0));
    float wind_effect = wind_speed.length() * sinf(course_setpoint - wind_angle) / airspeed;
    return wind_effect;
}
