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

    // 计算轨迹误差（垂直于路径的距离）
    matrix::Vector2f position_error = curr_pos_local - closest_point_on_path;
    
    // 计算轨迹误差的符号（左侧为正，右侧为负）
    matrix::Vector2f path_normal(-unit_path_tangent(1), unit_path_tangent(0));
    float signed_track_error = position_error.dot(path_normal);

    // 目标航向（路径切线方向）
    float path_course = atan2f(unit_path_tangent(1), unit_path_tangent(0));

    // 当前航向
    float current_course = atan2f(ground_vel(1), ground_vel(0));
    float ground_speed = ground_vel.length();

    // 计算航向误差
    float course_error = normalizeAngle(path_course - current_course);

    hrt_abstime current_time = hrt_absolute_time();
    float dt = (current_time - _last_time) / 1e6f; // 转换为秒
    _last_time = current_time;

    if (dt > 0.0f && dt < 1.0f && ground_speed > 1.0f) { // 防止异常时间间隔和低速
        // 限制积分项防止积分饱和
        _course_integral = math::constrain(_course_integral + course_error * dt, -10.0f, 10.0f);

        float course_derivative = (course_error - _course_error_prev) / dt;
        _course_error_prev = course_error;

        // 计算航向修正（考虑轨迹误差）
        float track_correction = math::constrain(signed_track_error * 0.1f, -0.5f, 0.5f); // 轨迹误差修正
        float course_correction = _course_kp * course_error +
                                _course_ki * _course_integral +
                                _course_kd * course_derivative +
                                track_correction;

        // 限制航向修正角度
        course_correction = math::constrain(course_correction, -math::radians(45.0f), math::radians(45.0f));

        // 计算目标航向（加上修正）
        float target_course = path_course + course_correction;

        // 计算横向加速度（基于航向修正和地速）
        // 使用向心加速度公式: a = v^2 / R, 其中 R = v / (dψ/dt)
        // 简化为: a = v * (dψ/dt)
        float lateral_acceleration = ground_speed * course_correction;

        // 限制横向加速度（典型值：2-5 m/s^2）
        lateral_acceleration = math::constrain(lateral_acceleration, -5.0f, 5.0f);

        output.course_setpoint = target_course;
        output.lateral_acceleration_feedforward = lateral_acceleration;

        _current_course_setpoint = target_course;
        _current_lateral_acceleration = lateral_acceleration;
        _current_track_error = signed_track_error;
    } else {
        // 低速或异常情况，保持当前航向
        output.course_setpoint = path_course;
        output.lateral_acceleration_feedforward = 0.0f;
    }

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
