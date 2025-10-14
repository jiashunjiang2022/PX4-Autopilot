#include "LOSAdapter.hpp"
#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <math.h>
#include <px4_platform_common/defines.h>

LOSAdapter::LOSAdapter()
{
    // 使用默认参数
}

GuidanceOutput LOSAdapter::guideToPath(const matrix::Vector2f &curr_pos_local,
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

    // 计算路径方向角（期望航向）
    float path_course = atan2f(unit_path_tangent(1), unit_path_tangent(0));

    // 计算横向误差（cross-track error）
    matrix::Vector2f position_error = curr_pos_local - closest_point_on_path;
    matrix::Vector2f path_normal(-unit_path_tangent(1), unit_path_tangent(0));
    float cross_track_error = position_error.dot(path_normal);

    // LOS修正角：chi_r = -atan(e / Delta)
    // 其中 e 是横向误差，Delta 是前视距离
    float course_correction = -atanf(cross_track_error / _lookahead_distance);

    // 限制修正角
    course_correction = math::constrain(course_correction, -_max_course_error, _max_course_error);

    // 期望航向 = 路径方向 + 修正角
    float desired_course = path_course + course_correction;

    // 侧风补偿（如果启用）
    float wind_correction = 0.0f;
    if (_enable_wind_compensation) {
        // 估算空速（假设飞机尽量保持航向）
        matrix::Vector2f airspeed_vec = ground_vel - wind_vel;
        float airspeed = airspeed_vec.length();
        
        if (airspeed > 3.0f) {
            wind_correction = calculateWindCorrectionAngle(wind_vel, unit_path_tangent, airspeed);
        }
    }

    // 最终航向设定点
    float course_setpoint = desired_course + wind_correction;

    // 当前航向
    float current_course = atan2f(ground_vel(1), ground_vel(0));

    // 计算航向误差
    float course_error = normalizeAngle(course_setpoint - current_course);

    // 计算横向加速度（简化的比例控制）
    // 使用 a = v^2 / R，其中 R 基于航向误差估算
    float turn_radius = _lookahead_distance / (2.0f * sinf(fabsf(course_error) / 2.0f + 0.01f));
    float lateral_acceleration = ground_speed * ground_speed / turn_radius;
    
    // 保持符号
    if (course_error < 0.0f) {
        lateral_acceleration = -lateral_acceleration;
    }

    // 限制横向加速度
    lateral_acceleration = math::constrain(lateral_acceleration, -4.0f, 4.0f);

    // 输出
    output.course_setpoint = course_setpoint;
    output.lateral_acceleration_feedforward = lateral_acceleration;

    // 保存状态
    _current_course_setpoint = course_setpoint;
    _current_lateral_acceleration = lateral_acceleration;
    _current_track_error = cross_track_error;

    return output;
}

float LOSAdapter::controlHeading(float heading_setpoint, float current_heading, float airspeed)
{
    // 简单的航向控制
    float heading_error = normalizeAngle(heading_setpoint - current_heading);
    
    // 比例控制，输出横向加速度
    float lateral_accel = heading_error * airspeed * 0.5f;
    
    return math::constrain(lateral_accel, -3.0f, 3.0f);
}

float LOSAdapter::mapCourseSetpointToHeadingSetpoint(float course_setpoint,
                                                    const matrix::Vector2f &wind_speed,
                                                    float airspeed_eas)
{
    // 风补偿已经在guideToPath中处理
    // 这里直接返回航向设定点
    if (airspeed_eas < 1.0f) {
        return course_setpoint;
    }

    float wind_angle = atan2f(wind_speed(1), wind_speed(0));
    float wind_speed_mag = wind_speed.length();
    
    // 计算侧风分量
    float crosswind = wind_speed_mag * sinf(course_setpoint - wind_angle);
    
    // 风修正角
    float wind_correction = asinf(math::constrain(crosswind / airspeed_eas, -1.0f, 1.0f));
    
    return course_setpoint + wind_correction;
}

float LOSAdapter::getMinAirspeedForCurrentBearing(float course_setpoint,
                                                 const matrix::Vector2f &wind_speed,
                                                 float max_true_airspeed,
                                                 float min_ground_speed)
{
    float wind_angle = atan2f(wind_speed(1), wind_speed(0));
    float wind_speed_mag = wind_speed.length();
    
    // 顺风分量
    float tailwind = wind_speed_mag * cosf(course_setpoint - wind_angle);
    
    // 所需空速 = 最小地速 - 顺风
    float required_airspeed = min_ground_speed - tailwind;
    
    return math::constrain(required_airspeed, 5.0f, max_true_airspeed);
}

float LOSAdapter::switchDistance(float wp_radius) const
{
    // LOS的切换距离基于前视距离
    return math::max(wp_radius, _lookahead_distance * 0.7f);
}

float LOSAdapter::normalizeAngle(float angle)
{
    while (angle > M_PI_F) angle -= 2.0f * M_PI_F;
    while (angle < -M_PI_F) angle += 2.0f * M_PI_F;
    return angle;
}

float LOSAdapter::calculateWindCorrectionAngle(const matrix::Vector2f &wind_vel,
                                              const matrix::Vector2f &unit_path_tangent,
                                              float airspeed)
{
    // 计算侧风分量（垂直于路径的风速）
    matrix::Vector2f path_normal(-unit_path_tangent(1), unit_path_tangent(0));
    float crosswind = wind_vel.dot(path_normal);

    // 风修正角：beta = asin(V_w_perp / V_a)
    // 其中 V_w_perp 是侧风分量，V_a 是空速
    if (airspeed < 1.0f) {
        return 0.0f;
    }

    float wind_correction = asinf(math::constrain(crosswind / airspeed, -1.0f, 1.0f));

    // 限制修正角
    return math::constrain(wind_correction, -M_PI_F / 6.0f, M_PI_F / 6.0f); // ±30度
}

