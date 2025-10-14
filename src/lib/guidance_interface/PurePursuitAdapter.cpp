#include "PurePursuitAdapter.hpp"
#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <math.h>
#include <px4_platform_common/defines.h>

PurePursuitAdapter::PurePursuitAdapter()
{
    // 使用默认参数
}

GuidanceOutput PurePursuitAdapter::guideToPath(const matrix::Vector2f &curr_pos_local,
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

    // 计算自适应前视距离
    // lookahead = gain * ground_speed，限制在min和max之间
    _lookahead_distance = math::constrain(_lookahead_gain * ground_speed, 
                                         _min_lookahead, _max_lookahead);

    // 计算前视点（在路径上距离closest_point前方lookahead_distance的点）
    matrix::Vector2f lookahead_point = closest_point_on_path + unit_path_tangent * _lookahead_distance;

    // 从飞机到前视点的向量
    matrix::Vector2f vehicle_to_lookahead = lookahead_point - curr_pos_local;
    float distance_to_lookahead = vehicle_to_lookahead.length();

    // 如果距离太近，直接使用路径切线方向
    if (distance_to_lookahead < 1.0f) {
        output.course_setpoint = atan2f(unit_path_tangent(1), unit_path_tangent(0));
        output.lateral_acceleration_feedforward = 0.0f;
        _current_course_setpoint = output.course_setpoint;
        _current_lateral_acceleration = 0.0f;
        _current_track_error = 0.0f;
        return output;
    }

    // 计算期望航向（从飞机指向前视点）
    float desired_course = atan2f(vehicle_to_lookahead(1), vehicle_to_lookahead(0));

    // 当前航向
    float current_course = atan2f(ground_vel(1), ground_vel(0));

    // 计算alpha角（飞机航向与前视点方向的夹角）
    float alpha = normalizeAngle(desired_course - current_course);

    // Pure Pursuit几何关系：曲率 = 2 * sin(alpha) / L
    // 其中L是前视距离
    float curvature = 2.0f * sinf(alpha) / distance_to_lookahead;

    // 横向加速度 = v^2 * curvature
    float lateral_acceleration = ground_speed * ground_speed * curvature;

    // 限制横向加速度
    lateral_acceleration = math::constrain(lateral_acceleration, -4.0f, 4.0f);

    // 计算轨迹误差
    matrix::Vector2f position_error = curr_pos_local - closest_point_on_path;
    matrix::Vector2f path_normal(-unit_path_tangent(1), unit_path_tangent(0));
    float signed_track_error = position_error.dot(path_normal);

    // 输出
    output.course_setpoint = desired_course;
    output.lateral_acceleration_feedforward = lateral_acceleration;

    // 保存状态
    _current_course_setpoint = desired_course;
    _current_lateral_acceleration = lateral_acceleration;
    _current_track_error = signed_track_error;

    return output;
}

float PurePursuitAdapter::controlHeading(float heading_setpoint, float current_heading, float airspeed)
{
    // 简单的航向控制
    float heading_error = normalizeAngle(heading_setpoint - current_heading);
    
    // 比例控制，输出横向加速度
    float lateral_accel = heading_error * airspeed * 0.5f;
    
    return math::constrain(lateral_accel, -3.0f, 3.0f);
}

float PurePursuitAdapter::mapCourseSetpointToHeadingSetpoint(float course_setpoint,
                                                            const matrix::Vector2f &wind_speed,
                                                            float airspeed_eas)
{
    // 简单的风补偿
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

float PurePursuitAdapter::getMinAirspeedForCurrentBearing(float course_setpoint,
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

float PurePursuitAdapter::switchDistance(float wp_radius) const
{
    // Pure Pursuit的切换距离基于前视距离
    return math::max(wp_radius, _lookahead_distance * 0.5f);
}

float PurePursuitAdapter::normalizeAngle(float angle)
{
    while (angle > M_PI_F) angle -= 2.0f * M_PI_F;
    while (angle < -M_PI_F) angle += 2.0f * M_PI_F;
    return angle;
}

