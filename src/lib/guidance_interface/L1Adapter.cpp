#include "L1Adapter.hpp"
#include <matrix/math.hpp>
#include <math.h>
#include <stdint.h>

L1Adapter::L1Adapter()
{
    // 使用L1控制器的默认参数
    _l1_controller.set_l1_period(25.0f);
    _l1_controller.set_l1_damping(0.75f);
    _l1_controller.set_l1_roll_limit(math::radians(30.0f));
}

GuidanceOutput L1Adapter::guideToPath(const matrix::Vector2f &curr_pos_local,
                                     const matrix::Vector2f &ground_vel,
                                     const matrix::Vector2f &wind_vel,
                                     const matrix::Vector2f &unit_path_tangent,
                                     const matrix::Vector2f &closest_point_on_path,
                                     const float path_curvature)
{
    GuidanceOutput output;
    
    if (_waypoint_mode_active) {
        // 使用L1控制器进行航点导航
        _l1_controller.navigate_waypoints(_current_waypoint_A, _current_waypoint_B,
                                        curr_pos_local, ground_vel);
        
        output.course_setpoint = _l1_controller.nav_bearing();
        output.lateral_acceleration_feedforward = _l1_controller.nav_lateral_acceleration_demand();
        
    } else if (_loiter_mode_active) {
        // 使用L1控制器进行盘旋
        _l1_controller.navigate_loiter(_current_waypoint_A, curr_pos_local,
                                    _loiter_radius, _loiter_direction, ground_vel);
        
        output.course_setpoint = _l1_controller.nav_bearing();
        output.lateral_acceleration_feedforward = _l1_controller.nav_lateral_acceleration_demand();
        
    } else {
        // 默认航向保持模式 - 使用路径切线作为目标航向
        float current_heading = atan2f(ground_vel(1), ground_vel(0));
        float target_heading = atan2f(unit_path_tangent(1), unit_path_tangent(0));
        
        _l1_controller.navigate_heading(target_heading, current_heading, ground_vel);
        
        output.course_setpoint = _l1_controller.nav_bearing();
        output.lateral_acceleration_feedforward = _l1_controller.nav_lateral_acceleration_demand();
    }
    
    return output;
}

float L1Adapter::controlHeading(float heading_setpoint, float current_heading, float airspeed)
{
    // 使用L1控制器的航向保持
    matrix::Vector2f ground_speed(airspeed * cosf(current_heading),
                                 airspeed * sinf(current_heading));

    _l1_controller.navigate_heading(heading_setpoint, current_heading, ground_speed);

    return _l1_controller.nav_lateral_acceleration_demand();
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
    return _l1_controller.nav_bearing();
}

float L1Adapter::getLateralAccelerationSetpoint() const
{
    return _l1_controller.nav_lateral_acceleration_demand();
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
    return _l1_controller.crosstrack_error();
}

float L1Adapter::getTrackErrorBound() const
{
    // L1控制器的航向误差边界
    return math::radians(90.0f);
}

float L1Adapter::getAdaptedPeriod() const
{
    return 25.0f; // L1控制器的默认周期
}

float L1Adapter::switchDistance(float wp_radius) const
{
    return _l1_controller.switch_distance(wp_radius);
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

    _l1_controller.navigate_waypoints(waypoint_A, waypoint_B, current_position, ground_speed);
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

    _l1_controller.navigate_loiter(center, current_position, radius, loiter_direction, ground_speed);
}

void L1Adapter::navigateHeading(float navigation_heading,
                                float current_heading,
                                const matrix::Vector2f &ground_speed)
{
    _waypoint_mode_active = false;
    _loiter_mode_active = false;

    _l1_controller.navigate_heading(navigation_heading, current_heading, ground_speed);
}

void L1Adapter::navigateLevelFlight(float current_heading)
{
    _waypoint_mode_active = false;
    _loiter_mode_active = false;

    _l1_controller.navigate_level_flight(current_heading);
}

void L1Adapter::setL1Period(float period)
{
    _l1_controller.set_l1_period(period);
}

void L1Adapter::setL1Damping(float damping)
{
    _l1_controller.set_l1_damping(damping);
}

void L1Adapter::setL1RollLimit(float roll_lim_rad)
{
    _l1_controller.set_l1_roll_limit(roll_lim_rad);
}

void L1Adapter::setRollSlewRate(float roll_slew_rate)
{
    _l1_controller.set_roll_slew_rate(roll_slew_rate);
}

void L1Adapter::setDt(float dt)
{
    _l1_controller.set_dt(dt);
}

float L1Adapter::getNavBearing() const
{
    return _l1_controller.nav_bearing();
}

float L1Adapter::getLateralAccelerationDemand() const
{
    return _l1_controller.nav_lateral_acceleration_demand();
}

float L1Adapter::getBearingError() const
{
    return _l1_controller.bearing_error();
}

float L1Adapter::getTargetBearing() const
{
    return _l1_controller.target_bearing();
}

float L1Adapter::getRollSetpoint() const
{
    return _l1_controller.get_roll_setpoint();
}

float L1Adapter::getCrosstrackError() const
{
    return _l1_controller.crosstrack_error();
}

bool L1Adapter::reachedLoiterTarget() const
{
    return _l1_controller.reached_loiter_target();
}

bool L1Adapter::circleMode() const
{
    return _l1_controller.circle_mode();
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
