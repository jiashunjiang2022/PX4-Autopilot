#pragma once

#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <stdint.h>
#include <math.h>

// 统一的制导输出结构
struct GuidanceOutput {
    float course_setpoint{NAN};
    float lateral_acceleration_feedforward{NAN};
    float lateral_acceleration_total{NAN};
    float track_error{NAN};
    float course_error{NAN};
    float bearing_feasibility{NAN};
    float track_proximity{NAN};
    float adapted_period{NAN};
    uint8_t guidance_mode{0}; // 0=NPFG, 1=PID
};

// 统一的制导接口
class GuidanceInterface
{
public:
    virtual ~GuidanceInterface() = default;
    
    // 主要制导函数 - 与NPFG完全相同的接口
    virtual GuidanceOutput guideToPath(const matrix::Vector2f &curr_pos_local, 
                                     const matrix::Vector2f &ground_vel,
                                     const matrix::Vector2f &wind_vel,
                                     const matrix::Vector2f &unit_path_tangent,
                                     const matrix::Vector2f &closest_point_on_path,
                                     const float path_curvature) = 0;
    
    // 航向控制函数 - 与AirspeedDirectionController相同接口
    virtual float controlHeading(float heading_setpoint, float current_heading, float airspeed) = 0;
    
    // 航向映射函数 - 与CourseToAirspeedRefMapper相同接口
    virtual float mapCourseSetpointToHeadingSetpoint(float course_setpoint, 
                                                   const matrix::Vector2f &wind_speed,
                                                   float airspeed_eas) = 0;
    
    virtual float getMinAirspeedForCurrentBearing(float course_setpoint,
                                                const matrix::Vector2f &wind_speed,
                                                float max_true_airspeed,
                                                float min_ground_speed) = 0;
    
    // 状态获取函数
    virtual float getCourseSetpoint() const = 0;
    virtual float getLateralAccelerationSetpoint() const = 0;
    virtual float getBearingFeasibility() const = 0;
    virtual float getBearingFeasibilityOnTrack() const = 0;
    virtual float getSignedTrackError() const = 0;
    virtual float getTrackErrorBound() const = 0;
    virtual float getAdaptedPeriod() const = 0;
    virtual float switchDistance(float wp_radius) const = 0;
};