#pragma once

#include "GuidanceInterface.hpp"
#include <lib/ecl/ECL_L1_Pos_Controller.hpp>
#include <matrix/math.hpp>
#include <stdint.h>

class L1Adapter : public GuidanceInterface
{
public:
    L1Adapter();

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

    // L1控制器特有方法
    void navigateWaypoints(const matrix::Vector2f &waypoint_A, 
                          const matrix::Vector2f &waypoint_B,
                          const matrix::Vector2f &current_position,
                          const matrix::Vector2f &ground_speed);
    
    void navigateLoiter(const matrix::Vector2f &center,
                       const matrix::Vector2f &current_position,
                       float radius,
                       int8_t loiter_direction,
                       const matrix::Vector2f &ground_speed);
    
    void navigateHeading(float navigation_heading, 
                       float current_heading,
                       const matrix::Vector2f &ground_speed);
    
    void navigateLevelFlight(float current_heading);

    // L1参数设置
    void setL1Period(float period);
    void setL1Damping(float damping);
    void setL1RollLimit(float roll_lim_rad);
    void setRollSlewRate(float roll_slew_rate);
    void setDt(float dt);

    // 状态查询
    float getNavBearing() const;
    float getLateralAccelerationDemand() const;
    float getBearingError() const;
    float getTargetBearing() const;
    float getRollSetpoint() const;
    float getCrosstrackError() const;
    bool reachedLoiterTarget() const;
    bool circleMode() const;

private:
    ECL_L1_Pos_Controller _l1_controller;
    
    // 状态变量
    matrix::Vector2f _current_waypoint_A;
    matrix::Vector2f _current_waypoint_B;
    bool _waypoint_mode_active{false};
    bool _loiter_mode_active{false};
    float _loiter_radius{50.0f};
    int8_t _loiter_direction{1};
    
    // 辅助函数
    matrix::Vector2f calculatePathTangent(const matrix::Vector2f &waypoint_A,
                                        const matrix::Vector2f &waypoint_B);
    matrix::Vector2f calculateClosestPointOnPath(const matrix::Vector2f &current_pos,
                                               const matrix::Vector2f &waypoint_A,
                                               const matrix::Vector2f &waypoint_B);
    float calculatePathCurvature(const matrix::Vector2f &waypoint_A,
                                 const matrix::Vector2f &waypoint_B);
};
