#pragma once

#include "GuidanceInterface.hpp"
#include <lib/npfg/DirectionalGuidance.hpp>
#include <lib/npfg/CourseToAirspeedRefMapper.hpp>
#include <lib/npfg/AirspeedDirectionController.hpp>
#include <matrix/math.hpp>

class NPFGAdapter : public GuidanceInterface
{
public:
    NPFGAdapter();

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

    // 直接访问原有NPFG对象（用于参数设置等）
    DirectionalGuidance& getDirectionalGuidance() { return _directional_guidance; }
    CourseToAirspeedRefMapper& getCourseToAirspeed() { return _course_to_airspeed; }
    AirspeedDirectionController& getAirspeedDirectionControl() { return _airspeed_direction_control; }

private:
    // 保持原有的NPFG对象不变
    DirectionalGuidance _directional_guidance;
    CourseToAirspeedRefMapper _course_to_airspeed;
    AirspeedDirectionController _airspeed_direction_control;
};
