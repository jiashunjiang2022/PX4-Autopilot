#pragma once

#include "GuidanceInterface.hpp"
#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <stdint.h>

/**
 * Pure Pursuit制导算法适配器
 * 
 * Pure Pursuit是一种经典的路径跟踪算法，通过追踪路径上前方固定距离的点来实现导航。
 * 
 * 核心思想：
 * 1. 在路径上找到距离飞机前方lookahead_distance的目标点
 * 2. 计算从飞机到目标点的方位角
 * 3. 使用几何关系计算所需的转弯曲率
 * 
 * 优点：简单、直观、计算量小
 * 适用场景：低速、大转弯半径、简单路径跟踪
 */
class PurePursuitAdapter : public GuidanceInterface
{
public:
    PurePursuitAdapter();

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
    float getCourseSetpoint() const override { return _current_course_setpoint; }
    float getLateralAccelerationSetpoint() const override { return _current_lateral_acceleration; }
    float getBearingFeasibility() const override { return 1.0f; } // Pure Pursuit不计算可行性
    float getBearingFeasibilityOnTrack() const override { return 1.0f; }
    float getSignedTrackError() const override { return _current_track_error; }
    float getTrackErrorBound() const override { return _lookahead_distance; }
    float getAdaptedPeriod() const override { return _lookahead_distance / 15.0f; } // 估算周期
    float switchDistance(float wp_radius) const override;

    // Pure Pursuit特有参数设置
    void setLookaheadGain(float gain) { _lookahead_gain = math::max(gain, 0.5f); }
    void setMinLookahead(float min_dist) { _min_lookahead = math::max(min_dist, 5.0f); }
    void setMaxLookahead(float max_dist) { _max_lookahead = math::max(max_dist, 20.0f); }

private:
    // Pure Pursuit参数
    float _lookahead_gain{2.0f};      // 前视距离增益（相对于地速）
    float _min_lookahead{10.0f};      // 最小前视距离 [m]
    float _max_lookahead{50.0f};      // 最大前视距离 [m]
    float _lookahead_distance{20.0f}; // 当前前视距离 [m]

    // 状态变量
    float _current_course_setpoint{0.0f};
    float _current_lateral_acceleration{0.0f};
    float _current_track_error{0.0f};

    // 辅助函数
    float normalizeAngle(float angle);
};

