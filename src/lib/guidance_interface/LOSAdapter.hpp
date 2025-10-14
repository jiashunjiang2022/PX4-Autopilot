#pragma once

#include "GuidanceInterface.hpp"
#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <stdint.h>

/**
 * LOS (Line-of-Sight) 制导算法适配器
 * 
 * LOS是一种基于视线角的路径跟踪算法，广泛应用于海洋船舶和无人机导航。
 * 
 * 核心思想：
 * 1. 计算从飞机到路径的横向误差（cross-track error）
 * 2. 根据横向误差和前视距离计算修正角
 * 3. 期望航向 = 路径方向 + 修正角
 * 4. 考虑侧风补偿
 * 
 * 优点：简单、鲁棒、有理论基础、适合直线段
 * 适用场景：直线路径跟踪、需要侧风补偿的场景
 */
class LOSAdapter : public GuidanceInterface
{
public:
    LOSAdapter();

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
    float getBearingFeasibility() const override { return 1.0f; } // LOS不计算可行性
    float getBearingFeasibilityOnTrack() const override { return 1.0f; }
    float getSignedTrackError() const override { return _current_track_error; }
    float getTrackErrorBound() const override { return _lookahead_distance; }
    float getAdaptedPeriod() const override { return _lookahead_distance / 15.0f; }
    float switchDistance(float wp_radius) const override;

    // LOS特有参数设置
    void setLookaheadDistance(float distance) { _lookahead_distance = math::max(distance, 10.0f); }
    void setMaxCourseError(float max_error_rad) { _max_course_error = math::constrain(max_error_rad, 0.1f, M_PI_F / 2.0f); }
    void setEnableWindCompensation(bool enable) { _enable_wind_compensation = enable; }

private:
    // LOS参数
    float _lookahead_distance{20.0f};        // 前视距离 [m]
    float _max_course_error{M_PI_F / 3.0f};  // 最大航向修正角 [rad] (60度)
    bool _enable_wind_compensation{true};     // 是否启用侧风补偿

    // 状态变量
    float _current_course_setpoint{0.0f};
    float _current_lateral_acceleration{0.0f};
    float _current_track_error{0.0f};

    // 辅助函数
    float normalizeAngle(float angle);
    float calculateWindCorrectionAngle(const matrix::Vector2f &wind_vel,
                                      const matrix::Vector2f &unit_path_tangent,
                                      float airspeed);
};

