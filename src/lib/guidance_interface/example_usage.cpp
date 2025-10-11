/**
 * @file example_usage.cpp
 * @brief 制导接口使用示例
 * 
 * 这个文件展示了如何使用参数驱动的制导控制器选择
 */

#include "GuidanceInterface.hpp"
#include "PIDAdapter.hpp"
#include "NPFGAdapter.hpp"
#include "L1Adapter.hpp"

/**
 * @brief 制导控制器使用示例
 */
class GuidanceControllerExample
{
public:
    GuidanceControllerExample() = default;
    ~GuidanceControllerExample() = default;

    /**
     * @brief 根据参数初始化制导控制器
     * @param guidance_mode 制导模式 (0=L1, 1=PID, 2=NPFG)
     */
    void initializeGuidance(int guidance_mode)
    {
        // 创建所有制导适配器
        _pid_adapter = new PIDAdapter();
        _npfg_adapter = new NPFGAdapter();
        _l1_adapter = new L1Adapter();
        
        // 根据参数选择制导接口
        switch (guidance_mode) {
            case 0: // L1制导
                _guidance_interface = _l1_adapter;
                printf("使用L1制导控制器\n");
                break;
                
            case 1: // PID制导
                _guidance_interface = _pid_adapter;
                // 设置PID参数
                _pid_adapter->setCoursePIDParams(2.0f, 0.1f, 0.5f);
                _pid_adapter->setHeadingPIDParams(1.5f, 0.05f, 0.3f);
                printf("使用PID制导控制器\n");
                break;
                
            case 2: // NPFG制导
                _guidance_interface = _npfg_adapter;
                printf("使用NPFG制导控制器\n");
                break;
                
            default:
                // 默认使用L1制导
                _guidance_interface = _l1_adapter;
                printf("未知的制导模式 %d，使用L1制导\n", guidance_mode);
                break;
        }
    }

    /**
     * @brief 执行制导计算
     * @param current_pos 当前位置
     * @param ground_vel 地速
     * @param wind_vel 风速
     * @param path_tangent 路径切线
     * @param closest_point 最近点
     * @param path_curvature 路径曲率
     * @return 制导输出
     */
    GuidanceOutput executeGuidance(const matrix::Vector2f &current_pos,
                                   const matrix::Vector2f &ground_vel,
                                   const matrix::Vector2f &wind_vel,
                                   const matrix::Vector2f &path_tangent,
                                   const matrix::Vector2f &closest_point,
                                   float path_curvature)
    {
        if (_guidance_interface == nullptr) {
            printf("错误：制导接口未初始化\n");
            return GuidanceOutput{};
        }

        // 使用统一的制导接口
        return _guidance_interface->guideToPath(current_pos, ground_vel, wind_vel,
                                               path_tangent, closest_point, path_curvature);
    }

    /**
     * @brief 获取制导状态信息
     */
    void printGuidanceStatus()
    {
        if (_guidance_interface == nullptr) {
            printf("制导接口未初始化\n");
            return;
        }

        printf("制导状态信息：\n");
        printf("  航向设定点: %.2f 度\n", math::degrees(_guidance_interface->getCourseSetpoint()));
        printf("  横向加速度: %.2f m/s²\n", _guidance_interface->getLateralAccelerationSetpoint());
        printf("  航向可行性: %.2f\n", _guidance_interface->getBearingFeasibility());
        printf("  航向误差: %.2f 度\n", math::degrees(_guidance_interface->getSignedTrackError()));
    }

    /**
     * @brief 清理资源
     */
    void cleanup()
    {
        if (_pid_adapter) {
            delete _pid_adapter;
            _pid_adapter = nullptr;
        }
        
        if (_npfg_adapter) {
            delete _npfg_adapter;
            _npfg_adapter = nullptr;
        }
        
        if (_l1_adapter) {
            delete _l1_adapter;
            _l1_adapter = nullptr;
        }
        
        _guidance_interface = nullptr;
    }

private:
    GuidanceInterface* _guidance_interface{nullptr};
    PIDAdapter* _pid_adapter{nullptr};
    NPFGAdapter* _npfg_adapter{nullptr};
    L1Adapter* _l1_adapter{nullptr};
};

/**
 * @brief 使用示例
 */
void example_usage()
{
    GuidanceControllerExample controller;
    
    // 示例1：使用L1制导
    printf("=== 示例1：L1制导 ===\n");
    controller.initializeGuidance(0); // L1制导
    
    // 模拟输入数据
    matrix::Vector2f current_pos(100.0f, 200.0f);
    matrix::Vector2f ground_vel(20.0f, 0.0f);
    matrix::Vector2f wind_vel(5.0f, 2.0f);
    matrix::Vector2f path_tangent(1.0f, 0.0f);
    matrix::Vector2f closest_point(100.0f, 200.0f);
    float path_curvature = 0.0f;
    
    // 执行制导计算
    GuidanceOutput output = controller.executeGuidance(current_pos, ground_vel, wind_vel,
                                                      path_tangent, closest_point, path_curvature);
    
    printf("L1制导输出：\n");
    printf("  航向设定点: %.2f 度\n", math::degrees(output.course_setpoint));
    printf("  横向加速度: %.2f m/s²\n", output.lateral_acceleration_feedforward);
    
    controller.printGuidanceStatus();
    
    // 示例2：使用PID制导
    printf("\n=== 示例2：PID制导 ===\n");
    controller.initializeGuidance(1); // PID制导
    
    output = controller.executeGuidance(current_pos, ground_vel, wind_vel,
                                       path_tangent, closest_point, path_curvature);
    
    printf("PID制导输出：\n");
    printf("  航向设定点: %.2f 度\n", math::degrees(output.course_setpoint));
    printf("  横向加速度: %.2f m/s²\n", output.lateral_acceleration_feedforward);
    
    controller.printGuidanceStatus();
    
    // 示例3：使用NPFG制导
    printf("\n=== 示例3：NPFG制导 ===\n");
    controller.initializeGuidance(2); // NPFG制导
    
    output = controller.executeGuidance(current_pos, ground_vel, wind_vel,
                                       path_tangent, closest_point, path_curvature);
    
    printf("NPFG制导输出：\n");
    printf("  航向设定点: %.2f 度\n", math::degrees(output.course_setpoint));
    printf("  横向加速度: %.2f m/s²\n", output.lateral_acceleration_feedforward);
    
    controller.printGuidanceStatus();
    
    // 清理资源
    controller.cleanup();
}
