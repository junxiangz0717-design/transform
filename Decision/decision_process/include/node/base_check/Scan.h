#pragma once

/**
 * @brief 云台扫描控制节点
 * @description 控制云台枪管在感知相机扫不到的区域（前方）往复扫动
 *              感知相机在背后，覆盖后方 86°（H: 86°, V: 70°）
 *              云台扫描前方区域，使用自瞄相机视场角(H: 69.2°, V: 54.3°)计算边界
 */

#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <algorithm>
#include <cmath>

using namespace std;
using namespace BT;

class Scan : public ConditionNode 
{
private:
    // 扫描状态
    bool lock = false;
    double angle = 0;
    double max_angle = 0;
    double min_angle = 0;
    Target target = Target::NONE;
    
    // 感知相机视场角参数（弧度）- 装在背后，朝向后方
    static constexpr double DETECTOR_FOV_H = 86.0 * M_PI / 180.0;   // 感知水平视场角 86°
    static constexpr double HALF_DETECTOR_FOV_H = DETECTOR_FOV_H / 2.0;  // 半视场角 43°
    
    // 自瞄相机视场角参数（弧度）- 装在云台上，朝向前方
    static constexpr double AUTOAIM_FOV_H = 69.2 * M_PI / 180.0;    // 自瞄水平视场角 69.2°
    static constexpr double HALF_AUTOAIM_FOV_H = AUTOAIM_FOV_H / 2.0;  // 半视场角 34.6°
    
    // 云台扫描边界（感知相机看不到的区域 = 前方）
    // 感知相机朝后(180°)，覆盖范围: 180° ± 43° = 137° ~ 223°（或等价 -137° ~ -223°）
    // 云台需要扫描的前方区域: -137° ~ +137°
    // 考虑自瞄视场角后的有效扫描边界:
    static constexpr double SCAN_BOUNDARY = M_PI - HALF_DETECTOR_FOV_H;  // 137° ≈ 2.39 rad
    
public:
    Scan(const string &name, const NodeConfig &config) : ConditionNode(name, config) {}
    
    ~Scan() override = default;
    
    static PortsList providedPorts()
    {
        return {
            OutputPort<int>("tripod_spin", "云台自旋模式"),
            InputPort<Target>("have_target", "检测到的目标")
        };
    }   
    
    NodeStatus tick() override
    {
        getInput("have_target", target);
        
        // 有目标时停止扫描，让自瞄接管
        if (target != Target::NONE) 
        { 
            lock = false;
            return NodeStatus::SUCCESS;
        }
        
        // 检查是否开启扫描
        if (!DD.config_is_scan)
        {
            if (DD.is_in_block) 
            { 
                DD.decision_yaw_az = -0.01;
                return NodeStatus::SUCCESS;
            }
            DD.decision_yaw_az = DD.config_az;
            return NodeStatus::SUCCESS;
        }
        
        // 获取当前云台角度（相对于车身前方，归一化到 -π ~ π）
        angle = DD.normalize_angle(DD.big_yaw);
        
        // 执行前方区域扫描（感知相机看不到的区域）
        return scanFrontArea();
    }

private:
    /**
     * @brief 扫描前方区域（感知相机覆盖不到的区域）
     * 
     * 感知相机在背后，覆盖后方 ±43°（相对于180°）
     * 云台扫描前方 ±137°，但考虑自瞄视场角后：
     *   - 有效扫描边界 = ±(137° - 34.6°) = ±102.4°
     *   - 这样自瞄相机在边界时刚好能看到感知相机看不到的边缘
     */
    NodeStatus scanFrontArea()
    {
        setOutput("tripod_spin", 0);
        
        // 计算有效扫描范围（基于视场角，不含补偿）
        // 扫描到边界时，自瞄相机的视野边缘刚好接上感知相机的视野边缘
        max_angle = SCAN_BOUNDARY - HALF_AUTOAIM_FOV_H;  // 约 102.4°
        min_angle = -SCAN_BOUNDARY + HALF_AUTOAIM_FOV_H; // 约 -102.4°
        
        executeScan();
        
        return NodeStatus::SUCCESS;
    }
    
    /**
     * @brief 执行扫描控制（往复扫动）
     */
    void executeScan()
    {
        if (angle > min_angle && angle < max_angle && !lock)
        { 
            // 在范围内，开始扫描
            lock = true; 
            DD.decision_yaw_az = DD.config_az;
        }
        // 判断条件加入偏移补偿，用于补偿云台惯性延迟
        // 云台转过边界后再多转 offset 才反向，避免来回抖动
        else if (angle > max_angle + DD.config_max_scan_offset_yaw)
        {
            // 超过最大角度+补偿，反向扫描
            DD.decision_yaw_az = -DD.config_az;
        }
        else if (angle < min_angle + DD.config_min_scan_offset_yaw)
        {
            // 低于最小角度+补偿，正向扫描
            DD.decision_yaw_az = DD.config_az;
        }
    }
    
    /**
     * @brief 弧度转角度（调试用）
     */
    static double rad2deg(double rad)
    {
        return rad * 180.0 / M_PI;
    }
};