/*
    created on 2025-11-16
*/

#ifndef DUMMY_AUTOAIM_NODE_HPP
#define DUMMY_AUTOAIM_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <msg_process/msg/autoaim_to_decision.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <memory>

// 兵种
enum class RobotType
{
    HERO = 0,
    ENGINEER = 1,
    FOOT_3 = 2,
    FOOT_4 = 3,
    OUTPOST = 4,
    BASE = 5,
    SENTRY = 6
};

class DummyAutoaim : public rclcpp::Node
{
public:
    DummyAutoaim();

private:
    // 发布器
    rclcpp::Publisher<msg_process::msg::AutoaimToDecision>::SharedPtr autoaim_to_decision_pub_;

    // 参数回调
    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 参数描述创建方法
    rcl_interfaces::msg::ParameterDescriptor create_enum_descriptor(
        const std::string &description,
        const std::vector<std::pair<int, std::string>> &enum_values);

    rcl_interfaces::msg::ParameterDescriptor create_bool_descriptor(const std::string &description);
    rcl_interfaces::msg::ParameterDescriptor create_int_descriptor(const std::string &description, int min, int max);
    rcl_interfaces::msg::ParameterDescriptor create_double_descriptor(const std::string &description, double min, double max);

    // 参数变量
    struct Params
    {
        bool stop = false;        // 停止参数发布
        bool target_mode = false; // 目标状态
        // 默认一开始是英雄
        RobotType target = RobotType::HERO;          // 自瞄目标
        RobotType detector_target = RobotType::HERO; // 感知目标
        int shootable_num = 0;                       // 打击范围内敌人数量

        // 自瞄开关
        int auto_0 = 0; // 自瞄英雄
        int auto_1 = 0; // 自瞄工程
        int auto_2 = 0; // 自瞄3号步兵
        int auto_3 = 0; // 自瞄4号步兵
        int auto_4 = 0; // 自瞄前哨
        int auto_5 = 0; // 自瞄基地
        int auto_6 = 0; // 自瞄哨兵

        // 自瞄距离
        double auto_distance_0 = 10.0;
        double auto_distance_1 = 10.0;
        double auto_distance_2 = 10.0;
        double auto_distance_3 = 10.0;
        double auto_distance_4 = 10.0;
        double auto_distance_5 = 10.0;
        double auto_distance_6 = 10.0;

        // 感知开关
        int detector_0 = 0; // 感知英雄
        int detector_1 = 0; // 感知工程
        int detector_2 = 0; // 感知3号步兵
        int detector_3 = 0; // 感知4号步兵
        int detector_4 = 0; // 感知前哨
        int detector_5 = 0; // 感知基地
        int detector_6 = 0; // 感知哨兵

        // 感知距离
        double detector_distance_0 = 10.0;
        double detector_distance_1 = 10.0;
        double detector_distance_2 = 10.0;
        double detector_distance_3 = 10.0;
        double detector_distance_4 = 10.0;
        double detector_distance_5 = 10.0;
        double detector_distance_6 = 10.0;
    } params_;

    void publish_data();       // 发布数据
    void log_current_params(); // 日志记录当前参数
};

#endif