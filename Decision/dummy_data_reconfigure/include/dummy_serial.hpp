/*
    created on 2025-11-16
*/ 

#ifndef DUMMY_SERIAL_NODE_HPP
#define DUMMY_SERIAL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <msg_process/msg/receive_data.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <memory>

// 己方颜色
enum class Color {
    RED = 0,                // 红色
    BLUE = 1                // 蓝色
};

// 比赛阶段
enum class GamePeriod {
    NO_START = 0,           // 未开始
    PREPARE = 1,            // 准备中
    CHECKING_15s = 2,       // 15秒裁判系统自检
    PRE_START_5s = 3,       // 5秒倒计时
    GAMING = 4,             // 比赛中
    END = 5                 // 比赛结算中
};

// 战斗风格
enum class Style {
    DEFENCE = 0,            // 保守
    CHARGE = 1,             // 进击
    PATROL = 2,             // 巡逻
    NORMAL = 3,             // 普通
    TRAINING_4 = 4,         // 适应性训练4
    TRAINING_5 = 5          // 适应性训练5
};

// 飞镖目标
enum class DartTarget {
    OUTPOST = 0,
    CONST_BASE = 1,
    RANDOM_BASE = 2
};

class DummySerial : public rclcpp::Node
{
public:
    DummySerial();

private:
    // 发布器
    rclcpp::Publisher<msg_process::msg::ReceiveData>::SharedPtr serial_receive_data_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pos_data_pub_;
    
    // 参数回调
    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & parameters);
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 倒计时定时器    26新加
    rclcpp::TimerBase::SharedPtr countdown_timer_;

    // 参数描述创建方法
    rcl_interfaces::msg::ParameterDescriptor create_enum_descriptor(
        const std::string& description, 
        const std::vector<std::pair<int, std::string>>& enum_values);
    
    // 参数变量
    struct Params {
        bool stop = false;                     // 停止参数发布
        double goal_x = 0.0;                   // 云台手给点x坐标
        double goal_y = 0.0;                   // 云台手给点y坐标
        int time = 300;                        // 比赛剩余时间
        int hp_sentry = 400;                   // 哨兵血量
        int defence_buff = 60;                 // 哨兵防御增益
        bool RFID_PATROL = true;               // 哨兵巡逻区RFID检测状态
        bool RFID_TREATMENT = false;           // 补给区RFID检测状态
        bool is_no_ammo = false;               // 子弹耗尽
        int ammo = 300;                        // 允许发弹量
        int restart_decision_game = 0;         // 重开决策比赛标志位
        int defence_base = 0;                  // 基地虚拟护盾
        int hp_base = 5000;                    // 基地血量
        int hp_outpost = 1500;                 // 前哨站血量
        int our_hero_hp = 250;                 // 我方英雄血量
        int our_engineer_hp = 250;             // 我方工程血量
        int our_foot_3_hp = 200;               // 我方3号步兵血量
        int our_foot_4_hp = 200;               // 我方4号步兵血量
        double our_hero_x = 5.0;               // 我方英雄x位置
        double our_hero_y = 5.0;               // 我方英雄y位置
        double our_foot_3_x = 2.0;             // 我方3号步兵x位置
        double our_foot_3_y = 2.0;             // 我方3号步兵y位置
        double our_foot_4_x = 2.0;             // 我方4号步兵x位置
        double our_foot_4_y = 2.0;             // 我方4号步兵y位置
        double cur_x = 0.0;                    // 自身x位置
        double cur_y = 0.0;                    // 自身y位置
        double cur_yaw = 0.0;                  // 当前大yaw
        int enemy_hero_hp = 250;               // 敌方英雄血量
        int enemy_engineer_hp = 200;           // 敌方工程血量
        int enemy_foot_3_hp = 200;             // 敌方3号步兵血量
        int enemy_foot_4_hp = 200;             // 敌方4号步兵血量
        int enemy_sentry_hp = 400;             // 敌方哨兵血量
        int enemy_outpost_hp = 1500;           // 敌方前哨站血量
        int enemy_base_hp = 5000;              // 敌方基地血量
        int success_home_buy_ammo = 0;         // 补给区成功兑换弹量
        int success_remote_buy_ammo_times = 0; // 远程成功买弹次数
        int success_remote_buy_hp_times = 0;   // 远程成功买血次数
        bool can_free_respawn = false;         // 是否可确认免费复活
        bool can_buy_respawn = false;          // 是否可买活
        int buy_respawn_money = 0;             // 当前买活所需金币
        bool is_disengaged = true;             // 脱战标志位
        int team_residual_ammo = 0;            // 队伍剩余可兑换弹量
        int is_center_area = 0;                // 中心区占领状态(0无，1我方，2敌方，3双方)
        Color color = Color::RED;
        GamePeriod game_period = GamePeriod::NO_START;
        Style style = Style::DEFENCE;
        DartTarget dart_target = DartTarget::OUTPOST;                  
    } params_;
    
    void publish_data();
};

#endif