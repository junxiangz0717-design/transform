/**
 * @file send_process.cpp
 * @author 张俊翔(junxiangz0717@gmail.com)
 * @brief  哨兵接收进程，从串口读取数据并发布为 ROS 2 话题
 * @date 2026-02-25
 * @copyright Copyright SCUT RobotLab(c) 2026
 */

#include <string>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include "msg_process/msg/receive_data.hpp"
#include "serial_data.h"

using namespace std;

class ReceiveProcessNode : public rclcpp::Node
{
public:
    ReceiveProcessNode() : Node("receive_process")
    {
        // 1. 初始化发布者
        serial_receive_data_pub_ = this->create_publisher<msg_process::msg::ReceiveData>("/serial_receive_data", 10);
        serial_error_data_pub_ = this->create_publisher<msg_process::msg::ReceiveData>("/serial_error_data", 10);
        can_exec_pub_ = this->create_publisher<std_msgs::msg::Bool>("/can_exec", 1);

        // 2. 初始化空数据模板 (用于校验)
        empty_data_.radar_data.resize(12, 0.0f);

        // 3. 创建定时器，以 80Hz 频率读取串口
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(12), // 约 83Hz
            std::bind(&ReceiveProcessNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Receive Process Node initialized at 80Hz.");
    }

private:
    void timer_callback()
    {
        // 读取串口原始数据
        ReceiveData serial_data = serialdata.zh_read();

        // 构造 ROS 2 消息
        auto temp_data = msg_process::msg::ReceiveData();
        temp_data.radar_data.resize(12);

        // 数据赋值
        temp_data.goalx = serial_data.goalx;
        temp_data.goaly = serial_data.goaly;
        temp_data.time = serial_data.time;
        temp_data.hp_sentry = serial_data.hp_sentry;
        temp_data.hp_outpost = serial_data.hp_outpost;
        temp_data.color = serial_data.color;

        // 坐标与位置信息
        temp_data.our_hero_x = serial_data.our_hero_x;
        temp_data.our_hero_y = serial_data.our_hero_y;
        temp_data.our_foot_3_x = serial_data.our_foot_3_x;
        temp_data.our_foot_3_y = serial_data.our_foot_3_y;
        temp_data.our_foot_4_x = serial_data.our_foot_4_x;
        temp_data.our_foot_4_y = serial_data.our_foot_4_y;

        // 血量信息
        temp_data.our_hero_hp = serial_data.our_hero_hp;
        temp_data.our_foot_3_hp = serial_data.our_foot_3_hp;
        temp_data.our_foot_4_hp = serial_data.our_foot_4_hp;
        temp_data.enemy_hero_hp = serial_data.enemy_hero_hp;
        temp_data.enemy_foot_3_hp = serial_data.enemy_foot_3_hp;
        temp_data.enemy_foot_4_hp = serial_data.enemy_foot_4_hp;
        temp_data.enemy_sentry_hp = serial_data.enemy_sentry_hp;
        temp_data.enemy_engineer_hp = serial_data.enemy_engineer_hp;
        temp_data.hp_base = serial_data.hp_base;
        temp_data.hp_enemy_base = serial_data.hp_enemy_base;
        temp_data.hp_enemy_outpost = serial_data.hp_enemy_outpost;

        // 状态位与增益
        temp_data.add_area_state = serial_data.add_area_state;
        temp_data.base_defence = serial_data.base_defence;
        temp_data.add_hp_buff = serial_data.add_hp_buff;
        temp_data.defence_buff = serial_data.defence_buff;
        temp_data.rfid = serial_data.RFID;

        // 复杂逻辑：RFID 判断
        temp_data.is_in_add_area = ((serial_data.RFID >> 19) & 1) | ((serial_data.RFID >> 20) & 1);
        temp_data.is_in_fort = (serial_data.RFID >> 24) & 1;

        temp_data.chassis_power = serial_data.chassis_power;
        temp_data.remain_energy = serial_data.remain_energy;
        temp_data.style = serial_data.style;
        temp_data.game_period = serial_data.game_period;

        // 特殊逻辑：比赛结束杀掉录制节点
        if (temp_data.game_period == 5)
        {
            string cmd = "ps -ef | grep record | grep -v grep | awk '{print $2}' | xargs kill -9";
            if (system(cmd.c_str()) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute kill command");
            }
        }

        temp_data.heat_1 = serial_data.heat_1;
        temp_data.is_no_ammo = serial_data.is_no_ammo;
        temp_data.restart_mapping = serial_data.restart_mapping;
        temp_data.rune = (serial_data.event >> 3) & 1;
        temp_data.delta_yaw = serial_data.delta_yaw;
        temp_data.vx = serial_data.vx;
        temp_data.vy = serial_data.vy;

        // 决策位解析
        temp_data.success_home_buy_ammo = serial_data.sentry_info & 0b11111111111;
        temp_data.success_remote_buy_ammo_times = (serial_data.sentry_info >> 11) & 0b1111;
        temp_data.success_remote_buy_hp_times = (serial_data.sentry_info >> 15) & 0b1111;

        // 雷达数据拷贝
        // 假设 msg_process/msg/ReceiveData 中 radar_data 是 float[] 数组
        std::copy(serial_data.radar_data, serial_data.radar_data + 12, temp_data.radar_data.begin());

        // 电控信息
        temp_data.x_data = serial_data.x_data;
        temp_data.y_data = serial_data.y_data;
        temp_data.rotation_state = serial_data.rotationState;
        temp_data.temp_x = serial_data.temp_x;
        temp_data.temp_y = serial_data.temp_y;
        temp_data.temp_theta = serial_data.temp_theta;
        temp_data.speed_x = serial_data.speed_X;
        temp_data.speed_y = serial_data.speed_Y;
        temp_data.speed_z = serial_data.speed_Z;
        temp_data.wheel_state1 = serial_data.wheel_state1;
        temp_data.wheel_state2 = serial_data.wheel_state2;
        temp_data.wheel_state3 = serial_data.wheel_state3;
        temp_data.wheel_state4 = serial_data.wheel_state4;


        /** 数据合法性校验 **/
        bool is_empty = true;
        for (int i = 0; i < 12; ++i)
            if (temp_data.radar_data[i] != 0)
                is_empty = false;

        if (is_empty && temp_data.hp_sentry == 0)
        {
            serial_error_data_pub_->publish(temp_data);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received empty data packet!");
            return;
        }

        // 范围校验逻辑 (保持原样)
        if (temp_data.goalx < 0.0f || temp_data.goalx > 28.0f || temp_data.hp_sentry > 1000)
        {
            serial_error_data_pub_->publish(temp_data);
            RCLCPP_ERROR(this->get_logger(), "Data Range Validation Failed!");
        }
        else
        {
            serial_receive_data_pub_->publish(temp_data);
            RCLCPP_DEBUG(this->get_logger(), "Sentry HP: %d", temp_data.hp_sentry);
        }
    }

    // 成员变量
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<msg_process::msg::ReceiveData>::SharedPtr serial_receive_data_pub_;
    rclcpp::Publisher<msg_process::msg::ReceiveData>::SharedPtr serial_error_data_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can_exec_pub_;
    msg_process::msg::ReceiveData empty_data_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReceiveProcessNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
