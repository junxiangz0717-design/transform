/**
 * @file send_process.cpp
 * @author 张俊翔(junxiangz0717@gmail.com)
 * @brief  哨兵发送进程，将 ROS 2 话题数据打包并通过串口发送至底层
 * @date 2026-02-25
 * @copyright Copyright SCUT RobotLab(c) 2026
 */

#include "msg_process/msg/send_data.hpp"
#include "serial_data.h"
#include <rclcpp/rclcpp.hpp>

class SendProcessNode : public rclcpp::Node
{

private:
    rclcpp::Subscription<msg_process::msg::SendData>::SharedPtr path_sub_;
    SendData serial_send_data_;

public:
    SendProcessNode() : Node("send_process")
    {
        path_sub_ = this->create_subscription<msg_process::msg::SendData>(
            "/serial_send_data", 
            10, 
            std::bind(&SendProcessNode::pathDataCallBack, this, std::placeholders::_1)
        );

        serial_send_data_.vx = 0.0f;
        serial_send_data_.vy = 0.0f;
        
        RCLCPP_INFO(this->get_logger(), "Send Process Node has started.");
    }

    ~SendProcessNode() override = default;

private:
    /**
     * @brief 订阅回调函数
     * @param msg 接收到的消息常指针
     */
    void pathDataCallBack(const msg_process::msg::SendData::SharedPtr msg)
    {
        // 基础运动控制
        this->serial_send_data_.vx = msg->vx;
        this->serial_send_data_.vy = msg->vy;

        // 机器人模式控制
        this->serial_send_data_.spin_mode = msg->spin_mode;
        this->serial_send_data_.tripod_spin = msg->tripod_spin; 
        this->serial_send_data_.style = msg->style;
        this->serial_send_data_.pitch_mode = msg->pitch_mode;
        this->serial_send_data_.attack_rune = msg->attack_rune;
        this->serial_send_data_.decision_yaw_az = msg->decision_yaw_az;

        // 哨兵自主决策位处理
        this->serial_send_data_.auto_decision_package = 0; 
        this->serial_send_data_.auto_decision_package |= (msg->confirm_respawn & 0b1);
        this->serial_send_data_.auto_decision_package |= (msg->buy_respawn & 0b1) << 1;
        this->serial_send_data_.auto_decision_package |= (msg->home_buy_ammo & 0b11111111111) << 2;
        this->serial_send_data_.auto_decision_package |= (msg->remote_buy_ammo_times & 0b1111) << 13;
        this->serial_send_data_.auto_decision_package |= (msg->remote_buy_hp_times & 0b1111) << 17;

        RCLCPP_WARN(this->get_logger(), "电机控制速度 -> vx: %.2f, vy: %.2f", 
                    this->serial_send_data_.vx, this->serial_send_data_.vy);
        RCLCPP_WARN(this->get_logger(), "模式 -> 小陀螺: %d, 云台自转: %d, Pitch模式: %d", 
                    this->serial_send_data_.spin_mode, this->serial_send_data_.tripod_spin, this->serial_send_data_.pitch_mode);
        RCLCPP_WARN(this->get_logger(), "决策 -> 风格: %d, 决策角速度: %.2f", 
                    this->serial_send_data_.style, this->serial_send_data_.decision_yaw_az);

        serialdata.zh_write(this->serial_send_data_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendProcessNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

