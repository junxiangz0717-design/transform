#include "serial_data.h"
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/rclcpp.hpp> 
#include "log.h"

class SendProcess : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    SendData serial_send_data_;

public:
    SendProcess() : Node("send_process")
    {
        this->cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_virtual", 1, 
            std::bind(&SendProcess::CmdVelCallback, this, std::placeholders::_1));

        // 线速度
        this->serial_send_data_.vx = 0.0f;
        this->serial_send_data_.vy = 0.0f;
        this->serial_send_data_.wz = 0.0f;
    }

    ~SendProcess() = default;

    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        this->serial_send_data_.vx = -msg->linear.x; 
        this->serial_send_data_.vy = msg->linear.y; 
        this->serial_send_data_.wz = msg->angular.z; 
        RCLCPP_INFO(this->get_logger(), "发送串口速度: vx=%.2f, vy=%.2f, wz=%.2f", 
                    this->serial_send_data_.vx, 
                    this->serial_send_data_.vy, 
                    this->serial_send_data_.wz);
        serialdata.zh_write(this->serial_send_data_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SendProcess>());
    rclcpp::shutdown();
    return 0;
}
