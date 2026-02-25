#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class VirtualHeadNode : public rclcpp::Node {
public:
    VirtualHeadNode() : Node("virtual_head_node") {
        // 初始化 TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 1. 订阅高频里程计 (替代原来的定时器查询 TF)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odin1/odometry_highfreq", 10, 
            std::bind(&VirtualHeadNode::odom_callback, this, std::placeholders::_1));

        // 2. 订阅虚拟系指令速度 (保留原功能)
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VirtualHeadNode::vel_callback, this, std::placeholders::_1));

        // 3. 发布实际到底盘的速度 (保留原功能)
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_virtual", 10);

        yaw_real_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "虚拟头节点已启动：订阅高频里程计并发布动态 TF...");
    }

private:
    // 处理高频里程计并发布 TF
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // --- 保留并适配你原有的 yaw_real_ 计算逻辑 ---
        // 原逻辑是从 odom -> virtual_base_link 的 TF 中提取 yaw 并加偏移
        // 这里我们先获取 odin1_base_link 的 yaw
        double current_odin_yaw = tf2::getYaw(msg->pose.pose.orientation);
        
        // 按照你代码中的逻辑：yaw_real_ = t.yaw + M_PI + 0.60
        // 注意：如果你之前的 virtual_base_link 相对于 odin1_base_link 有静态旋转，需在此体现
        // 假设静态变换中 virtual_base_link 相对 odin1_base_link 的 yaw 偏移是 M_PI
        yaw_real_ = current_odin_yaw + 0.06 ; //
        //  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        // "速度转换 [Yaw: %.2f rad]",yaw_real_ );

        // --- 发布 base_link 的动态 TF ---
        // 逻辑：结合里程计位姿与静态变换参数
        tf2::Transform tf_odom_to_odin;
        tf2::fromMsg(msg->pose.pose, tf_odom_to_odin);

        // 静态变换参数 (来自你的 launch 配置)
        double offset_x = -0.12;
        double offset_y = -0.175;
        double offset_z = -0.2;
        tf2::Quaternion q_static;
        q_static.setRPY(0, 0, M_PI); // 对应 launch 中的 3.1416

        tf2::Transform tf_odin_to_base(q_static, tf2::Vector3(offset_x, offset_y, offset_z));

        // 计算最终 base_link 在 odom 下的位姿
        tf2::Transform tf_odom_to_base = tf_odom_to_odin * tf_odin_to_base;

        geometry_msgs::msg::TransformStamped v_t;
        v_t.header.stamp = msg->header.stamp;
        v_t.header.frame_id = "odom";
        v_t.child_frame_id = "base_link";
        
        // --- 保留原代码中“不随车旋转”或特定姿态的逻辑 ---
        // 你原代码里将 v_t 的姿态硬设置为 RPY(0,0,M_PI)，这里予以保留：
        v_t.transform.translation.x = tf_odom_to_base.getOrigin().x();
        v_t.transform.translation.y = tf_odom_to_base.getOrigin().y();
        v_t.transform.translation.z = tf_odom_to_base.getOrigin().z();

        tf2::Quaternion q_fixed;
        q_fixed.setRPY(0, 0, M_PI); // 原代码：q.setRPY(0, 0, M_PI);
        v_t.transform.rotation = tf2::toMsg(q_fixed);

        tf_broadcaster_->sendTransform(v_t);
    }

    // 速度转换逻辑 (完全保留原功能)
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto real_vel = geometry_msgs::msg::Twist();
        double angle = yaw_real_; 
        
        real_vel.linear.x = msg->linear.x * std::cos(angle) + msg->linear.y * std::sin(angle);
        real_vel.linear.y = -msg->linear.x * std::sin(angle) + msg->linear.y * std::cos(angle);
        real_vel.angular.z = msg->angular.z;
        RCLCPP_INFO(this->get_logger(),
        "速度转换 [Yaw: %.2f rad]: 输入(%.2f, %.2f) -> 输出(%.2f, %.2f)",
        angle, msg->linear.x, msg->linear.y, real_vel.linear.x, real_vel.linear.y);
    
        vel_pub_->publish(real_vel);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double yaw_real_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualHeadNode>());
    rclcpp::shutdown();
    return 0;
}