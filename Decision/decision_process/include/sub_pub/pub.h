// 有内联，不适合放在cpp文件中
// 最底下实例化全局对象

#pragma once

#include "data/decision_data.h"
#include "data/enum.h"
#include "tool/logger/log.h"
#include "tool/tf.h"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"

#include "msg_process/msg/autoaim_to_decision.hpp"
#include "msg_process/msg/decison_send_data.hpp"
#include "msg_process/msg/receive_data.hpp"
#include "msg_process/msg/pursuit_data.hpp"
#include "msg_process/msg/decision_to_autoaim.hpp"
#include "msg_process/msg/decision_msg.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>

class MessagePublisher
{
private:
    rclcpp::Publisher<msg_process::msg::DecisonSendData>::SharedPtr serial_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr way_point_pub;
    rclcpp::Publisher<msg_process::msg::DecisionToAutoaim>::SharedPtr autoaim_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr move_mode_pub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr map_mode_pub;
    rclcpp::Publisher<msg_process::msg::DecisionMsg>::SharedPtr decision_data_pub;

    std::shared_ptr<rclcpp::Node> node_;

    std_msgs::msg::Bool stop_;

    std_msgs::msg::Int8 move_mode_data_;
    std_msgs::msg::Int8 map_mode_data_;

    msg_process::msg::DecisonSendData decision_send_data_;
    msg_process::msg::DecisionToAutoaim decision_send_autoaim_;

    msg_process::msg::DecisionMsg decision_data_;

    geometry_msgs::msg::PoseStamped way_point_;

    zh_timer posture_timer{5000};
    bool is_first_posture_ = true;
    
public:
    ~MessagePublisher() = default;

    msg_process::msg::PursuitData pursuit_data_;
    rclcpp::Publisher<msg_process::msg::PursuitData>::SharedPtr pursuit_data_pub;

    explicit MessagePublisher(std::shared_ptr<rclcpp::Node> node) : node_(node)
    {
        // 给串口
        serial_pub = node_->create_publisher<msg_process::msg::DecisonSendData>("/serial_send_data", 50);

        // 给自瞄
        autoaim_pub = node_->create_publisher<msg_process::msg::DecisionToAutoaim>("/decision_to_autoaim", 100);

        // 导航
        way_point_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 50);
        // 下面不发给导航
        stop_pub = node_->create_publisher<std_msgs::msg::Bool>("/stop", 50);
        move_mode_pub = node_->create_publisher<std_msgs::msg::Int8>("/decision_mode", 1); // 0小陀螺开 2关 1慢速  //给导航的移动模式

        pursuit_data_pub = node_->create_publisher<msg_process::msg::PursuitData>("/pursuit_data", 100);  // 用于rosbag回放作日志分析
        decision_data_pub = node_->create_publisher<msg_process::msg::DecisionMsg>("/decision_data", 50); // 用于rosbag回放作日志分析

        // 初始化串口数据
        init_serial();
    }

    inline void stop_moving() // 用于暂停移动时发布停止消息，同时短暂for循环防止丢帧
    {
        try
        {
            for (int i = 0; i < 5; i++)
            {
                stop_.data = true;
                stop_pub->publish(stop_);
            }
            stop_.data = false;
            stop_pub->publish(stop_);
        }

        catch (const std::exception &e)
        {
            zh_log.error("stop_move error");
            zh_log.error(e.what());
        }
    }
    inline void decision_data()
    {
        decision_data_.is_in_block = DD.is_in_block;
        decision_data_.style = DD.style;
        decision_data_.in_hit = DD.in_hit;
        decision_data_.is_pursuiting = DD.is_pursuiting;
        decision_data_.rush = DD.rush;
        decision_data_.is_outposting = DD.is_outposting;
        decision_data_pub->publish(decision_data_);
    }

    inline void pathplan_base_tripod(int move_mode, int tripod_spin)
    {
        try
        {
            switch (move_mode)
            {
            case 0:
                decision_send_data_.spin_mode = 1;
                move_mode_data_.data = 0;
                cout << "开小陀螺-快速移动" << endl;
                break;
            case 1:
                decision_send_data_.spin_mode = 1;
                move_mode_data_.data = 1;
                cout << "开小陀螺-追击" << endl;
                break;
            case 2:
                decision_send_data_.spin_mode = 0;
                move_mode_data_.data = 2;
                cout << "关小陀螺-快速移动" << endl;
                break;
            case 6:
                decision_send_data_.spin_mode = 0;
                move_mode_data_.data = 6;
                break;
            default:
                decision_send_data_.spin_mode = 1;
                move_mode_data_.data = 0;
                cout << "开小陀螺-快速移动" << endl;
            }
            move_mode_pub->publish(move_mode_data_);
            decision_send_data_.tripod_spin = tripod_spin;
        }
        catch (const std::exception &e)
        {
            zh_log.error("pathplan_base_tripod error");
            zh_log.error(e.what());
        }
    }
    inline void move_mode(int a)
    {
        try
        {
            move_mode_data_.data = a;
            move_mode_pub->publish(move_mode_data_);
        }
        catch (const std::exception &e)
        {
            zh_log.error("move_mode error");
            zh_log.error(e.what());
        }
    }

    inline void pub_decision_point(const double x, const double y) // 给导航发点
    {
        try
        {
            DD.cur_decision_point_x = x;
            DD.cur_decision_point_y = y;
            way_point_.header.stamp = node_->now();
            way_point_.header.frame_id = "map";
            way_point_.pose.position.x = x; //+ 4.971;
            way_point_.pose.position.y = y; // + 7.5;
            way_point_.pose.position.z = 0;
            way_point_.pose.orientation.x = yawToQuaternion(DD.big_yaw)[1];
            way_point_.pose.orientation.y = yawToQuaternion(DD.big_yaw)[2];
            way_point_.pose.orientation.z = yawToQuaternion(DD.big_yaw)[3];
            way_point_.pose.orientation.w = yawToQuaternion(DD.big_yaw)[0];
            way_point_pub->publish(way_point_);
        }

        catch (const std::exception &e)
        {
            zh_log.error("pub_decision_point error");
            zh_log.error(e.what());
        }
    }

    void set_base_tripod_spin(int base, Tripod_Spin tripod);

    void set_posture(int posture);

    void confirm_free_respawn(bool is_confirm);

    void pub_decision_to_autoaim();

    void pub();

    void init_serial();

    // 范围内随机设点
    vector<float> randdom_point(const float &radius, const float &x, const float &y);
};

inline void MessagePublisher::set_base_tripod_spin(int base, Tripod_Spin tripod)
{
    try
    {
        decision_send_data_.spin_mode = base; // 给电控的小陀螺标志位1开2关0默认
        decision_send_data_.tripod_spin = static_cast<int>(tripod);
    }
    catch (const std::exception &e)
    {
        zh_log.error("pub set_base_tripod_spin error");
        zh_log.error(e.what());
    }
}

inline void MessagePublisher::set_posture(int posture)
{
    try
    {
        if (decision_send_data_.posture == posture && !is_first_posture_)
        {
            std::cout << "当前姿态为：" << decision_send_data_.posture << std::endl;
            return; // 姿态未改变且不是第一次设置，跳过发布
        }

        if (posture_timer.is_timeout() || is_first_posture_)
        {
            decision_send_data_.posture = posture;
            std::cout << "更换姿态为：" << posture << std::endl;
            posture_timer.reset();
            is_first_posture_ = false;
        }
    }
    catch (const std::exception &e)
    {
        zh_log.error("pub set_posture error");
        zh_log.error(e.what());
    }
}

inline std::vector<float> MessagePublisher::randdom_point(const float &radius, const float &x, const float &y) // 随机范围点（暂时弃用）
{
    std::srand(std::time(0));
    // 随机生成一个在 [0, radius] 范围内的半径
    float r = static_cast<float>(std::rand()) / RAND_MAX * radius;
    // 随机生成一个在 [0, 2π] 范围内的角度
    float radian = static_cast<float>(std::rand()) / RAND_MAX * 2 * 3.1415926;
    float x_ = r * std::cos(radian) + x;
    float y_ = r * std::sin(radian) + y;
    std::vector<float> point;
    point.push_back(x_);
    point.push_back(y_);
    return point;
}
inline void MessagePublisher::confirm_free_respawn(bool is_confirm)
{
    try
    {
        decision_send_data_.confirm_respawn = is_confirm;
    }
    catch (const std::exception &e)
    {
        zh_log.error("pub confirm_free_respawn error");
        zh_log.error(e.what());
    }
}

inline void MessagePublisher::pub_decision_to_autoaim()
{
    try
    {
        if (DD.targets.decision_target == 7)
        {
            decision_send_autoaim_.target = 0;
        } // 应自瞄要求 当没有目标时默认给英雄
        else
        {
            decision_send_autoaim_.target = DD.targets.decision_target;
        }
        decision_send_autoaim_.decision_shoot_mode       = DD.targets.Shoot_Mode;
        decision_send_autoaim_.decide_yaw = DD.big_yaw;                                        // 发送当前云台大yaw 计算全局坐标
        decision_send_autoaim_.approachable_state_vector = DD.approachable_state(DD.enemy_hp); // 发送可攻击状态
        autoaim_pub->publish(decision_send_autoaim_);
    }

    catch (const std::exception &e)
    {
        cout << "pub_to_autoaim error" << endl;
        std::cerr << e.what() << std::endl;
    }
}

void MessagePublisher::pub()
{
    try
    {
        if (!DD.is_outposting) // 在不攻击前哨时使用pitch调0
        {
            DD.pitch_mode = 0;
        }
        decision_send_data_.style = DD.style; // 返回给电控 用于日志记录
        if (DD.is_in_block && !DD.is_arrive)
        {
            decision_send_data_.decision_yaw_az = -0.01;
        } // 发0.01是为了控住云台
        else
        {
            decision_send_data_.decision_yaw_az = DD.decision_yaw_az;
        }
        decision_send_data_.pitch_mode = DD.pitch_mode;
        decision_send_data_.attack_rune = DD.to_rune; // 前往开符
        serial_pub->publish(decision_send_data_);
        decision_data();
    }
    catch (const std::exception &e)
    {
        zh_log.error("pub decision_send_serial error");
        zh_log.error(e.what());
    }

    try
    {
        if (DD.is_outposting)
        {
            DD.targets.decision_target = 4;
        }
        else if (DD.to_rune)
        {
            DD.targets.decision_target = 10;
        }
        pub_decision_to_autoaim();
    }
    catch (const std::exception &e)
    {
        zh_log.error("pub decision_send_autoaim error");
        zh_log.error(e.what());
        zh_log << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    }
}

void MessagePublisher::init_serial()
{
    decision_send_data_.spin_mode = 0;
    decision_send_data_.decision_yaw_az = 0;

    // decision_send_data_.odom_yaw=0;
    // decision_send_data_.detector_flag=0;
    decision_send_data_.pitch_mode = 4;
    decision_send_data_.confirm_respawn = 0;
    decision_send_data_.buy_respawn = 0;
    decision_send_data_.home_buy_ammo = 0;
    decision_send_data_.remote_buy_ammo_times = 0;
    decision_send_data_.remote_buy_hp_times = 0;
}

// inline MessagePublisher mes_puber; 改成指针
inline std::shared_ptr<MessagePublisher> mes_puber = nullptr;