// 在main.cpp里面实例化对象

#pragma once

#include "data/decision_data.h"
#include "data/enum.h"

#include "tool/tf.h"
#include "tool/logger/log.h"
#include "sub_pub/pub.h"
#include "msg_process/msg/autoaim_to_decision.hpp"
#include "msg_process/msg/receive_data.hpp"
#include "msg_process/msg/pursuit.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <iostream>
#include <chrono>
#include <vector>
#include <deque>
#include <utility>
#include <algorithm>
#include <cstdint>

bool can_arrive_flag = true;
bool pre_can_arrive_flag = true;

class DataSubscriber
{
private:
    // rclcpp::Subscriber step_blocked_sub; RMUL用不到
    // rclcpp::Subscriber is_down_step_sub; RMUL用不到

    rclcpp::Subscription<msg_process::msg::ReceiveData>::SharedPtr serial_datas_sub;
    rclcpp::Subscription<msg_process::msg::AutoaimToDecision>::SharedPtr autoaim_datas_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr cur_pos_datas_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_arrive_sub;
    rclcpp::Subscription<msg_process::msg::Pursuit>::SharedPtr pursuit_sub; // 仿真用
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_block_sub;

    std::shared_ptr<rclcpp::Node> node_;

public:
    DataSubscriber(std::shared_ptr<rclcpp::Node> node);
    ~DataSubscriber() = default;

    void serial_datas_callback(const msg_process::msg::ReceiveData::SharedPtr &serial_receive_data);
    void autoaim_datas_callback(const msg_process::msg::AutoaimToDecision::SharedPtr &autoaim_to_decision_data);
    void cur_pos_datas_callback(const nav_msgs::msg::Odometry::SharedPtr &odom_datas);
    void is_arrive_callback(const std_msgs::msg::Bool::SharedPtr &is_arrive_data);
    void pursuit_callback(const msg_process::msg::Pursuit::SharedPtr &pursuit_data);
    void inblock_callback(const std_msgs::msg::Bool::SharedPtr &in_block_data);
};

DataSubscriber::DataSubscriber(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
    // 使用 lambda 表达式替代 std::bind
    serial_datas_sub = node_->create_subscription<msg_process::msg::ReceiveData>(
        "/serial_receive_data", 10,
        [this](msg_process::msg::ReceiveData::SharedPtr msg)
        {
            this->serial_datas_callback(msg);
        });

    autoaim_datas_sub = node_->create_subscription<msg_process::msg::AutoaimToDecision>(
        "/autoaim_to_decision", 10,
        [this](msg_process::msg::AutoaimToDecision::SharedPtr msg)
        {
            this->autoaim_datas_callback(msg);
        });

    cur_pos_datas_sub = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg)
        {
            this->cur_pos_datas_callback(msg);
        });

    is_arrive_sub = node_->create_subscription<std_msgs::msg::Bool>(
        "/is_arrive", 10,
        [this](std_msgs::msg::Bool::SharedPtr msg)
        {
            this->is_arrive_callback(msg);
        });

    pursuit_sub = node_->create_subscription<msg_process::msg::Pursuit>(
        "/pursuit", 10,
        [this](msg_process::msg::Pursuit::SharedPtr msg)
        {
            this->pursuit_callback(msg);
        });

    is_block_sub = node_->create_subscription<std_msgs::msg::Bool>(
        "/in_block", 10,
        [this](std_msgs::msg::Bool::SharedPtr msg)
        {
            this->inblock_callback(msg);
        });
}

void DataSubscriber::inblock_callback(const std_msgs::msg::Bool::SharedPtr &in_block_data)
{
    if (in_block_data->data == 1)
    {
        DD.is_in_block = 1;
        DD.block_timer.reset();
    }
    else if (!DD.block_timer.is_timeout())
    {
        DD.is_in_block = 1;
    }
    else
    {
        DD.is_in_block = 0;
    }
}

void DataSubscriber::pursuit_callback(const msg_process::msg::Pursuit::SharedPtr &pursuit_data)
{
    DD.xx = pursuit_data->x;
    DD.yy = pursuit_data->y;
    DD.pub = 1;
}

void DataSubscriber::is_arrive_callback(const std_msgs::msg::Bool::SharedPtr &is_arrive_data)
{
    DD.is_arrive = is_arrive_data->data;
}

void DataSubscriber::autoaim_datas_callback(const msg_process::msg::AutoaimToDecision::SharedPtr &autoaim_to_decision_data)
{
    try
    {
        double temp_x = DD.cur_x;
        double temp_y = DD.cur_y;

        // 刷新自瞄连接超时器（RMUL固定3位）
        // 在锁定目标的时候调用is_time_out函数，确保目标数据来自超时前的自瞄数据
        DD.autoaim_connect_timer.reset();

        if (autoaim_to_decision_data->target_vector.size() != 7 ||
            autoaim_to_decision_data->distance_vector.size() != 7 ||
            autoaim_to_decision_data->polar_r_vector.size() != 7 ||
            autoaim_to_decision_data->polar_angle_vector.size() != 7)
        {
            cout << "自瞄数组长度非法！" << endl;
            cout << "target_vector: " << autoaim_to_decision_data->target_vector.size() << endl;
            cout << "distance_vector: " << autoaim_to_decision_data->distance_vector.size() << endl;
            cout << "polar_r_vector: " << autoaim_to_decision_data->polar_r_vector.size() << endl;
            cout << "polar_angle_vector: " << autoaim_to_decision_data->polar_angle_vector.size() << endl;
            return;
        }
        DD.targets.Shoot_Mode       = autoaim_to_decision_data->shoot_mode;
        DD.targets.autoaim_target = autoaim_to_decision_data->target;
        DD.targets.Target_Mode = autoaim_to_decision_data->target_mode;
        DD.targets.autoaim_num = autoaim_to_decision_data->shootable_num;
        // 筛选出自瞄视野内出现的装甲板编号（只要数量大于0，不论多少都只视为存在）
        std::vector<uint8_t> enemy;
        for (size_t i = 0; i < autoaim_to_decision_data->target_vector.size(); ++i)
        {
            if (autoaim_to_decision_data->target_vector[i] > 0 && DD.is_approachable(static_cast<Target>(i)))
                enemy.push_back(static_cast<uint8_t>(i));
        }

        // 将出现的装甲板按距离从小到大排序放入 DD.targets 中
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "size: %zu", enemy.size());
        DD.targets.targets_vector.clear();
        std::sort(enemy.begin(), enemy.end(), [&](uint8_t a, uint8_t b)
                  { return autoaim_to_decision_data->distance_vector[a] < autoaim_to_decision_data->distance_vector[b]; });
        for (auto &e : enemy)
        {
            DD.targets.targets_vector.push_back(static_cast<Target>(e));
        }

        // 距离
        DD.targets.targets_distance_vector.assign(autoaim_to_decision_data->distance_vector.begin(), autoaim_to_decision_data->distance_vector.end());
        std::vector<double> b = DD.targets.targets_distance_vector;
        for (int i = 0; i < 7; ++i)
        {
            // 目标消失记录
            if ((autoaim_to_decision_data->target_vector[i] == 0u ||
                 autoaim_to_decision_data->distance_vector[i] == .0 ||
                 autoaim_to_decision_data->polar_r_vector[i] == .0) &&
                DD.is_approachable(static_cast<Target>(i)))
            {
                if (DD.targets.targets_x_y[2 * i] != .0 ||
                    DD.targets.targets_x_y[2 * i + 1] != .0)
                {
                    DD.targets.targets_disappear_x_y[2 * i] = DD.targets.targets_x_y[2 * i];
                    DD.targets.targets_disappear_x_y[2 * i + 1] = DD.targets.targets_x_y[2 * i + 1];
                    DD.targets.targets_disappear_timeout[i].reset();
                    DD.targets.targets_x_y[2 * i] = .0;
                    DD.targets.targets_x_y[2 * i + 1] = .0;
                }
                else
                {
                    DD.targets.targets_disappear_x_y[2 * i] = .0;
                    DD.targets.targets_disappear_x_y[2 * i + 1] = .0;
                }
                continue;
            }
            // 相对极坐标解算到Map坐标系
            /*一代车*/
            // double absolute_yaw = DD.big_yaw + autoaim_to_decision_data->polar_angle_vector[i]*M_PI/180; // 哨兵与目标连线相对于Map坐标系的绝对角度
            /*二代车*/
            double absolute_yaw = DD.big_yaw + autoaim_to_decision_data->polar_angle_vector[i] * M_PI / 180;
            DD.targets.targets_x_y[2 * i] = temp_x + autoaim_to_decision_data->polar_r_vector[i] * cos(absolute_yaw);     // 哨兵与目标连线相对于Map坐标系的x
            DD.targets.targets_x_y[2 * i + 1] = temp_y + autoaim_to_decision_data->polar_r_vector[i] * sin(absolute_yaw); // 哨兵与目标连线相对于Map坐标系的y
            // if(i!=0)
            // {
            //     if(DD.targets.waypoint[i].size()==10){DD.targets.waypoint[i].pop_front();}
            //     DD.targets.waypoint[i].push_back({DD.targets.targets_x_y[2*i], DD.targets.targets_x_y[2*i+1]});
            //     if(checker(DD.targets.waypoint[i])){DD.targets.waypoint[i].clear();}
            // }
        }
        DD.detector_connect_timer.reset();
        if (autoaim_to_decision_data->detector_target_vector.size() != 7 ||
            autoaim_to_decision_data->detector_distance_vector.size() != 7 ||
            autoaim_to_decision_data->detector_polar_r_vector.size() != 7 ||
            autoaim_to_decision_data->detector_polar_angle_vector.size() != 7)
        {
            cout << "感知相机数组长度非法！" << endl;
            cout << "target_vector: " << autoaim_to_decision_data->detector_target_vector.size() << endl;
            cout << "distance_vector: " << autoaim_to_decision_data->detector_distance_vector.size() << endl;
            cout << "polar_r_vector: " << autoaim_to_decision_data->detector_polar_r_vector.size() << endl;
            cout << "polar_angle_vector: " << autoaim_to_decision_data->detector_polar_angle_vector.size() << endl;
            return;
        }
        // 筛选出感知视野内出现的装甲板编号（只要数量大于0，不论多少都只视为存在）

        std::vector<uint8_t> detector_enemy;

        for (size_t i = 0; i < autoaim_to_decision_data->detector_target_vector.size(); ++i)
        {
            if (autoaim_to_decision_data->detector_target_vector[i] > 0 && DD.is_approachable(static_cast<Target>(i)))
                detector_enemy.push_back(static_cast<uint8_t>(i));
        }

        // 将出现的装甲板按距离从小到大排序放入 DD.targets 中
        DD.targets.detector_targets_vector.clear();
        std::sort(detector_enemy.begin(), detector_enemy.end(), [&](uint8_t a, uint8_t b)
                  { return autoaim_to_decision_data->detector_distance_vector[a] < autoaim_to_decision_data->detector_distance_vector[b]; });
        for (auto e : detector_enemy)
        {
            DD.targets.detector_targets_vector.push_back(static_cast<Target>(e));
        }
        // 距离
        DD.targets.detector_distance_vector.assign(autoaim_to_decision_data->detector_distance_vector.begin(), autoaim_to_decision_data->detector_distance_vector.end());
        for (int i = 0; i < 7; ++i)
        {
            // 相对极坐标解算到Map坐标系

            double absolute_yaw = DD.big_yaw + autoaim_to_decision_data->detector_polar_angle_vector[i] * M_PI / 180; // 哨兵与目标连线相对于Map坐标系的绝对角度
            DD.targets.detector_angle[i] = absolute_yaw;
            DD.targets.detector_targets_x_y[2 * i] = temp_x + autoaim_to_decision_data->detector_polar_r_vector[i] * cos(absolute_yaw);     // 哨兵与目标连线相对于Map坐标系的x
            DD.targets.detector_targets_x_y[2 * i + 1] = temp_y + autoaim_to_decision_data->detector_polar_r_vector[i] * sin(absolute_yaw); // 哨兵与目标连线相对于Map坐标系的y
        }
    }

    // mes_puber.autoaim_target_feedback();
    catch (const std::exception &e)
    {
        zh_log.error("autoaimCallback error: ");
        zh_log.error(e.what());
    }
}

void DataSubscriber::serial_datas_callback(const msg_process::msg::ReceiveData::SharedPtr &serial_receive_data)
{

    try
    {
        DD.color = int(serial_receive_data->color);

        /**
         * 其他机器人血量参数   暂时不改编号
         */
        DD.our_hero_hp = serial_receive_data->our_hero_hp;
        DD.our_engineer_hp = serial_receive_data->our_engineer_hp;
        DD.our_foot_3_hp = serial_receive_data->our_foot_3_hp;
        DD.our_foot_4_hp = serial_receive_data->our_foot_4_hp;
        DD.our_base_hp = serial_receive_data->hp_base;

        DD.our_hero_x = (DD.color ? 25 - serial_receive_data->our_hero_x : serial_receive_data->our_hero_x);
        DD.our_hero_y = (DD.color ? 15 - serial_receive_data->our_hero_y : serial_receive_data->our_hero_y);
        // DD.our_engineer_x    = serial_receive_data->our_engineer_x;
        // DD.our_engineer_y    = serial_receive_data->our_engineer_y;
        DD.our_foot_3_x = (DD.color ? 25 - serial_receive_data->our_foot_3_x : serial_receive_data->our_foot_3_x);
        DD.our_foot_3_y = (DD.color ? 15 - serial_receive_data->our_foot_3_y : serial_receive_data->our_foot_3_y);
        // DD.our_foot_4_x      = (DD.color ? 25-serial_receive_data->our_foot_4_x : serial_receive_data->our_foot_4_x);
        // DD.our_foot_4_y      = (DD.color ? 15-serial_receive_data->our_foot_4_y : serial_receive_data->our_foot_4_y);

        // 仅RMUL（0无，1我方，2敌方，3双方）
        DD.is_center_area = serial_receive_data->is_center_area;

        DD.enemy_outpost_hp = serial_receive_data->hp_enemy_outpost;
        DD.enemy_base_hp = serial_receive_data->hp_enemy_base;
        cout << "剩余能量:" << DD.remain_energy << endl;
        cout << "当前能量:" << DD.power.getTotalEnergy() << endl;
        if (DD.power.getTotalEnergy() > 20100)
        {
            DD.is_weak = 1;
        }
        if (DD.config_is_serial_hp && DD.game_period == 4)
        {
            DD.enemy_hero_hp = DD.set_hero.updateHealth(serial_receive_data->enemy_hero_hp);
            // int a=DD.set_engineer.updateHealth(serial_receive_data->enemy_engineer_hp);int b= serial_receive_data->enemy_engineer_hp ;
            DD.enemy_engineer_hp = DD.set_engineer.updateHealth(serial_receive_data->enemy_engineer_hp);
            DD.enemy_foot_3_hp = DD.set_foot_3.updateHealth(serial_receive_data->enemy_foot_3_hp);
            DD.enemy_foot_4_hp = DD.set_foot_4.updateHealth(serial_receive_data->enemy_foot_4_hp);
            DD.enemy_sentry_hp = DD.set_sentry.update_sentry_Health(serial_receive_data->enemy_sentry_hp);
            cout << DD.enemy_hero_hp << " " << DD.enemy_engineer_hp << " " << DD.enemy_foot_3_hp << " " << DD.enemy_foot_4_hp << " " << DD.enemy_sentry_hp << endl;
        }

        // 0血量校验，防止一帧0血量干爆
        if (serial_receive_data->hp_sentry != 0)
        {
            DD.hp_sentry = serial_receive_data->hp_sentry; // 0血量校验，防止一帧0血量干爆
            DD.hp_0_timer.reset();
        }
        else if (serial_receive_data->hp_sentry == 0 && DD.hp_0_timer.is_timeout())
        {
            DD.hp_sentry = 0;
            cout << "0血量校验超时，认为哨兵阵亡！ " << endl;
        }

        DD.time = serial_receive_data->time;
        DD.game_period = int(serial_receive_data->game_period);

        DD.enemy_hp[0] = DD.enemy_hero_hp;
        DD.enemy_hp[1] = DD.enemy_engineer_hp;
        DD.enemy_hp[2] = DD.enemy_foot_3_hp;
        DD.enemy_hp[3] = DD.enemy_foot_4_hp;
        DD.enemy_hp[4] = DD.enemy_outpost_hp;
        DD.enemy_hp[5] = DD.enemy_base_hp;
        DD.enemy_hp[6] = DD.enemy_sentry_hp;

        // 云台手给点
        DD.command_x = serial_receive_data->goalx;
        DD.command_y = serial_receive_data->goaly;
        if (DD.command_y > 0.001f && DD.command_x > 0.001f)
        {
            DD.have_goal = true;
            if (DD.command_x != DD.pre_command_x || DD.command_y != DD.pre_command_y)
            {
                DD.pre_command_x = DD.command_x;
                DD.pre_command_y = DD.command_y;
            }
        }
        else
        {
            DD.have_goal = false;
        }

        // 自身状态
        DD.defence_buff = serial_receive_data->defence_buff;
        DD.strategy = serial_receive_data->style;
        if (int(serial_receive_data->hp_sentry) == 0)
            DD.death_time = chrono::system_clock::now(); // 记录死亡时间
        if (int(serial_receive_data->hp_sentry) == 1001)
            DD.is_invincible = 1; // 无敌状态
        else
            DD.is_invincible = 0;
        DD.ammo = int(serial_receive_data->ammo);
        DD.is_no_ammo = 0;
        DD.can_respawn = int(serial_receive_data->can_free_respawn);

        DD.delta_yaw = serial_receive_data->delta_yaw; // delta_yaw角

        // 比赛信息
        DD.is_in_add_area = int(serial_receive_data->is_in_add_area); // 仅RMUL

        DD.is_in_fort = int(serial_receive_data->is_in_fort);
        if (DD.self_pos->is_in(area_manager["补给区"]))
        {
            DD.is_in_add_area = 1;
        }
        float chassis_power = serial_receive_data->chassis_power;
        if (DD.game_period == 4)
        {
            DD.power.updatePower(chassis_power);
        }
        // 将剩余时间转换为正计时 RMUC
        // DD.start_time = 420 - serial_receive_data->time;
        // RMUL
        DD.start_time = 300 - serial_receive_data->time;
        // 可获取弹量更新
        DD.available_ammo = DD.start_time / 60;
        /** 雷达标定信息 **/
        if (serial_receive_data->radar_data.size() == 12)
        {
            for (int i = 0; i < 6; ++i)
            {
                // 接收雷达标定信息
                DD.targets.radar_x_y[2 * i] = serial_receive_data->radar_data[2 * i];
                DD.targets.radar_x_y[2 * i + 1] = serial_receive_data->radar_data[2 * i + 1];
            }
        }
        else
            cout << "雷达标定信息非法！" << endl;
        DD.rune = serial_receive_data->rune;
    }
    catch (const std::exception &e)
    {
        zh_log.error("serialCallback error: ");
        zh_log.error(e.what());
    }
}

void DataSubscriber::cur_pos_datas_callback(const nav_msgs::msg::Odometry::SharedPtr &odom_datas)
{
    // odom -> map 坐标系偏移修正（rmul暂存）
    DD.cur_x = odom_datas->pose.pose.position.x + 0.85;
    DD.cur_y = odom_datas->pose.pose.position.y - 0.80;
    orientation_to_yaw(odom_datas, DD.big_yaw); // 获取云台大yaw角，相对地图坐标系
}