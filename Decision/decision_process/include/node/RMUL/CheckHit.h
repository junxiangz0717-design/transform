//
// Created by zh on 24-3-26.
//
#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include "sub_pub/pub.h"

using namespace std;
using namespace BT;

class CheckHit: public ConditionNode
{
private:
    // 所有受击检测器共用同一个先前血量
    static int pre_hp_;
    
    int last_time_ = 3; // 受击后受击状态的持续时间
    int hit_threshold = 10; // 受击检测阈值,两次检测间受到伤害大于等于该阈值才认为受击
    
    chrono::seconds hit_last_duration_;
    chrono::system_clock::time_point hit_state_reset_time_point_;

    bool being_hit = false;
    
public:
    CheckHit(const string& name, const NodeConfig& config): ConditionNode(name, config)
    {
        hit_last_duration_ = chrono::seconds(last_time_);
    }
    ~CheckHit() = default;
    
    static PortsList providedPorts(){return{ };}
    
    NodeStatus tick() override
    {
        // 无敌状态不受击检测
        if (DD.hp_sentry == 1001||DD.is_invincible||DD.defence_buff >= 100)
        {
            DD.in_hit = 0;
            return NodeStatus::SUCCESS;
        }
    
        // 换算成防御增益前的伤害
        if (DD.hp_sentry <= pre_hp_ - double(hit_threshold)/100*(100-DD.defence_buff))
        {
            pre_hp_ = DD.hp_sentry;
            hit_state_reset_time_point_ = chrono::system_clock::now()+=(hit_last_duration_);
            cout << "哨兵受击！" << endl;
            return NodeStatus::SUCCESS;
        }
        
        if (chrono::system_clock::now() < hit_state_reset_time_point_)
        {
            DD.in_hit = 1;
            if ((DD.hp_sentry < 150 && DD.is_in_center_area) || DD.hp_sentry < 110)
            {
                // 防御姿态
                DD.posture = 2;
                mes_puber->set_posture(DD.posture);
            }
            
            cout << "哨兵仍处于受击后" << last_time_ << "s内的受击状态" << endl;
            auto timestamp = hit_state_reset_time_point_ - chrono::system_clock::now();
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(timestamp % std::chrono::minutes(1)).count();
            auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp % std::chrono::seconds(1)).count();
            cout << "还剩" << seconds << "." << milliseconds << "s" << "解除受击状态" << endl;
            return NodeStatus::SUCCESS;
        }
        DD.in_hit =0;
        pre_hp_ = DD.hp_sentry;
        cout << "哨兵未受到高于阈值的伤害" << endl;
        return NodeStatus::SUCCESS;
    }
};

int CheckHit::pre_hp_ = 400;