#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "data/enum.h"

using namespace std;
using namespace BT;


class CheckFort : public ConditionNode 
{
public:
    CheckFort(const string &name, const NodeConfig &config) : ConditionNode(name, config) {}
    
    ~CheckFort() override = default;
    
    static PortsList providedPorts()
    {
        return {};
    }
    
    NodeStatus tick() override
    {
        if(open){return NodeStatus::SUCCESS;}
        if(DD.start_time >= 180 && DD.enemy_outpost_hp == 0 && DD.enemy_base_hp > 2000 && 
            checkZeroCount(DD.enemy_foot_3_hp, DD.enemy_foot_4_hp, DD.enemy_sentry_hp) && 
            DD.hp_sentry > 200 &&
            DD.foot_3->is_in(area_manager["前半场"]) &&
            DD.foot_4->is_in(area_manager["前半场"]))
        {
            DD.state = State::TO_FORT;
        }
        else
        {
            DD.state = State::FINDING;
            return NodeStatus::SUCCESS;
        }
        if((DD.foot_3->is_in(area_manager["堡垒"]) || 
           DD.foot_4->is_in(area_manager["堡垒"]) ||
           DD.self_pos->is_in(area_manager["堡垒"])))
        {
            if(flag)
            {
                fort_timer.reset();
                flag = false;
            }
            timeout_timer.reset();
            fort_timer.restart();
        }
        else
        {
            if(timeout_timer.is_timeout()){flag = true;}
            else{fort_timer.pause();}
        }
        if(fort_timer.is_2_timeout()){open = 1;DD.state = State::FINDING;}
        return NodeStatus::SUCCESS;
    }   
    int checkZeroCount(int a, int b, int c) {return (a == 0) + (b == 0) + (c == 0) >= 1;}
private:
    int flag =true;
    zh_timer fort_timer{25000}; //堡垒区计时器
    zh_timer timeout_timer{3000}; //超时计时器
    int open = 0;
};