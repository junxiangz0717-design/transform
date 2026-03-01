
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"

using namespace BT;
using namespace std;

class SetStyle: public SyncActionNode
{

public:
    SetStyle(const string& name, const NodeConfig& config): SyncActionNode(name, config)
    {};
    ~SetStyle() override = default;
  
    static PortsList providedPorts()
    {
        return
        {};
    }

    BT::NodeStatus tick() override
    {

        if(DD.targets.autoaim_target != 7 && DD.style == Style::CHARGE && DD.self_pos->is_in(area_manager["对面家"]))
        {DD.find_target_timer.reset();}
        if(DD.find_target_timer.is_timeout())
        {
            DD.style = Style::NORMAL;
        }
        else
        {
            if(DD.foot_3->is_in(area_manager["前压区"]) && DD.foot_4->is_in(area_manager["前压区"]))
            {
                DD.style = Style::CHARGE;
            }
            else
            {
                DD.style = Style::NORMAL;
            }
        }
        if(DD.our_base_hp <= 1800 || DD.is_weak ){DD.style = Style::DEFENCE;}
        if(DD.hero->is_in(area_manager["前压区"])){DD.style = Style::CHARGE;}
        cout<<DD.style<<endl;
        return NodeStatus::SUCCESS;
        
    };
private:
    int flag = 1;
};