#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include "sub_pub/pub.h"

#include <iostream>
#include <chrono>

using namespace std;
using namespace BT;

class CheckRune : public ConditionNode 
{
private:
    zh_timer drop_timer{300};

public:
    CheckRune(const string &name, const NodeConfig &config) : ConditionNode(name, config) {drop_timer.reset();}
    ~CheckRune() override = default;
    
    static PortsList providedPorts() {return{ };};
    
    NodeStatus tick() override
    {
        if(DD.targets.autoaim_num != 0)
        {
            DD.decision_yaw_az = 0;
            return NodeStatus::SUCCESS;
        }
        else
        {
            DD.decision_yaw_az = DD.config_az;
            return NodeStatus::FAILURE;
        }
       
    }
};
