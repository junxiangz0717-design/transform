#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>

using namespace std;
using namespace BT;

class InFort : public SyncActionNode
{
public:
    InFort(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~InFort()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "IN_FORT")
        {
            if(DD.targets.autoaim_target == 7){DD.decision_yaw_az = DD.config_az;}
            cout<<"在堡垒"<<endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
string subtree;
};