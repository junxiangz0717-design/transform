#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>

using namespace std;
using namespace BT;

class LowAmmo : public SyncActionNode
{
public:
    LowAmmo(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~LowAmmo()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "LOW_AMMO")
        {
            cout<<"低发弹量行为"<<endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
string subtree;
};