// 无弹量
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>

using namespace std;
using namespace BT;

class NoAmmo : public SyncActionNode
{
public:
    NoAmmo(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~NoAmmo()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "NO_AMMO")
        {
            cout<<"无弹行为"<<endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
string subtree;
};