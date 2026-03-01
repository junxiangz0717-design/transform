// 前往堡垒
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>

using namespace std;
using namespace BT;

class ToFort : public SyncActionNode
{
public:
    ToFort(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~ToFort()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "TO_FORT")
        {
            cout<<"前往堡垒中"<<endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
string subtree;
};