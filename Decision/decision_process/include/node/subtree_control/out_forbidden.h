#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>

using namespace std;
using namespace BT;

class out_forbidden : public SyncActionNode
{
public:
    out_forbidden(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~out_forbidden()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "OUT_FORBIDDEN")
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
string subtree;
};