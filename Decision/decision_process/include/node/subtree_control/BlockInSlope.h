#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>

using namespace std;
using namespace BT;

class BlockInSlope : public SyncActionNode
{
public:
    BlockInSlope(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~BlockInSlope()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "BLOCK_IN_SLOPE")
        {
            cout<<"无实体弹行为"<<endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
string subtree;
};