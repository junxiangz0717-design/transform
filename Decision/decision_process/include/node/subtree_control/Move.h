//此节点的作用是检查是否切换子树，同时设置style
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"

#include <string>

using namespace std;
using namespace BT;

class Move : public SyncActionNode
{
public:
    Move(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~Move()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树"),
                InputPort<string>("start_point","点位")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        getInput("start_point",temp_subtree);
        if (temp_subtree  == subtree)
        {
            cout<<"开局移动"<<endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
    string subtree;
    string temp_subtree;
};
