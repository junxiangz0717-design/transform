// 此节点的作用是检查是否切换子树
// 巡逻
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"

#include <string>

using namespace std;
using namespace BT;

class Finding : public SyncActionNode
{
public:
    Finding(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~Finding()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树"),
                InputPort<string>("area","搜寻方式")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        getInput("area",temp_subtree);
        if (temp_subtree  == subtree)
        {
            cout<<"巡逻"<<endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
    string subtree;
    string temp_subtree;
};
