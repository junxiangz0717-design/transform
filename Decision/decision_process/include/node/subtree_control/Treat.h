// 回血
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>

using namespace std;
using namespace BT;

class Treat : public SyncActionNode
{
public:
    Treat(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~Treat()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "TREATING")
        {
            DD.hp_state = 2;
            cout<<"回血中"<<endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
string subtree;
};