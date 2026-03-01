// 打前哨站
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>


using namespace std;
using namespace BT;

class AttackOutPost : public SyncActionNode
{
public:
    AttackOutPost(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~AttackOutPost()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "ATTACK_OUTPOST")
        {
            cout<<"打前哨"<<endl;
            return NodeStatus::SUCCESS;
        }
        DD.is_outposting = false;
        return NodeStatus::FAILURE;
    }
private:
string subtree;
};
