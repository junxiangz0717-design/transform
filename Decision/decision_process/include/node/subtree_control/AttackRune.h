// 打符
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>


using namespace std;
using namespace BT;

class AttackRune : public SyncActionNode
{
public:
    AttackRune(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~AttackRune()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "ATTACK_RUNE")
        {
            cout<<"打神符"<<endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }
private:
string subtree;
};
