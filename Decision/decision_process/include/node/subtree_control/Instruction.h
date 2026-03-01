// 云台手下指令
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>

using namespace std;
using namespace BT;

class Instruction : public SyncActionNode
{
public:
    Instruction(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~Instruction()override=default;
    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("subtree", "当前子树"),
            OutputPort<std::string>("instruction", "云台手点位")
        };
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "INSTRUCTION")
        {
            std::string instruction = std::to_string(DD.command_x) + "," +
                                      std::to_string(DD.command_y);
            setOutput("instruction", instruction);
            cout<<"云台手下达指令"<<endl;
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
string subtree;
};