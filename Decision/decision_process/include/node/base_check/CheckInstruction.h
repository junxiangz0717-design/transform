#pragma once
#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>
using namespace std;
using namespace BT;

class CheckInstruction : public ConditionNode 
{
public:
    CheckInstruction(const string &name, const NodeConfig &config) : ConditionNode(name, config) {}
    
    ~CheckInstruction() override = default;
    
    static PortsList providedPorts()
    {
        return {OutputPort<string>("instruction","指令位置")};
    }
    
    NodeStatus tick() override
    {
        if(DD.have_goal)
        {
            if (DD.is_arrive)
            {
                DD.state = State::FINDING;
                DD.have_goal = false; 
                return NodeStatus::SUCCESS;
            }
            string instruction = to_string(DD.command_x) + "," + to_string(DD.command_y);
            setOutput("instruction", instruction);
            DD.state = State::INSTRUCTION;
        }
        return NodeStatus::SUCCESS;
    }
};