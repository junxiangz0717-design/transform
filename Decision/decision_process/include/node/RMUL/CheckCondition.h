#pragma once

#include "behaviortree_cpp/condition_node.h"

class CheckCondition : public BT::ConditionNode
{
public:
    CheckCondition(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("value",0,"是否跳过")};
    }

    BT::NodeStatus tick() override
    {
        int value;
        if (!getInput<int>("value" ,value))
        {
            value = 0;//默认为不跳过
            std::cout<<"value:"<<value<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        else if(value == 0)
        {
            std::cout<<"value:"<<value<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        else 
        {
            std::cout<<"value:"<<value<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }

    }
private:
};