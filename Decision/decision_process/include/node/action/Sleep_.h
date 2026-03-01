#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <vector>
#include <algorithm>

using namespace std;
using namespace BT;

class Sleep_ : public SyncActionNode
{
public:
    Sleep_(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~Sleep_()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<int>("sleep_time","睡眠时间")};
    }
    NodeStatus tick() override
    {
        getInput("sleep_time",sleep_time);
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(sleep_time)));
        return NodeStatus::SUCCESS;
    }

private:

int sleep_time = 0;
};