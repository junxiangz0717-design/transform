#pragma once

#include "behaviortree_cpp/action_node.h"
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

using namespace BT;

class LoopDecisionPoint : public SyncActionNode
{
public:
    LoopDecisionPoint(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config), current_index_(0) {}
    ~LoopDecisionPoint() override = default;

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("queue"), OutputPort<std::vector<double>>("value") };
    }

    NodeStatus tick() override
    {
        std::string queue_str;
        if (!getInput("queue", queue_str))
        {
            throw BT::RuntimeError("missing required input [queue]");
        }

        std::vector<std::pair<double, double>> points = parseQueue(queue_str);

        if (points.empty())
        {
            throw BT::RuntimeError("queue is empty");
        }

        // 循环读取点
        if (current_index_ >= points.size())
        {
            current_index_ = 0;
        }

        std::vector<double> goal_points = { points[current_index_].first, points[current_index_].second };
        setOutput("value", goal_points);

        std::cout << "Current goal point: (" << goal_points[0] << ", " << goal_points[1] << ")" << std::endl;

        current_index_++;
        return NodeStatus::SUCCESS;
    }

private:
    size_t current_index_;
    std::vector<std::pair<double, double>> parseQueue(const std::string& str)
    {
        std::vector<std::pair<double, double>> points;
        std::stringstream ss(str);
        std::string item;
        while (std::getline(ss, item, ';'))
        {
            std::stringstream item_ss(item);
            std::string x_str, y_str;
            if (std::getline(item_ss, x_str, ',') && std::getline(item_ss, y_str, ','))
            {
                double x = std::stod(x_str);
                double y = std::stod(y_str);
                points.emplace_back(x, y);
            }
        }
        return points;
    }
};