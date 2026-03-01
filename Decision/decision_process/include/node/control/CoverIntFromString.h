#pragma once

#include "behaviortree_cpp/action_node.h"
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

using namespace BT;

class CoverIntFromString : public SyncActionNode
{
public:
    CoverIntFromString(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config) {}
    ~CoverIntFromString() override = default;

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("input_string"), OutputPort<std::vector<double>>("goal_points") };
    }

    NodeStatus tick() override
    {
        std::string input_string;
        if (!getInput("input_string", input_string))
        {
            throw BT::RuntimeError("missing required input [input_string]");
        }

        std::vector<double> goal_points = parseStringToDoubles(input_string);

        if (goal_points.empty())
        {
            throw BT::RuntimeError("goal_points is empty");
        }

        setOutput("goal_points", goal_points);

        std::cout << "Parsed goal points: ";
        for (const auto& point : goal_points)
        {
            std::cout << point << " ";
        }
        std::cout << std::endl;

        return NodeStatus::SUCCESS;
    }

private:
    std::vector<double> parseStringToDoubles(const std::string& str)
    {
        std::vector<double> points;
        std::stringstream ss(str);
        std::string item;
        while (std::getline(ss, item, ','))
        {
            try
            {
                double point = std::stod(item);
                points.push_back(point);
            }
            catch (const std::invalid_argument& e)
            {
                std::cerr << "Invalid argument: " << e.what() << std::endl;
            }
            catch (const std::out_of_range& e)
            {
                std::cerr << "Out of range: " << e.what() << std::endl;
            }
        }
        return points;
    }
};