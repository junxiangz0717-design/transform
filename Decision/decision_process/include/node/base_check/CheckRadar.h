#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include "Pursuit/include/pursuit.h"

#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;
using namespace BT;

class CheckRadar : public ConditionNode
{
private:
    vector<pair<double, double>> radar_x_y;

public:
    CheckRadar(const string &name, const NodeConfig &config) : ConditionNode(name, config) {}

    ~CheckRadar() override = default;

    static PortsList providedPorts()
    {
        return {OutputPort<string>("point_to_find", "雷达联动点位")};
    }

    NodeStatus tick() override
    {
        if (!DD.radar_arrive)
        {
            return NodeStatus::SUCCESS;
        }
        for (auto &point : DD.targets.radar_point)
        {
            pair<double, double> radar_x_y = ER.pixel2map(ER.get_radar_target(ER.map2pixel(point->get_x(), point->get_y())));
            if (radar_x_y.first <= 0 || radar_x_y.first >= 28 || radar_x_y.second <= 0 || radar_x_y.second >= 15)
            {
                DD.radar_point_vec.push_back(make_pair(0, 0));
            }
            else
            {
                DD.radar_point_vec.push_back(ER.pixel2map(ER.get_radar_target(ER.map2pixel(point->get_x(), point->get_y()))));
            }
        }
        for (size_t i = 0; i < DD.radar_point_vec.size(); i++)
        {
            if (DD.radar_point_vec[i].first != 0 && DD.radar_point_vec[i].second != 0)
            {
                DD.radar_have = 1;
                DD.radar_arrive = 0;
                DD.radar_point.first = DD.radar_point_vec[i].first;
                DD.radar_point.second = DD.radar_point_vec[i].second;
                return NodeStatus::SUCCESS;
            }
        }
        DD.radar_have = 0;
        return NodeStatus::SUCCESS;
    }
};