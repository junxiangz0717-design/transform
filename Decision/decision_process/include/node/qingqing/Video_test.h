#pragma once
// 已加姿态
#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include "tool/time/zh_timer.h"
#include <vector>
#include <algorithm>
#include "semantic_map/include/semantic_map.h"
#include "Pursuit/include/pursuit.h"
#include <opencv2/opencv.hpp>
#include <sub_pub/pub.h>

using namespace std;
using namespace BT;

class Adaptive : public SyncActionNode
{
private:
    vector<double> point;
    cv::Point enemy_point;
    cv::Point agent;
    vector<double> waypoint;
    pair<double, double> target;

public:
    Adaptive(const std::string &name, const NodeConfig &config) : SyncActionNode(name, config) {}
    ~Adaptive() override = default;
    static PortsList providedPorts() {}
    NodeStatus tick() override
    {
        if (DD.targets.autoaim_target != 7)
        {
            DD.posture = 1; // 设置为攻击姿态
            mes_puber->set_posture(DD.posture);
            return NodeStatus::SUCCESS;
        }
        else if (DD.hp_sentry < 150)
        {
            DD.posture = 2; // 设置为防守姿态
            mes_puber->set_posture(DD.posture);
            return NodeStatus::SUCCESS;
        }
        else
        {
            DD.posture = 0; // 设置为移动姿态
            mes_puber->set_posture(DD.posture);
            return NodeStatus::SUCCESS;
        }
    }
    
};
