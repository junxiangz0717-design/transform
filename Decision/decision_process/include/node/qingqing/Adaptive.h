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
    static PortsList providedPorts()
    {
        return {OutputPort<vector<double>>("goal_points", "追击点")};
    }
    NodeStatus tick() override
    {
        point.clear();
        waypoint.clear();
        if (DD.targets.autoaim_target == 7)
        {
            DD.is_pursuiting = 0;
            cout << "无目标" << endl;
            waypoint.push_back(0);
            waypoint.push_back(0);
            setOutput("goal_points", waypoint);
            return NodeStatus::SUCCESS;
        }
        if (DD.targets.targets_point[DD.targets.autoaim_target]->get_x() != 0 && DD.targets.targets_point[DD.targets.autoaim_target]->get_y() != 0) // 目标未消失
        {
            DD.pursuit_timer.reset();
            DD.is_pursuiting = 1;
            // 移动姿态
            DD.posture = 3;
            mes_puber->set_posture(DD.posture);

            target = ER.pixel2qingqing(ER.find_visible_target(ER.qingqing2pixel(DD.cur_x, DD.cur_y),
                                                         ER.qingqing2pixel(DD.targets.targets_point[DD.targets.autoaim_target]->get_x(),
                                                                      DD.targets.targets_point[DD.targets.autoaim_target]->get_y())));
            cout << "(" << target.first << "," << target.second << ")" << endl;
            mes_puber->pursuit_data_.agent_x = ER.qingqing2pixel(DD.cur_x, DD.cur_y).x;
            mes_puber->pursuit_data_.agent_y = ER.qingqing2pixel(DD.cur_x, DD.cur_y).y;
            mes_puber->pursuit_data_.enemy_x = ER.qingqing2pixel(DD.targets.targets_point[DD.targets.autoaim_target]->get_x(), DD.targets.targets_point[DD.targets.autoaim_target]->get_y()).x;
            mes_puber->pursuit_data_.enemy_y = ER.qingqing2pixel(DD.targets.targets_point[DD.targets.autoaim_target]->get_x(), DD.targets.targets_point[DD.targets.autoaim_target]->get_y()).y;
            mes_puber->pursuit_data_.target_x = ER.find_visible_target(ER.qingqing2pixel(DD.cur_x, DD.cur_y), ER.qingqing2pixel(DD.targets.targets_point[DD.targets.autoaim_target]->get_x(),
                                                                                                                      DD.targets.targets_point[DD.targets.autoaim_target]->get_y()))
                                                    .x;
            mes_puber->pursuit_data_.target_y = ER.find_visible_target(ER.qingqing2pixel(DD.cur_x, DD.cur_y), ER.qingqing2pixel(DD.targets.targets_point[DD.targets.autoaim_target]->get_x(),
                                                                                                                      DD.targets.targets_point[DD.targets.autoaim_target]->get_y()))
                                                    .y;
            mes_puber->pursuit_data_.agent_real_x = DD.cur_x;
            mes_puber->pursuit_data_.agent_real_y = DD.cur_y;
            mes_puber->pursuit_data_.enemy_real_x = DD.targets.targets_point[DD.targets.autoaim_target]->get_x();
            mes_puber->pursuit_data_.enemy_real_y = DD.targets.targets_point[DD.targets.autoaim_target]->get_y();
            mes_puber->pursuit_data_.target_real_x = target.first;
            mes_puber->pursuit_data_.target_real_y = target.second;
            mes_puber->pursuit_data_pub->publish(mes_puber->pursuit_data_);

            // waypoint.push_back(target.first);
            // waypoint.push_back(target.second);
            // setOutput("goal_points",waypoint);
            if (DD.strategy == 0 && DD.start_time >= 60)
            {
                waypoint.push_back(target.first);
                waypoint.push_back(target.second);
                setOutput("goal_points", waypoint);
            }
            else
            {
                waypoint.push_back(DD.cur_x);
                waypoint.push_back(DD.cur_y);
                setOutput("goal_points", waypoint);
            }
        }
        return NodeStatus::SUCCESS;
    }
};
