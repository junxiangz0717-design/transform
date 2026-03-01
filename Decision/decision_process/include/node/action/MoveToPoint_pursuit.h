#pragma once
// 已加姿态
#include "behaviortree_cpp/action_node.h"
#include "sub_pub/pub.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include "tool/time/zh_timer.h"
#include <vector>
#include <cstdlib>  



using namespace std;
using namespace BT;

class MoveToPoint_pursuit: public StatefulActionNode{
private:
    vector<pair<double, double>> points;
    int current_index = 0;
    int staying_time = 4000 ;
    zh_timer staying{6000};

public:

    MoveToPoint_pursuit(const string& name, const NodeConfig& config): StatefulActionNode(name, config)
    {        
        
    }

    ~MoveToPoint_pursuit() override = default;

    static PortsList providedPorts()
    {
        return {
            InputPort<vector<double>>("goal_points", "多个目标点，每两个值表示一个点 (x, y)"),
        };
    }

    NodeStatus onStart() override
    {


        vector<double> goal_points;
        getInput("goal_points", goal_points);
        getInput("staying_time", staying_time);
        staying(staying_time);
        points.clear();

        if(goal_points.empty() || (goal_points[0] == 0 && goal_points[1] == 0))
        {
            cout << "目标点数目为0！" << endl;
            return NodeStatus::SUCCESS;
        }
        if (goal_points.size() % 2 != 0 ) 
        {
            cout << "目标点数目不是偶数！" << endl;
            if(goal_points.size() == 1)
            {
                cout << "目标点数目为1！" << endl;
                points.emplace_back(goal_points[0], goal_points[0]);
            }
            else
            {
                goal_points.pop_back();
            }
        }

        auto target_point = make_shared<ref_Point>(goal_points[0], goal_points[1]);
        if(target_point->is_in(area_manager["禁区_1"]))
        {
            DD.goal_x = 15.76;
            DD.goal_y = 5.99;

        }
        else if(target_point->is_in(area_manager["禁区_2"]))
        {
            DD.goal_x = 13.21;
            DD.goal_y = 9.9;
        }
        else
        {
            DD.goal_x = goal_points[0];
            DD.goal_y = goal_points[1];
        }

        cout << "下达自主决策目标点：(" << DD.goal_x << "," << DD.goal_y << ")" << endl;
        
        if (!DD.is_arrived_point( DD.cur_x, DD.cur_y, DD.goal_x, DD.goal_y))
        {
            DD.posture = 3; // 移动姿态
            mes_puber->set_posture(DD.posture);
            mes_puber->stop_moving();
            mes_puber->pub_decision_point(DD.goal_x, DD.goal_y);
            DD.pub_point.reset();
            DD.pre_x = DD.cur_x;
            DD.pre_y = DD.cur_y;
            staying.reset();
            DD.is_pursuiting = 1;
            return NodeStatus::RUNNING;
        }
        
        else
        {
            cout  << "已到达目标点：(" << DD.goal_x << "," << DD.goal_y << ")附近" << endl;
            cout << "当前位置：(" << DD.cur_x << "," << DD.cur_y << ")" << endl;
            DD.is_pursuiting = 0;
            return NodeStatus::SUCCESS;
        }
    }

    NodeStatus onRunning() override
    {
        // if(DD.can_pathplan_stop) 
        // {
        //     mes_puber.pub_decision_point(DD.cur_x, DD.cur_y);
        //     mes_puber.stop_move(true);
        //     cout << "去目标点：(" << DD.goal_x << "," << DD.goal_y << ")行为终止！" << endl;
        //     return NodeStatus::SUCCESS;
        // } 一代车大yaw控制权分配问题
        if(DD.is_arrive)
        {
            cout << "已到达目标点：(" << DD.goal_x << "," << DD.goal_y << ")附近" << endl;
            cout << "当前位置：(" << DD.cur_x << "," << DD.cur_y << ")" << endl;
            DD.is_pursuiting = 0;
            return NodeStatus::SUCCESS;
        }
        else
        {
            if(DD.pub_point.is_timeout()){mes_puber->pub_decision_point(DD.goal_x, DD.goal_y);DD.pub_point.reset();}
            cout << "正在去往目标点：(" << DD.goal_x << "," << DD.goal_y << ")" << endl;
            cout << "当前位置：(" << DD.cur_x << "," << DD.cur_y << ")" << endl;
            printf("%s %s\n\n", this->name().c_str(), toStr(NodeStatus::RUNNING, true).c_str());
            DD.is_pursuiting = 1;
            return NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        mes_puber->stop_moving();
        mes_puber->pub_decision_point(DD.cur_x, DD.cur_y);
        cout << "去目标点：(" << DD.goal_x << "," << DD.goal_y << ")行为被终止！" << endl;
        DD.is_pursuiting = 0;
    };
};
