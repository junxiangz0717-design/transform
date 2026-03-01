#pragma once

#include "behaviortree_cpp/action_node.h"
#include "sub_pub/pub.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include "tool/time/zh_timer.h"
#include <vector>
#include <cstdlib>  



using namespace std;
using namespace BT;

class MoveToPoint: public StatefulActionNode{
private:
    vector<pair<double, double>> points;
    int current_index = 0;
    int staying_time = 4000 ;
    zh_timer staying{6000};

public:

    MoveToPoint(const string& name, const NodeConfig& config): StatefulActionNode(name, config){}

    ~MoveToPoint() override = default;

    static PortsList providedPorts()
    {
        return {InputPort<vector<double>>("goal_points", "多个目标点，每两个值表示一个点 (x, y)"),
                InputPort<int>("staying_time","移动过程停留原地的超时时间")};
    }

    NodeStatus onStart() override
    {
        vector<double> goal_points;
        getInput("goal_points", goal_points);
        getInput("staying_time", staying_time);
        staying(staying_time);
        points.clear();
        if(goal_points.empty())
        {
            cout << "目标点数目为0！" << endl;
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
            
        for (size_t i = 0; i < goal_points.size(); i += 2) {
            points.emplace_back(goal_points[i], goal_points[i + 1]);
        }
        DD.goal_x = points[current_index].first;
        DD.goal_y = points[current_index].second;
        current_index = (current_index + 1) % points.size();

        cout << "下达自主决策目标点：(" << DD.goal_x << "," << DD.goal_y << ")" << endl;
        
        if (!DD.is_arrived_point( DD.cur_x, DD.cur_y, DD.goal_x, DD.goal_y))
        {
            mes_puber->stop_moving();
            mes_puber->pub_decision_point(DD.goal_x, DD.goal_y);
            DD.pub_point.reset();
            DD.pre_x = DD.cur_x;
            DD.pre_y = DD.cur_y;
            staying.reset();
            return NodeStatus::RUNNING;
        }
        
        else
        {
            cout  << "已到达目标点：(" << DD.goal_x << "," << DD.goal_y << ")附近" << endl;
            cout << "当前位置：(" << DD.cur_x << "," << DD.cur_y << ")" << endl;
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
            return NodeStatus::SUCCESS;
        }
        else
        {
            // 移动姿态
            DD.posture = 3;
            mes_puber->set_posture(DD.posture);
            if(DD.pub_point.is_timeout()){mes_puber->pub_decision_point(DD.goal_x, DD.goal_y);DD.pub_point.reset();}
            cout << "正在去往目标点：(" << DD.goal_x << "," << DD.goal_y << ")" << endl;
            cout << "当前位置：(" << DD.cur_x << "," << DD.cur_y << ")" << endl;
            printf("%s %s\n\n", this->name().c_str(), toStr(NodeStatus::RUNNING, true).c_str());
            return NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        mes_puber->stop_moving();
        mes_puber->pub_decision_point(DD.cur_x, DD.cur_y);
        cout << "去目标点：(" << DD.goal_x << "," << DD.goal_y << ")行为被终止！" << endl;
    };
};
