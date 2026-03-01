//
// Created by zh on 24-3-30.
// Updated by tzz on 25-1-15

//todo:启动点集待修改
#pragma once

#include "behaviortree_cpp/condition_node.h"

#include "data/decision_data.h"
#include "sub_pub/pub.h"

using namespace std;
using namespace BT;

class CheckGameStart : public ConditionNode
{
private:

public:
    CheckGameStart(const string &name, const NodeConfig &config) : ConditionNode(name, config) {}

    ~CheckGameStart() override = default;

    static PortsList providedPorts()
    {
        return {OutputPort<bool>("Online", "是否在线模式(正式比赛)"),
                OutputPort<bool>("GameStart", "比赛开始")};
    }

    BT::NodeStatus tick() override
    {
        if(!DD.config_is_skip_gamestart)
        {
            // 是否在线模式判断
            if (DD.game_period != 0)
            {
                setOutput("Online", true);
                cout << "Online!" << endl;
            }
            else
                setOutput("Online", false);
            
            if (DD.game_period == 4)
            {
                cout << "比赛开始！" << endl;
                setOutput("GameStart", true);
                return NodeStatus::SUCCESS;
            }
            if(DD.game_period == 5)
            {
                cout << "比赛结束！等待返回" << endl;
                return NodeStatus::SUCCESS;
            }
            
            cout << "当前比赛阶段：" << game_period_to_str(DD.game_period) << endl;
            cout << "比赛未开始……" << endl;
            return NodeStatus::FAILURE;
        }
        else
        {
            return NodeStatus::SUCCESS;
        }
        
    };

    static string game_period_to_str(int game_period)
    {
        switch (game_period)
        {
        case (0):
            return "0：未开始比赛";
        case (1):
            return "1：准备阶段（检修阶段）";
        case (2):
            return "2：十五秒裁判系统自检阶段";
        case (3):
            return "3：五秒倒计时";
        case (4):
            return "4：比赛中";
        case (5):
            return "5：比赛结算中";
        default:
            return "比赛阶段标志位非法，超出0-5！";
        }
    }
};