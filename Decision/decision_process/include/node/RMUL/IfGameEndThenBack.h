//
// Created by zh on 24-4-3.
//
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "sub_pub/pub.h"

using namespace BT;
using namespace std;

class IfGameEndThenBack: public SyncActionNode
{
private:
    bool game_end = false;
    bool is_init  = true;//回出发点
    double home_x;
    double home_y;
public:
    IfGameEndThenBack(const string& name, const NodeConfig& config): SyncActionNode(name, config){}
    ~IfGameEndThenBack() override = default;

    static PortsList providedPorts()
    {
        return
        {
            OutputPort<bool>("GameStart", "比赛开始标志位"),
            InputPort<double>("home_x", "哨兵起点x坐标"),
            InputPort<double>("home_y", "哨兵起点y坐标")
        };
    }

    NodeStatus tick() override
    {
        if(!DD.config_is_skip_gamestart)
        {
            getInput("home_x", home_x);
            getInput("home_y", home_y);

            // 在比赛阶段不为4：比赛中，且哨兵处于视觉接管状态，且定位模块数据有效时认为比赛结束（后者是为了确保前者的可信）
            if (game_end)
            {
                setOutput("GameStart", false);
                game_end = false;
                
                cout << "进入下一局比赛阶段！" << endl;
                return NodeStatus::FAILURE;
            }
            if (DD.game_period != 4 /*&&  DD.is_mapping == 1*/)
            {
                cout << "比赛阶段从[4:比赛中]转换为[" << game_period_to_str(DD.game_period) << "]" << endl;
                cout << "比赛结束！" << endl;
                DD.tripod_spin=Tripod_Spin::PI;
                if (is_init)
                {
                    mes_puber->pub_decision_point(home_x, home_y);
                    is_init=false;
                }

                cout << "自主决策目标点：哨兵启动区(" << home_x << "," << home_y << ")" << endl;
                game_end = true;
                if(DD.is_arrive)//这里的1只是调试用，实际上需要更换为DD.is_arrive
                {
                    cout<<"比赛结束，重启程序"<<endl;
                    //rqt测试通过，但是debug模式下重启节点后需要强制关闭，建议debug时注释下面一行
                    //system("rosnode kill decision_process; echo 'y' | rosnode cleanup; rosrun decision_process decision_process");
                }
                
                if(DD.game_period==5)
                return NodeStatus::FAILURE;
            }
            return NodeStatus::SUCCESS;
        }
        else
        {
            return NodeStatus::SUCCESS;
        }
            
    }
    
    static string game_period_to_str(int game_period)
    {
        switch (game_period)
        {
            case(0):
                return "0：未开始比赛";
            case(1):
                return "1：准备阶段（检修阶段）";
            case(2):
                return "2：十五秒裁判系统自检阶段";
            case(3):
                return "3：五秒倒计时";
            case(4):
                return "4：比赛中";
            case(5):
                return "5：比赛结算中";
            default:
                return "比赛阶段标志位非法，超出0-5范围！";
        }
    }
};


