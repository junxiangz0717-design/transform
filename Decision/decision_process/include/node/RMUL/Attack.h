#pragma once
//二代车底盘云台解耦
//配合scipt使用,调试yaw

#include "behaviortree_cpp/action_node.h"
#include "sub_pub/pub.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <cmath>
#include <vector>

using namespace BT;
using namespace std;

class Attack : public SyncActionNode
{
private:
    float ccw_yaw=0;//逆时针
    float cw_yaw=0;//顺时针
    Target target;
    bool need_turn_big_yaw = false;
    bool once;  // 调整用标志位
public:
    Attack(const string &name, const NodeConfig &config) : SyncActionNode(name, config) {once=true;}
    ~Attack() override = default;

    static PortsList providedPorts()
    {
        return {InputPort<Target>("have_target","当前目标"),
                InputPort<bool>("need_turn_big_yaw","是否需要转动大yaw,用于调转枪口和感知")};
    }
    
    NodeStatus tick() override
    {
        getInput("need_turn_big_yaw",need_turn_big_yaw);
        getInput("have_target",target);
        cout<<":::"<<DD.detector_turn.is_timeout()<<endl;
        if(need_turn_big_yaw && once)
        {
            DD.detector_again.reset();
            DD.detector_turn.reset();
            once = false;
        }
        if(DD.detector_again.is_timeout()){once=true;}//重置一次
        if(!DD.detector_turn.is_timeout() && !DD.is_weak)//攻击
        {
            cout<<DD.targets.detector_angle[static_cast<int>(target)]<<endl;
            if(DD.is_arrive) DD.decision_yaw_az = DD.config_detector_turn_az;
            cout<<"感知转头"<<endl;
        }
        else if(!DD.is_weak)
        {
            if(DD.targets.autoaim_target != 7)
            {
                DD.decision_yaw_az = 0;
                DD.targets.decision_target = static_cast<int>(target);
                DD.posture = 1; //攻击姿态
                mes_puber->set_posture(DD.posture);
                mes_puber->pub_decision_to_autoaim();
                return NodeStatus::SUCCESS;
            }
            else
            {
                DD.decision_yaw_az = DD.config_az;
                DD.targets.decision_target = 7;
                DD.posture = 3; //移动姿态
                mes_puber->set_posture(DD.posture);
                mes_puber->pub_decision_to_autoaim();
                cout<<"不攻击"<<endl;
                return NodeStatus::FAILURE;
            }
        }
        else
        {
            DD.decision_yaw_az = 0;
            DD.targets.decision_target = 7;
            DD.posture = 3; //移动姿态
            mes_puber->set_posture(DD.posture);
            mes_puber->pub_decision_to_autoaim();
            cout<<"不攻击"<<endl;
        }
        return NodeStatus::SUCCESS;
    }
};

// inline double Attack::normalize_angle(double angle) 
// {
//     // 将角度标准化到 -180 到 180 之间
//     while (angle > 180.0) {
//         angle -= 360.0;
//     }
//     while (angle < -180.0) {
//         angle += 360.0;
//     }
//         return angle;
// }

// inline double Attack::calculate_min_rotation(double yaw_large, double yaw_small) 
// {
//     // 计算大yaw和小yaw的差值
//     double yaw_diff = yaw_large - yaw_small;

//     // 计算最小旋转角度，标准化到 -180 到 180 之间
//     return normalize_angle(yaw_diff);
// }