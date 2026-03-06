#pragma once
//二代车底盘云台解耦
//配合scipt使用,调试yaw
// 已加姿态

#include "behaviortree_cpp/action_node.h"
#include "sub_pub/pub.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <cmath>
#include <vector>

using namespace BT;

class Attack : public SyncActionNode
{
private:
    float ccw_yaw=0;//逆时针
    float cw_yaw=0;//顺时针
    Target target;
    bool need_turn_big_yaw = false;
    bool once;
public:
    Attack(const string &name, const NodeConfig &config) : SyncActionNode(name, config) {once=true;}
    ~Attack() override = default;

    static PortsList providedPorts()
    {
        return {InputPort<Target>("have_target","当前目标"),
                InputPort<bool>("need_turn_big_yaw","是否需要转动大yaw,用于调转枪口和感知")
        };
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
        if(!DD.detector_turn.is_timeout())//攻击
        {

            cout<<DD.targets.detector_angle[static_cast<int>(target)]<<endl;
            if(!DD.is_in_block || DD.is_arrive)DD.decision_yaw_az = DD.config_detector_turn_az;
            cout<<"感知转头"<<endl;
        }
        else if(DD.targets.autoaim_target != 7)
        {
            DD.decision_yaw_az = 0;
            DD.targets.decision_target = static_cast<int>(target);
            DD.posture = 1; //攻击姿态
            mes_puber->set_posture(DD.posture);
            if(DD.is_outposting){DD.targets.decision_target = 4;}
            else if(DD.to_rune){DD.targets.decision_target = 10;}
            mes_puber->pub_decision_to_autoaim();
            return NodeStatus::SUCCESS;
        }
        else
        {
            DD.decision_yaw_az = 0;
            DD.targets.decision_target = 7;
            if(DD.is_outposting)
            {
                DD.targets.decision_target = 4;
                DD.posture = 1; //攻击姿态
                mes_puber->set_posture(DD.posture);
            }
            else if(DD.to_rune)
            {
                DD.targets.decision_target = 10;
                DD.posture = 1; //攻击姿态
                mes_puber->set_posture(DD.posture);
            }
            mes_puber->pub_decision_to_autoaim();
            mes_puber->pub_decision_to_autoaim();
            DD.posture = 3; //移动姿态
            mes_puber->set_posture(DD.posture);
            
            cout<<"不攻击"<<endl;
        }
        return NodeStatus::SUCCESS;
    }
};