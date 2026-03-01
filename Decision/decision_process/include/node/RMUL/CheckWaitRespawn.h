//creat


//RMUL版本 没有买活
//todo 发送点位
#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include "sub_pub/pub.h"

#include <iostream>
#include <chrono>

using namespace std;
using namespace BT;

class CheckWaitRespawn : public ConditionNode 
{
private:
    bool respawn = false;
public:
    CheckWaitRespawn(const string &name, const NodeConfig &config) : ConditionNode(name, config) {}
    ~CheckWaitRespawn() override = default;
    
    static PortsList providedPorts() {return{ };};
    
    NodeStatus tick() override
    {
        if(!DD.config_is_skip_gamestart)
        {
            if(DD.hp_sentry != 0&& DD.game_period==4)
            {
                cout<<"未阵亡"<<endl;
                respawn = false;
                pub_respawn(respawn);//重置标志位
                return NodeStatus::SUCCESS;
            }
            else
            {
                respawn = true;
                DD.is_weak = 1;
                pub_respawn(respawn);
                DD.state = State::BACKING;
                DD.posture = 3;
                mes_puber->set_posture(DD.posture);
                return NodeStatus::FAILURE;
            }
        }
        else
        {
            return NodeStatus::SUCCESS;
        }
    }

    static inline void pub_respawn(bool if_respawn) 
    {
        if(if_respawn)
        {
            cout<<"发送复活标志"<<endl;
            mes_puber->confirm_free_respawn(if_respawn);//发送确认复活
        }
        else
        {
            cout<<"不发送复活标志"<<endl;
            mes_puber->confirm_free_respawn(if_respawn);
        }
        
    }

};
