#pragma once
#include "behaviortree_cpp/action_node.h"
#include "sub_pub/pub.h"
#include "data/enum.h"
#include "semantic_map/include/semantic_map.h"
using namespace BT;

class ModeBaseTripod : public SyncActionNode
{
private:
    int mode{};
    int tripod{};
public:
    ModeBaseTripod(const string &name, const NodeConfig &config) : SyncActionNode(name, config) {}
    ~ModeBaseTripod() override = default;

    static PortsList providedPorts(){ return {}; }

    NodeStatus tick() override
    {
        if(DD.game_period != 4 && !DD.config_is_skip_gamestart)
        {
            mes_puber->set_base_tripod_spin(0);//0关
            return NodeStatus::SUCCESS;
        }
        else if(DD.in_hit == 0){mes_puber->set_base_tripod_spin(2);}//2慢速
    
        else
        {   
            if (DD.is_in_center_area)
            {
                DD.posture = 2;
                mes_puber->set_posture(DD.posture);
            }
             
            mes_puber->set_base_tripod_spin(1);
        }//1快速
        return NodeStatus::SUCCESS;
    }
};
