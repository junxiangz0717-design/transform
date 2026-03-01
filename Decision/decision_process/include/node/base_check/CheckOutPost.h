#pragma once
#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>
using namespace std;
using namespace BT;

class CheckOutPost : public ConditionNode 
{
private:
    double goal_x;
    double goal_y;
    bool lock =false;
public:
    CheckOutPost(const string &name, const NodeConfig &config) : ConditionNode(name, config) 
    {
        goal_x = 3.0;
        goal_y = 2.0;
    }
    
    ~CheckOutPost() override = default;
    
    static PortsList providedPorts(){return { };}
    
    NodeStatus tick() override
    {
        if(DD.self_pos->is_in(area_manager["前哨击打区"]))
        {
            DD.pitch_mode = 1;
            DD.is_outposting = true;
        }
        else
        {
            DD.pitch_mode = 0;
            DD.is_outposting = false;
        }
        cout<<"Pitch_mode:"<<DD.pitch_mode<<endl;
        if(DD.pitch_mode != 1){return NodeStatus::FAILURE;}
        else
        {
            if(DD.targets.autoaim_target == 6 || !DD.outpost.is_timeout())
            {   DD.outpost.reset();
                return NodeStatus::SUCCESS;
            }
            else
            {
                DD.decision_yaw_az = -0.9;
                return NodeStatus::FAILURE;
            }
        }
    }
};