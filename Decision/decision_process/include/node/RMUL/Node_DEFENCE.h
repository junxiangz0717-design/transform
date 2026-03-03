#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>

using namespace std;
using namespace BT;

/*一个TotalState节点对应一个具体策略*/
class Node_DEFENCE : public  SyncActionNode 
{
public:
    Node_DEFENCE(const string &name, const NodeConfig &config) : SyncActionNode(name, config) 
    {charge_timer.reset();defence_timer.reset();}
    
    ~Node_DEFENCE() override = default;
    
    static PortsList providedPorts()
    {
        return {InputPort<Target>("have_target","目标"),
                OutputPort<string>("subtree","当前子树")};
    }
    
    NodeStatus tick() override
    {
        subtree.clear();
        getInput("have_target",have_target);

        DD.posture = 3; // 移动姿态
        mes_puber->set_posture(DD.posture);

        if (DD.start_time >= 150 && DD.game_period == 4)
        {
            DD.rush = 1;
        }
        
        if(DD.state == State::BACKING)
        {
            subtree = "BACKING";
            setOutput("subtree",subtree);
        }
        else if (DD.state == State::TREATING)
        {
            subtree = "TREATING";
            setOutput("subtree",subtree);
        }
        else if(DD.rush == 1 && ( DD.is_center_area == 2 || DD.is_center_area == 3))
        {  
            if((have_target == Target::HERO || DD.is_pursuiting == 1) && DD.is_arrive)
            {
                {
                    DD.state = State::ATTACKING;
                    subtree = "PURSUITING";
                    setOutput("subtree",subtree);   
                }
            }
            else
            {
                subtree = "TO_CENTER";
                setOutput("subtree",subtree);
            }
        }
        else
        {
            cout<<":::"<<have_target<<endl;
            cout<<":::"<<DD.is_pursuiting <<endl;
            if((have_target != Target::NONE || DD.is_pursuiting == 1) && !DD.is_arrive)
            {
                DD.state = State::ATTACKING;
                subtree = "PURSUITING";
                setOutput("subtree",subtree);   
            }
            else 
            {
                subtree="TO_CENTER";
                setOutput("subtree",subtree);
            }
        }
        cout<<"::"<<find<<endl;
        cout<<subtree<<endl;
        if(DD.state != State::FINDING){charge_timer.reset();defence_timer.reset();}
        if(find)
        {
            if(charge_timer.is_timeout()){find = !find;defence_timer.reset();}
        }
        else
        {
            if(defence_timer.is_timeout()){find = !find;charge_timer.reset();}
        }
        return NodeStatus::SUCCESS;
    }
private:
    Target have_target;
    string subtree;
    zh_timer charge_timer{40000}; 
    zh_timer defence_timer{10000}; 
    bool find = true;
};
