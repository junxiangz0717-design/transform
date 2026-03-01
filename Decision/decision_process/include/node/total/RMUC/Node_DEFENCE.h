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
        return {InputPort<AMMO>("ammo_state","当前弹药状态"),
                InputPort<Target>("have_target","目标"),
                OutputPort<string>("subtree","当前子树")};
    }
    
    NodeStatus tick() override
    {
        subtree.clear();
        getInput("ammo_state",ammo_state);
        getInput("have_target",have_target);
        if(DD.start_time >= 350 && DD.game_period == 4)
        {   
            if(DD.ammo>=100){DD.rush = 1;DD.priority =3;}
            DD.ammo_thresholds = 90;
        }
        if(DD.state == State::INSTRUCTION)
        {
            subtree = "INSTRUCTION";
            setOutput("subtree",subtree);
            
        }
        else if(DD.state == State::BACKING)
        {
            subtree = "BACKING";
            setOutput("subtree",subtree);
            
        }
        else if (DD.state == State::TREATING)
        {
            subtree = "TREATING";
            setOutput("subtree",subtree);
            
        }
        else if(ammo_state == AMMO::NO_AMMO)
        {
            subtree = "NO_AMMO";
            setOutput("subtree",subtree);
            
        }
        else if(DD.ammo <= DD.ammo_thresholds && DD.get_ammo_count <DD.available_ammo)
        {
            subtree ="BACKING";
            DD.state = State::BACKING;
            setOutput("subtree",subtree);
        }
        else if(DD.rush == 1 )
        {  
            DD.ammo_thresholds = 1;
            if((have_target == Target::HERO || DD.is_pursuiting == 1) && (!DD.is_in_block || DD.is_arrive))
            {
                {
                    DD.state = State::ATTACKING;
                    subtree = "PURSUITING";
                    setOutput("subtree",subtree);   
                }
            }
            else
            {
                subtree = "FIND_RUSH";
                setOutput("subtree",subtree);
            }

        }
        else
        {
            cout<<":::"<<have_target<<endl;
            cout<<":::"<<DD.is_pursuiting <<endl;
            if((have_target != Target::NONE || DD.is_pursuiting == 1) && (!DD.is_in_block || DD.is_arrive))
            {
                DD.state = State::ATTACKING;
                subtree = "PURSUITING";
                DD.to_rune = 0;
                setOutput("subtree",subtree);   
            }
            else if(DD.start_time <= 30)
            {
                DD.state = State::FINDING;
                subtree="FIND_STANDING";
                setOutput("subtree",subtree);
            }
            else 
            {
                DD.state = State::FINDING;
                subtree="FIND_CENTER";
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
    AMMO ammo_state;
    Target have_target;
    string subtree;
    zh_timer charge_timer{40000}; 
    zh_timer defence_timer{10000}; 
    bool find = true;
};
