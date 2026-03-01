/*过狗洞速推前哨*/
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>
#include <vector>
#include <algorithm>

using namespace std;
using namespace BT;

/*一个TotalState节点对应一个具体策略*/
class Node_RADAR : public  SyncActionNode 
{
public:
    Node_RADAR(const string &name, const NodeConfig &config) : SyncActionNode(name, config) 
    {}
    
    ~Node_RADAR() override = default;
    
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
        /*基本行为*/
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
        else if(DD.state == State::TO_FORT)
        {
            subtree = "TO_FORT";
            setOutput("subtree",subtree);
        }
        else if(DD.ammo <= 10 && DD.get_ammo_count <DD.available_ammo)
        {
            subtree ="BACKING";
            DD.state = State::BACKING;
            setOutput("subtree",subtree);
        }
        else/*具体策略在这里修改*/
        {
            if(DD.start_time <= 30)
            {
                if((have_target != Target::NONE || DD.is_pursuiting == 1) && !DD.is_in_block)
                {
                    DD.state = State::ATTACKING;
                    subtree = "PURSUITING";
                    setOutput("subtree",subtree);   
                }
                else
                {
                    subtree = "FIND_CENTER";
                    setOutput("subtree",subtree);   
                }
            }
            else
            {
                if((have_target != Target::NONE || DD.is_pursuiting == 1) && !DD.is_in_block)
                {
                    DD.state = State::ATTACKING;
                    subtree = "PURSUITING";
                    setOutput("subtree",subtree);   
                }
                else
                {
                    DD.state = State::FINDING;
                    if(DD.radar_have == 1 && DD.radar_arrive == 0)
                    {
                        subtree = "FIND_RADAR";
                        setOutput("subtree",subtree);   
                    }
                    else
                    {
                        subtree = "FIND_CENTER";
                        setOutput("subtree",subtree);
                    }
                }
            }
        }
        cout<<"::"<<subtree<<endl;
        return NodeStatus::SUCCESS;
    }
private:
    AMMO ammo_state;
    Target have_target;
    string subtree;
    int ammo_thresholds = 10;
};


