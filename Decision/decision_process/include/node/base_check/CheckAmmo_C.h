#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "data/enum.h"

using namespace std;
using namespace BT;

class CheckAmmo_C : public ConditionNode 
{
private:
    zh_timer target_timer{5000};
public:
    CheckAmmo_C(const string &name, const NodeConfig &config) : ConditionNode(name, config) {}
    
    ~CheckAmmo_C() override = default;
    
    static PortsList providedPorts()
    {
        return {OutputPort<AMMO>("ammo_state","当前弹药状态")};
    }
    
    NodeStatus tick() override
    {
        cout<<DD.get_ammo_count<<endl;
        cout<<DD.available_ammo<<endl;
        if(DD.is_in_add_area)
        {
            DD.get_ammo_count = DD.start_time/60;
        }
        if (DD.is_no_ammo)
        {
            cout << "实体弹已耗尽" << endl;
            setOutput("ammo_state",AMMO::NO_AMMO);
            return NodeStatus::SUCCESS;
        }
        if(DD.ammo <= 5)
        {
            if(DD.get_ammo_count < DD.available_ammo)
            {
                cout << "低弹量" << endl;
                setOutput("ammo_state",AMMO::LOW_AMMO);
            }
            else
            {
                cout << "无弹量" << endl;
                setOutput("ammo_state",AMMO::NO_AMMO);
            }
            
        }
        else
        {
            setOutput("ammo_state",AMMO::HIGH_AMMO);
            cout << "高弹量" << endl;
        }
        return NodeStatus::SUCCESS;
    }
};