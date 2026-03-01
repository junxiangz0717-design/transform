#pragma once
// 未加姿态
#include "behaviortree_cpp/action_node.h"
#include "sub_pub/pub.h"

#include "data/decision_data.h"
#include "data/enum.h"

using namespace std;
using namespace BT;

class Treatment: public StatefulActionNode
{
private:
    static int pre_hp;
public:

    Treatment(const string& name, const NodeConfig& config): StatefulActionNode(name, config){}

    ~Treatment() override = default;

    static PortsList providedPorts()
    {
        return
        {};
    }

    NodeStatus onStart() override
    {    

        if (DD.hp_sentry >= DD.add_to_target_hp)
        {
            cout << "回血完毕！" << endl;
            cout << "当前血量：" << DD.hp_sentry << endl;
            DD.hp_state = 0;
            pre_hp = DD.hp_sentry;
            return NodeStatus::SUCCESS;
        }
        pre_hp = DD.hp_sentry;
        cout << "开始回血" << endl;
        DD.hp_state = 2;
        cout << "当前血量：" << DD.hp_sentry << endl;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {   
        if (DD.hp_sentry >= DD.add_to_target_hp)
        {
            cout << "回血完毕！" << endl;
            cout << "当前血量：" << DD.hp_sentry << endl;
            DD.hp_state = 0;
            pre_hp = DD.hp_sentry;
            return NodeStatus::SUCCESS;
        }
        if(DD.hp_sentry <= pre_hp)
        {
            cout<<"未到达补给点，补血失败"<<endl;
            cout<<"等待导航到达补给点"<<endl;
            DD.hp_state = 3;
            return NodeStatus::FAILURE;
        }

        if (DD.hp_sentry > pre_hp)          // 修改了判断逻辑，使用前后变化来判断
        {
            cout << "正在回血……" << endl;
            cout << "当前血量：" << DD.hp_sentry << endl;
            DD.hp_state = 2;
        }
        pre_hp = DD.hp_sentry;
        return NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        DD.hp_state = 0;
        pre_hp = DD.hp_sentry;
        cout<< "补血行为终止！"<< endl;
    };
};

int Treatment::pre_hp = 400; 