// 自适应距离追击
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>
#include "sub_pub/pub.h"

using namespace std;
using namespace BT;

class AdaptiveChoose : public SyncActionNode
{
public:
    AdaptiveChoose(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~AdaptiveChoose()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "PURSUITING")
        {
            // 移动姿态
            DD.posture = 3;
            mes_puber->set_posture(DD.posture);
            cout<<"自适应"<<endl;
            return NodeStatus::SUCCESS;
        }
        DD.is_pursuiting = 0;
        return NodeStatus::FAILURE;
    }
private:
string subtree;
};
