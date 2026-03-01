// 回家
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <vector>
#include <algorithm>
#include "sub_pub/pub.h"
#pragma once

using namespace std;
using namespace BT;

class BackHome : public SyncActionNode
{
public:
    BackHome(const std::string& name, const NodeConfig& config) : SyncActionNode(name, config){}
    ~BackHome()override=default;
    static PortsList providedPorts()
    {
        return {InputPort<string>("subtree","当前子树")};
    }
    NodeStatus tick() override
    {
        getInput("subtree",subtree);
        if (subtree == "BACKING")
        {
            // 移动姿态
            DD.posture = 3;
            mes_puber->set_posture(DD.posture);
            cout<<"回家中"<<endl;
            DD.can_pathplan_stop = false;
            return NodeStatus::SUCCESS;
        }
        DD.can_pathplan_stop = true;
        return NodeStatus::FAILURE;
    }

private:
string subtree;
};