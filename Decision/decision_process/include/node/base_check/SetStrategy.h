//
// Created by zh on 24-4-2.
//
#pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <string>
#include <vector>

using namespace BT;
using namespace std;

class SetStrategy: public SyncActionNode
{
private:
    int once =1;
    string strategy;
    vector<string>strategyDescriptions= {
    "++++下对面隧道_追击++++",
    "++++++守家++++++",
    "++++全程中央高地++++",
    "++++占对面u弯++++",
    "+++打前哨+++"};

public:
    SetStrategy(const string& name, const NodeConfig& config): SyncActionNode(name, config)
    {};
    ~SetStrategy() override = default;
  
    static PortsList providedPorts()
    {
        return
        {OutputPort<string>("choose_strategy","输出策略代号")};
    }
    BT::NodeStatus tick() override
    {

        int strategyNum = DD.strategy;
        if (strategyNum < 0 || strategyNum > 4) {
            strategyNum = 0; // 默认使用激进策略
        }
        cout<<"策略编号："<<strategyNum<<endl;
        // 转换为字符串并输出
        strategy = to_string(strategyNum);
        setOutput("choose_strategy", strategy);

        // 输出策略描述
        cout << strategyDescriptions[strategyNum] <<endl;
        return NodeStatus::SUCCESS;
        
    };
};