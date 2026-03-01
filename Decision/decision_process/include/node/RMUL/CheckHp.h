// 新增了回家判断虚弱状态是否解除
#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "data/enum.h"

using namespace std;
using namespace BT;


class CheckHp : public ConditionNode {
public:
    CheckHp(const string &name, const NodeConfig &config) : ConditionNode(name, config) {}
    
    ~CheckHp() override = default;
    
    static PortsList providedPorts(){return { };}
    
    NodeStatus tick() override
    {
        if(DD.state == State::BACKING || DD.hp_state ==3 || DD.is_weak == 1)
        {
            DD.hp_state = 3;
            cout << "前往补给点中"<< endl;
            cout <<"血量："<<DD.hp_sentry<<endl;
            cout << "虚弱状态:" << DD.is_weak << endl;
        }
        if (DD.is_in_add_area && DD.hp_sentry !=400 )
        {
            DD.hp_state = 2;
            cout << "回血中：" << DD.hp_sentry << endl;
            DD.state = State::TREATING;
        }
        else
        {
            checkHpState(DD.hp_sentry, DD.hp_thresholds, DD.is_weak);
        }
        return NodeStatus::SUCCESS;
    }

private:
    int pre_hp =400;
    void checkHpState( int& hp_sentry, int& hp_thresholds, int& weak)
    {
        if (hp_sentry >= hp_thresholds && weak == 0)
        {
            cout << "血量健康：" << hp_sentry << endl;
            cout << "虚弱状态：" << weak << endl;
            DD.hp_state = 0;
            DD.state = State::FINDING;
        }
        else
        {
            cout << "虚弱状态：" << weak << endl;
            cout << "血量不健康：" << hp_sentry << endl;
            DD.hp_state = 1;
            DD.state = State::BACKING;
        }
    }
};