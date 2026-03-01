//
// Created by tzz on 24-1-15.
//
# pragma once

#include "behaviortree_cpp/action_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <sub_pub/pub.h>

using namespace BT;
using namespace std;

class SetCheckHpParam: public SyncActionNode
{
public:
  SetCheckHpParam(const string& name, const NodeConfig& config): SyncActionNode(name, config){}
  ~SetCheckHpParam() override = default;

  static PortsList providedPorts()
  {
      return
    {
        InputPort<int>("attack_hp_threshold","战斗时回补给点血量阈值"),
        InputPort<int>("hp_threshold","非战斗时回补给点血量阈值"),
        InputPort<int>("add_to_target_hp","回血上限")
    };
  }

  BT::NodeStatus tick() override
  {
    getInput("attack_hp_threshold" ,attack_hp_thresholds);
    getInput("hp_threshold" ,hp_thresholds);
    getInput("add_to_target_hp", DD.add_to_target_hp);
    if(DD.targets.autoaim_target != 7){DD.hp_thresholds = attack_hp_thresholds;}
    else if(DD.state == State::TO_FORT || DD.is_in_fort || DD.rush){DD.hp_thresholds = 50;}
    else{DD.hp_thresholds = hp_thresholds;} 
    cout << "设置血量下限阈值为：" << DD.hp_thresholds << endl;
    cout << "设置补血行为目标补充到的血量为：" <<  DD.add_to_target_hp << endl;

    DD.posture = 3; // 移动姿态
    mes_puber->set_posture(DD.posture);
    return NodeStatus::SUCCESS;
  }
private:
  int hp_thresholds;
  int attack_hp_thresholds;
};
