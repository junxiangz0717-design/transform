#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "sub_pub/pub.h"

static zh_timer center_timer{2000};

class CheckInCenter : public BT::ConditionNode
{
public:
    CheckInCenter(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts(){ return {}; }

    BT::NodeStatus tick() override
    {
        const bool in_center_now = DD.self_pos->is_in(area_manager["中心区域"]);
        
        if (!in_center_now)
        {
            DD.is_in_center_area = false;
            center_timer.reset();   
            return BT::NodeStatus::SUCCESS;
        }

        // 进入中心区域后开始计时
        if (center_timer.is_timeout())
        {
            DD.is_in_center_area = true;
            std::cout << "哨兵在中心区域" << std::endl;
            if (DD.is_center_area != 1)
            {
                DD.posture = 2;
                mes_puber->set_posture(DD.posture);
            }
            else
            {
                DD.posture = 1;
                mes_puber->set_posture(DD.posture);
            }
        }
        else
        {
            DD.is_in_center_area = false;
        }

        return BT::NodeStatus::SUCCESS;
    }
};