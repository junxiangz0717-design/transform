#pragma once

#include "behaviortree_cpp/condition_node.h"
#include "data/decision_data.h"
#include "data/enum.h"
#include <algorithm>
#include <unordered_map>
using namespace std;
using namespace BT;


class ScanToArea : public ConditionNode 
{
private:
    bool lock = false;
    double angle = 0;
    double max_angle = 0;
    double min_angle = 0;
    Target target;
    std::unordered_map<std::string, std::string> areaMapping = {
        {"己方公路", "己方梯高三角"},
        {"敌方公路", "敌方梯高三角"},
        {"己方梯高", "己方公路三角"},
        {"敌方梯高", "敌方公路三角"}
    };

public:
    ScanToArea(const string &name, const NodeConfig &config) : ConditionNode(name, config) {}
    
    ~ScanToArea() override = default;
    
    static PortsList providedPorts()
    {
        return {OutputPort<int>("tirpod_spin","云台自旋模式"),
                InputPort<Target>("have_target")};
    }   
    
    NodeStatus tick() override
    {
        getInput("have_target",target);
        
        if(target != Target::NONE){ lock = false;return NodeStatus::SUCCESS;}
        if(!DD.config_is_scan)
        {
            if(DD.is_in_block){ DD.decision_yaw_az = -0.01;return NodeStatus::SUCCESS;}
            DD.decision_yaw_az = DD.config_az;return NodeStatus::SUCCESS;
        }
        angle = DD.normalize_angle(DD.big_yaw);
        for (const auto& pair : areaMapping) 
        {
            if (DD.self_pos->is_in(area_manager[pair.first])) 
            {
                setOutput("tirpod_spin", 0);
                max_angle = DD.self_pos->CCW_sector_to(area_manager[pair.second]).first;
                min_angle = DD.self_pos->CCW_sector_to(area_manager[pair.second]).second;
                if (angle > min_angle && angle < max_angle && !lock) 
                {
                    lock = true;
                    DD.decision_yaw_az = DD.config_az;
                } 
                else if (angle > max_angle + DD.config_max_scan_offset_yaw) {DD.decision_yaw_az = -DD.config_az;} 
                else if (angle < min_angle + DD.config_min_scan_offset_yaw) {DD.decision_yaw_az = DD.config_az;}
                return BT::NodeStatus::SUCCESS;
            }
        }
        {
            max_angle = 1.9;
            min_angle = -1.9;
            if(angle > min_angle && angle < max_angle && !lock){ lock = true; DD.decision_yaw_az = DD.config_az;}
            else if(angle > max_angle){DD.decision_yaw_az = -DD.config_az;}
            else if(angle < min_angle){DD.decision_yaw_az = DD.config_az;}
        }
        return NodeStatus::SUCCESS;
    }
};