//
// Created by tzz on 25-1-18.
//
#pragma once


#include "behaviortree_cpp/action_node.h"
#include "data/enum.h"
#include "data/decision_data.h"

// #include "tool/autoaim_to_decision_logger.h"

using namespace BT;
using namespace std;
class SetTarget : public SyncActionNode
{
private:
    int restart_count=0;
public:
    SetTarget(const string &name, const NodeConfig &config) : SyncActionNode(name, config)
    {
        setOutput("need_turn_big_yaw", false);
    }
    
    ~SetTarget() override = default;

    static PortsList providedPorts()
    {
        return
            {
                OutputPort<bool>("need_turn_big_yaw", "是否需要转动大yaw,用于调转枪口和感知"),
                OutputPort<Target>("have_target","目标"),
            };
    }

    NodeStatus tick() override
    {   
        if(DD.time <= 180){DD.target_list[0][2] = 10;DD.target_list[1][2] = 10;DD.target_list[2][2] = 10;}

        cout<<static_cast<int>(DD.targets.autoaim_target)<<endl;
        cout<<static_cast<int>(DD.targets.pre_target)<<endl;

        if (DD.targets.autoaim_target == 7 &&
            DD.targets.pre_target != Target::NONE
            && DD.is_approachable(DD.targets.pre_target)&&!DD.autoaim_dropped_frames.is_timeout())//目标消失
        {
            cout << "自瞄目标丢帧或消失，且目标未阵亡，保持追击目标状态" << endl;
            cout<<"target:"<<DD.targets.autoaim_target<<endl;
            setOutput("need_turn_big_yaw", false);
            return NodeStatus::SUCCESS;
        }
        if (DD.targets.autoaim_target != 7)
        {
            DD.targets.pre_target = static_cast<Target>(DD.targets.autoaim_target);
        }
        DD.autoaim_dropped_frames.reset();
        if (DD.targets.targets_vector.empty()&&DD.targets.detector_targets_vector.empty())
        {
            cout << "自瞄、感知相机均无目标" << endl;
            setOutput("have_target", Target::NONE);
            DD.targets.pre_target = static_cast<Target>(7);
            setOutput("need_turn_big_yaw", false);
            return NodeStatus::SUCCESS;
            
        }
        else if(DD.targets.targets_vector.empty() && !DD.targets.detector_targets_vector.empty())
        {
            cout << "自瞄无目标，感知相机有目标" << endl;
            std::pair<Target, double> best_detector_target = BestTarget(DD.targets.detector_targets_vector,DD.targets.detector_distance_vector,DD.pitch_mode);
            setOutput("have_target", best_detector_target.first);
            setOutput("need_turn_big_yaw", true);
            cout<<Target_to_str(best_detector_target.first)<<endl;
            return NodeStatus::SUCCESS;
        }
        else if(!DD.targets.targets_vector.empty() && DD.targets.detector_targets_vector.empty())
        {
            cout << "自瞄有目标，感知相机无目标" << endl;
            std::pair<Target, double> best_autoaim_target = BestTarget(DD.targets.targets_vector,DD.targets.targets_distance_vector,DD.pitch_mode);
            setOutput("have_target", best_autoaim_target.first);
            setOutput("need_turn_big_yaw", false);
            cout<<Target_to_str(best_autoaim_target.first)<<endl;
            return NodeStatus::SUCCESS;
        }
        else
        {
            cout << "自瞄、感知相机均有目标" << endl;
            std::pair<Target, double> best_autoaim_target  = BestTarget(DD.targets.targets_vector,DD.targets.targets_distance_vector,DD.pitch_mode);
            std::pair<Target, double> best_detector_target = BestTarget(DD.targets.detector_targets_vector,DD.targets.detector_distance_vector,DD.pitch_mode);
            if (best_autoaim_target.second > best_detector_target.second)
            {
                setOutput("have_target", best_autoaim_target.first);
                setOutput("need_turn_big_yaw", false);
                cout<<Target_to_str(best_autoaim_target.first)<<endl;
            }
            else
            {
                setOutput("have_target", best_detector_target.first);
                setOutput("need_turn_big_yaw", true);           
                cout<<Target_to_str(best_detector_target.first)<<endl;
            }
            
            return NodeStatus::SUCCESS;
        }
    }

    std::pair<Target, double> BestTarget(const vector<Target> &targets_vector, const vector<double> &targets_distance_vector, int if_detec)
    {
        double max_score = 0;
        Target best_target = Target::NONE;
        if(if_detec != 1) { return {best_target, max_score}; }
        for (const auto& target :targets_vector) 
        {
            double score = 0;
            int target_index = static_cast<int>(target);
            // RMUL只有 HERO=0, FOOT_3=2, SENTRY=6, 跳过无效索引防止越界
            if(target_index < 0 || target_index >= static_cast<int>(DD.enemy_hp.size())){continue;}
            if((DD.target_list[DD.priority][target_index] ==0)||(target_index != 0 && target_index != 2 && target_index != 6)){continue;}
            double distance  = targets_distance_vector[target_index];
            int hp = DD.enemy_hp[target_index];
            if(distance >= 7.0)continue;
            else if(distance < 7.0 && distance >= 5.5)score += 300;
            else 
            {
                score += 600;
                score += distance;
            }
            if(hp < 120){score += 1000;}
            score += DD.target_list[DD.priority][target_index];
            if (score > max_score) 
            {
                max_score = score;
                best_target = target;
            }
        }
        return {best_target, max_score};
    }
};
    

