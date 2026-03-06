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
class SetTarget_QQ : public SyncActionNode
{
private:
    int restart_count=0;
public:
    SetTarget_QQ(const string &name, const NodeConfig &config) : SyncActionNode(name, config)
    {
        setOutput("need_turn_big_yaw", false);
    }
    
    ~SetTarget_QQ() override = default;

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

        cout<<"========== SetTarget Debug Info =========="<<endl;
        cout<<"autoaim_target: "<<static_cast<int>(DD.targets.autoaim_target)<<endl;
        cout<<"pre_target: "<<static_cast<int>(DD.targets.pre_target)<<endl;
        cout<<"priority: "<<DD.priority<<"  time: "<<DD.time<<"  pitch_mode: "<<DD.pitch_mode<<endl;

        // 打印 targets_vector (自瞄目标)
        cout<<"targets_vector["<<DD.targets.targets_vector.size()<<"]: ";
        for(size_t i=0;i<DD.targets.targets_vector.size();i++)
            cout<<static_cast<int>(DD.targets.targets_vector[i])<<" ";
        cout<<endl;

        // 打印 targets_distance_vector (自瞄距离)
        cout<<"targets_distance_vector["<<DD.targets.targets_distance_vector.size()<<"]: ";
        for(size_t i=0;i<DD.targets.targets_distance_vector.size();i++)
            cout<<DD.targets.targets_distance_vector[i]<<" ";
        cout<<endl;

        // 打印 detector_targets_vector (感知目标)
        cout<<"detector_targets_vector["<<DD.targets.detector_targets_vector.size()<<"]: ";
        for(size_t i=0;i<DD.targets.detector_targets_vector.size();i++)
            cout<<static_cast<int>(DD.targets.detector_targets_vector[i])<<" ";
        cout<<endl;

        // 打印 detector_distance_vector (感知距离)
        cout<<"detector_distance_vector["<<DD.targets.detector_distance_vector.size()<<"]: ";
        for(size_t i=0;i<DD.targets.detector_distance_vector.size();i++)
            cout<<DD.targets.detector_distance_vector[i]<<" ";
        cout<<endl;

        // 打印 target_list (优先级表)
        for(size_t i=0;i<DD.target_list.size();i++){
            cout<<"target_list["<<i<<"]: ";
            for(size_t j=0;j<DD.target_list[i].size();j++)
                cout<<DD.target_list[i][j]<<" ";
            cout<<endl;
        }

       

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
        cout<<"--- BestTarget 评分过程 (if_detec="<<if_detec<<") ---"<<endl;
        for (const auto& target :targets_vector) 
        {
            double score = 0;
            int target_index = static_cast<int>(target);
            cout<<"  target_index="<<target_index;
            // RMUL只有 HERO=0, FOOT_3=2, SENTRY=6, 跳过无效索引防止越界
            if(target_index < 0 || target_index >= static_cast<int>(DD.enemy_hp.size())){continue;}
            if((DD.target_list[DD.priority][target_index] ==0)||(target_index != 0 && target_index != 2 && target_index != 6)){continue;}
            double distance  = targets_distance_vector[target_index];
            if(distance >= 5.0)continue;
            else if(distance < 5.0 && distance >= 2.5)score += 300;
            else 
            {
                score += 600;
                score += distance;
            }
            score += DD.target_list[DD.priority][target_index];
            cout<<" -> score="<<score<<endl;
            if (score > max_score) 
            {
                max_score = score;
                best_target = target;
            }
        }
        cout<<"  best_target="<<static_cast<int>(best_target)<<" max_score="<<max_score<<endl;
        cout<<"--- BestTarget 评分结束 ---"<<endl;
        return {best_target, max_score};
    }
};
    

