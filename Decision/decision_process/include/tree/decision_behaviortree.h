#pragma once

#include <thread>
#include <atomic>

//自定义
#include "data/enum.h"
#include "data/decision_data.h"
#include "sub_pub/sub.h"
#include "path/decision_package_path.h"
#include "semantic_map/include/semantic_map.h"
//BT
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
//工具
#include "tool/time/time_measure.h"
#include "tool/logger/logger.h"
#include "tool/time/timer_update.h"
#include "tool/logger/BTlogger.h"

// qingqing
#include "node/qingqing/Adaptive.h"
#include "node/qingqing/SetTarget.h"

// RMUL
#include "node/RMUL/Adaptive.h"
#include "node/RMUL/AdaptiveChoose.h"
#include "node/RMUL/Attack.h"
#include "node/RMUL/BackHome.h"
#include "node/RMUL/CheckCondition.h"
#include "node/RMUL/CheckGameStart.h"
#include "node/RMUL/CheckHp.h"
#include "node/RMUL/CheckInCenter.h"
#include "node/RMUL/CheckHit.h"
#include "node/RMUL/CheckWaitRespawn.h"
#include "node/RMUL/CoverIntFromString.h"
#include "node/RMUL/Finding.h"
#include "node/RMUL/IfGameEndThenBack.h"
#include "node/RMUL/ModeBaseTripod.h"
#include "node/RMUL/MoveToPoint.h"
#include "node/RMUL/MoveToPoint_pursuit.h"
#include "node/RMUL/Node_DEFENCE.h"
#include "node/RMUL/Node_RADICAL.h"
#include "node/RMUL/SetCheckHpParam.h"
#include "node/RMUL/SetStrategy_UL.h"
#include "node/RMUL/SetTarget.h"
#include "node/RMUL/ScanToArea.h"
#include "node/RMUL/TimerOut.h"
#include "node/RMUL/Treat.h"
#include "node/RMUL/ToCenter.h"
#include "node/RMUL/Treatment.h"

//节点
// #include "node/base_check/CheckAmmo_C.h"
// #include "node/base_check/CheckHp.h"
// #include "node/base_check/CheckRadar.h"
// #include "node/base_check/CheckGameStart.h"
// #include "node/base_check/CheckWaitRespawn.h"
// #include "node/base_check/SetCheckHpParam.h"
// #include "node/base_check/SetTarget.h"
// #include "node/base_check/SetStrategy.h"
// #include "node/base_check/CheckFort.h"
// #include "node/base_check/CheckHit.h"
// #include "node/base_check/CheckRune.h"
// #include "node/base_check/IfGameEndThenBack.h"
// #include "node/base_check/SetStyle.h"
// #include "node/base_check/ScanToArea.h"
// #include "node/base_check/CheckInstruction.h"
// #include "node/base_check/CheckOutPost.h"
// #include "node/base_check/SetStrategy_qingqing.h"
// #include "node/base_check/SetStrategy_huaguang.h"
// #include "node/base_check/SetStrategy_Southern.h"

// #include "node/action/Attack.h"
// #include "node/action/ModeBaseTripod.h"
// #include "node/action/MoveToPoint.h"
// #include "node/action/MoveToPoint_once.h"
// #include "node/action/MoveToPoint_radar.h"
// #include "node/action/MoveToPoint_pursuit.h"
// #include "node/action/Treatment.h"
// #include "node/action/TimerOut.h"
// #include "node/action/Adaptive.h"
// #include "node/action/GetOutForbidden.h"

// #include "node/control/CheckCondition.h"
// #include "node/control/CoverIntFromString.h"
#include "node/control/MultiAsyncReactiveSequence.h"

// #include "node/subtree_control/BackHome.h"
// #include "node/subtree_control/Finding.h"
// #include "node/subtree_control/InFort.h"
// #include "node/subtree_control/ToFort.h"
// #include "node/subtree_control/LowAmmo.h"
// #include "node/subtree_control/NoAmmo.h"
// #include "node/subtree_control/Treat.h"
// #include "node/subtree_control/AdaptiveChoose.h"
// #include "node/subtree_control/Instruction.h"
// #include "node/subtree_control/BlockInSlope.h"
// #include "node/subtree_control/AttackOutPost.h"
// #include "node/subtree_control/AttackRune.h"
// #include "node/subtree_control/out_forbidden.h"

// #include "node/total/qingqing/TotalState_home.h"
// #include "node/total/qingqing/TotalState_enemy_home.h"
// #include "node/total/qingqing/TotalState_outpost.h"
// #include "node/total/qingqing/TotalState_center.h"

// #include "node/total/RMUC/Node_CHARGE.h"
// #include "node/total/RMUC/Node_CHARGE_OUTPOST.h"
// #include "node/total/RMUC/Node_CHARGE_1.h"
// #include "node/total/RMUC/Node_CHARGE_2.h"
// #include "node/total/RMUC/Node_DEFENCE.h"
// #include "node/total/RMUC/Node_RADAR.h"


using namespace std;
using namespace BT;

class decision_behaviortree
{
private:
    int delay_milsec_;
    thread tree_thread_;
    Tree tree_;
    std::atomic<bool> stop_thread_{false};

public:
    explicit decision_behaviortree(int delay_milsec);

    ~decision_behaviortree();
};


decision_behaviortree::decision_behaviortree(int delay_milsec):delay_milsec_(delay_milsec)
{
    tree_thread_ = thread(
        [this]()
        {
            try
            {
                BehaviorTreeFactory factory;

                // 注册自定义枚举类型
                factory.registerScriptingEnums<Target>();
                factory.registerScriptingEnums<AMMO>();
                factory.registerScriptingEnums<State>();
                factory.registerScriptingEnums<Style>();
                factory.registerScriptingEnums<Tripod_Spin>();

                // factory.registerNodeType<CheckAmmo_C>("CheckAmmo_C");
                factory.registerNodeType<CheckHp>("CheckHp");
                // factory.registerNodeType<CheckRadar>("CheckRadar");
                factory.registerNodeType<CheckGameStart>("CheckGameStart");
                factory.registerNodeType<CheckWaitRespawn>("CheckWaitRespawn");
                factory.registerNodeType<SetCheckHpParam>("SetCheckHpParam");
                factory.registerNodeType<SetTarget>("SetTarget");
                factory.registerNodeType<SetTarget_QQ>("SetStrategy_QQ");
                factory.registerNodeType<SetStrategy>("SetStrategy_UL");
                // factory.registerNodeType<SetStrategy>("SetStrategy");
                // factory.registerNodeType<SetStyle>("SetStyle");
                factory.registerNodeType<ScanToArea>("ScanToArea");
                // factory.registerNodeType<SetStrategy_huaguang>("SetStrategy_huaguang");
                // factory.registerNodeType<SetStrategy_qingqing>("SetStrategy_qingqing");
                // factory.registerNodeType<SetStrategy_Southern>("SetStrategy_Southern");
                // factory.registerNodeType<CheckFort>("CheckFort");
                factory.registerNodeType<CheckHit>("CheckHit");
                // factory.registerNodeType<CheckRune>("CheckRune");
                factory.registerNodeType<IfGameEndThenBack>("IfGameEndThenBack");
                factory.registerNodeType<Attack>("Attack");
                // factory.registerNodeType<GetOutForbidden>("GetOutForbidden");
                factory.registerNodeType<ModeBaseTripod>("ModeBaseTripod");
                factory.registerNodeType<MoveToPoint>("MoveToPoint");
                // factory.registerNodeType<MoveToPoint_once>("MoveToPoint_once");
                factory.registerNodeType<MoveToPoint_pursuit>("MoveToPoint_pursuit");
                // factory.registerNodeType<MoveToPoint_radar>("MoveToPoint_radar");
                factory.registerNodeType<Treatment>("Treatment");
                factory.registerNodeType<CheckCondition>("CheckCondition");
                factory.registerNodeType<CoverIntFromString>("CoverIntFromString");
                factory.registerNodeType<MultiAsyncReactiveSequence>("MultiAsyncReactiveSequence");
                factory.registerNodeType<BackHome>("BackHome");
                factory.registerNodeType<Finding>("Finding");
                // factory.registerNodeType<InFort>("InFort");
                // factory.registerNodeType<ToFort>("ToFort");
                // factory.registerNodeType<LowAmmo>("LowAmmo");
                // factory.registerNodeType<NoAmmo>("NoAmmo");
                factory.registerNodeType<Treat>("Treat");
                factory.registerNodeType<TimerOut>("TimerOut");
                factory.registerNodeType<Adaptive>("Adaptive");
                factory.registerNodeType<Adaptive_QQ>("Adaptive_QQ");
                factory.registerNodeType<AdaptiveChoose>("AdaptiveChoose");
                factory.registerNodeType<ToCenter>("ToCenter");
                factory.registerNodeType<CheckInCenter>("CheckInCenter");
                // factory.registerNodeType<Instruction>("Instruction");
                // factory.registerNodeType<CheckInstruction>("CheckInstruction");
                // factory.registerNodeType<BlockInSlope>("BlockInSlope");
                // factory.registerNodeType<AttackOutPost>("AttackOutPost");
                // factory.registerNodeType<AttackRune>("AttackRune");
                // factory.registerNodeType<CheckOutPost>("CheckOutPost");
                // factory.registerNodeType<out_forbidden>("out_forbidden");


                // factory.registerNodeType<Node_CHARGE>("Node_CHARGE");
                // factory.registerNodeType<Node_CHARGE_OUTPOST>("Node_CHARGE_OUTPOST");
                // factory.registerNodeType<Node_RADAR>("Node_RADAR");
                // factory.registerNodeType<Node_CHARGE_1>("Node_CHARGE_1");
                // factory.registerNodeType<Node_CHARGE_2>("Node_CHARGE_2");
                factory.registerNodeType<Node_DEFENCE>("Node_DEFENCE");
                factory.registerNodeType<Node_RADICAL>("Node_RADICAL");
                
                factory.registerBehaviorTreeFromFile(DECISION_PACKAGE_PATH_XML + DD.config["tree_xml"]["xml_name"].as<string>());
                tree_ = factory.createTree("MainTree");
                CustomLogger logger(tree_);

                setlocale(LC_ALL, "");
                stop_thread_=false;
                area_manager.init();
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                time_measure tm;       
                while(rclcpp::ok() && !stop_thread_)
                {
                    tm.start();
        
                    tree_.tickOnce();
                    //决策数据
                    mes_puber->pub();
                    long long elapsed_time = tm.stop<chrono::microseconds>();
                    cout << "计算目标点耗时：" << elapsed_time << endl;
                    //updata_timer();
                    cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << endl;
                    tree_.sleep(chrono::milliseconds(delay_milsec_));
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << "行为树线程异常: " << e.what() << std::endl;
            }
        }
    );
    

    
}

decision_behaviortree::~decision_behaviortree()
{
    stop_thread_ = true;
    if (tree_thread_.joinable())
    {
        tree_thread_.join();
    }
}