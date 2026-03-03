/**
 * @brief 决策全局数据类头文件，包含该头文件的所有文件使用公共对象DD（decisionData的别名）访问决策的全局变量
 * @author 唐真卓
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <vector>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include "data/target.h"
#include "data/enum.h" 
#include "semantic_map/include/semantic_map.h"
#include "path/decision_package_path.h"

#include "tool/time/zh_timer.h"
#include "tool/power.h"
#include "tool/Sethp.h"

using namespace std;

// 决策全局数据

class decision_data
{
private:

public:
//比赛数据####
    Style style               = Style::NORMAL;        // 默认为NORMAL
    State state               = State::FINDING;       // 默认为FINDING
    int strategy              = 0;   
    int time                  = 300;                  // 比赛剩余时间
    int game_period;                                  //比赛阶段
//#######
    

//移动数据####
    //当前目标点
    float cur_decision_point_x = 0;      
    float cur_decision_point_y = 0;         

    // 云台手给点或者决策目标点
    bool have_goal = false;//云台手点位
    float command_x{};
    float command_y{};
    float pre_command_x{};
    float pre_command_y{};

    //导航相关
    float goal_x{};
    float goal_y{};
    bool can_arrive;                    //能否到达
    bool is_arrive;                     //是否到达
    bool can_pathplan_stop = true;      //能否停下
    zh_timer pub_point{3000};           //发送间隔
//#######

//自身数据####
    int posture = 3;   // 默认是移动姿态 1进攻 2防御 3移动
    bool is_in_block ;          //是否在斜坡
    int ammo_thresholds = 10;
    int is_in_fort = 0;         //是否在堡垒
    int base_spin_mode = 0;     //底盘小陀螺旋转标志位 0停止 1逆时针 2顺时针 3变速   ###目前弃用，通过decision_mode控制
    Tripod_Spin tripod_spin = Tripod_Spin::PI; // 云台自旋模式

    std::chrono::system_clock::time_point death_time;
    int buy_respawn = 0;    //买活次数

    int is_no_ammo = 0;
    int ammo = 300;         // 允许发弹量   RMUC为300
    int get_ammo_count = 0; // 领弹轮数
    int available_ammo = 0; // 可领总发弹量

    int is_invincible = 0;       // 无敌状态 0：非无敌 1：无敌 
    int can_respawn = 0;    // 是否可以重生 0：不可以 1：可以

    int defence_buff = 0; // 防御增益
    int hp_sentry = 400;
    double pre_x = 0.0;   //上次移动位置x
    double pre_y = 0.0;   //上次移动位置y
    double cur_x = 0.50;  //当前移动位置x
    double cur_y = 0.50;  //当前移动位置y
    ref_PointPtr self_pos = make_shared<ref_Point>(cur_x, cur_y); // 当前位置指针形式，用于语义地图

    double big_yaw{};  // 云台大yaw角 相对于地图坐标系
    double delta_yaw{};  // 大小yaw角差值
    int pitch_mode = 0; // 云台pitch角 0正常 1抬头只打前哨 2低头 3全

    int hp_thresholds = 230;        // 血量阈值
    int add_to_target_hp = 395;     // 回血上限
    int hp_state = 0;               // 0:健康 1：不健康 2：回血中 3回家中

    int attack_buff =0; //攻击增益
    int defence_debuff =0; //防御负增益

    int in_hit = 0;//受击
    int in_forbidden = 0;  //在禁区
    int remain_energy = 0; //剩余能量

//#######

//机器人数据####
    int our_hero_hp     = 200;
    int our_engineer_hp = 200;
    int our_foot_3_hp   = 200;
    int our_foot_4_hp   = 200;
    int our_base_hp     = 5000;
    int our_outpost_hp  = 1500;

    double our_hero_x = 0.0;
    double our_hero_y = 0.0;
    double our_engineer_x = 0.0;
    double our_engineer_y = 0.0;
    double our_foot_3_x = 0.0;
    double our_foot_3_y = 0.0;
    double our_foot_4_x = 0.0;
    double our_foot_4_y = 0.0;
    ref_PointPtr foot_3 = make_shared<ref_Point>(our_foot_3_x, our_foot_3_y);
    ref_PointPtr foot_4 = make_shared<ref_Point>(our_foot_4_x, our_foot_4_y);
    ref_PointPtr hero = make_shared<ref_Point>(our_hero_x, our_hero_y);

    int enemy_hero_hp    = 200;
    int enemy_engineer_hp= 200;
    int enemy_foot_3_hp  = 200;
    int enemy_foot_4_hp  = 200;
    int enemy_sentry_hp  = 400;

    int enemy_base_hp    = 5000;
    int enemy_outpost_hp = 1500;
    zh_timer enemy_hero_timer{12000}; // 敌方1号血量校验计时器
    zh_timer enemy_foot_3_timer{12000}; // 敌方3号血量校验计时器
    zh_timer enemy_foot_4_timer{12000}; // 敌方4号血量校验计时器
    zh_timer enemy_sentry_timer{12000}; // 敌方哨兵血量校验计时器
    vector<int>enemy_hp={200,200,200,200,200,200,400}; //敌方机器人血量 英雄,工程,3号,4号,前哨,基地,哨兵
//#######

//自瞄数据####
    bool is_pursuiting = false;//是否追击
    Target_Data targets;
    int rune = 0;               
    int to_rune = 0;            //打符模式
    zh_timer target_timer = {4000}; // 自瞄无目标超时器 用与判断领弹
    zh_timer autoaim_connect_timer = {2000}; // 自瞄连接超时
    zh_timer detector_connect_timer = {2000}; // 感知相机连接超时
    zh_timer autoaim_reconnect_timer = {5000}; // 自瞄相机连接超时
    zh_timer detector_reconnect_timer = {5000}; // 感知相机连接超时
    zh_timer detector_turn = {1000};
    zh_timer hp_0_timer{500}; // 0血量校验计时器
    zh_timer autoaim_dropped_frames{500};//自瞄丢帧超时器
    zh_timer pursuit_timer{3000}; // 追击超时器
    zh_timer detector_again{5000};//感知转头冷却时间
    zh_timer outpost{300}; // 追击超时器
    zh_timer change_time{5000};//风格切换时长
    zh_timer find_target_timer{25000};//巡逻区域切换时长
    zh_timer block_timer{100};        //卡斜坡时间

    int color{};                      
    int decision_shoot_mode = 0;      
    int need_turn_yaw =0;             
//#######
    //无敌单位计算
    EnergyCalculator power;           
    EnemyHealthManager set_hero;
    EnemyHealthManager set_engineer;
    EnemyHealthManager set_foot_3;
    EnemyHealthManager set_foot_4;
    EnemyHealthManager set_sentry;


//其他####
    int rush = 0; //是否冲家
    vector<pair<double,double>>radar_point_vec;
    pair<double,double>radar_point;
    int radar_have = 0;
    int radar_arrive = 0;
    int is_start = 1;
    int is_weak = 0;   //是否虚弱状态
    zh_timer fort_timer{15000};      //堡垒计时器
    bool is_outposting = false; //是否在打前哨
    int is_in_add_area{};//是否在补给区
    int is_center_area{}; // 中心区域占领状态 
    int buff_area_state{}; // 增益区占领状态(0无，1我方，2敌方，3双方)
    bool is_in_center_area{};// 烧饼自身是否在中心区域
    float decision_yaw_az{};//给电控要转动的yaw的角速度
    int start_time{}; //比赛正计时，仅用于计算发弹量
    chrono::high_resolution_clock::duration start_time_chrono;
    YAML::Node config;
    float big_yaw_az = 0.0; //大yaw角速度
    float xx;//仿真用
    float yy;//仿真用
    int pub; //仿真用

    /*
    以下为配置信息
    */
    int     config_HZ;
    float   config_detector_turn_az;
    float   config_az;
    bool    config_is_skip_gamestart;
    bool    config_is_scan;
    float   config_max_scan_offset_yaw;
    float   config_min_scan_offset_yaw;
    int     config_pursuit_delay;
    bool    config_check_connect;
    int     priority;
    int     config_detector_turn_time;
    int     config_detector_again;
    bool    config_is_serial_hp;
    vector<vector<int>> target_list;
    vector<int>         score_list_0;
    vector<int>         score_list_1;
    vector<int>         score_list_2;
    vector<int>         score_list_3;

//#######

    decision_data();

    bool is_arrived_point(const double& cur_x, const double& cur_y, const double& goal_x, const double& goal_y) const;//判断是否到达目标点

    bool is_approachable(const Target &id) const;//判断单个目标是否可攻击

    vector<uint8_t> approachable_state(const vector<int>&enemy_hp_)const; //给自瞄发布的时候调用
    
    double normalize_angle(const double &angle) const;//标准化角度到-180到180之间

    float scan_to_target(int target) const;
};

inline float decision_data::scan_to_target(int target) const
{
    if(self_pos->is_in(targets.targets_disappear_area[target]))
    {
        double yaw_1 = self_pos->CCW_sector_to(targets.targets_disappear_area[target]).first;
        double yaw_2 = self_pos->CCW_sector_to(targets.targets_disappear_area[target]).second;
        if(big_yaw > yaw_1){return -config_az;}
        else if(big_yaw < yaw_2){return  config_az;}
    }
    else
    {
        return decision_yaw_az;
    }
}

decision_data::decision_data()
{
    try 
    {
        config = YAML::LoadFile(YAML_PATH);
        cout << "YAML file loaded successfully." << endl;
        } catch (const YAML::BadFile& e) {
        cerr << "Failed to load YAML file: " << e.what() << endl;
        } catch (const YAML::Exception& e) {
        cerr << "YAML parsing error: " << e.what() << endl;
    }
    config_az                   = config["big_yaw_az"].as<float>();
    config_HZ                   = config["HZ"].as<int>();
    config_detector_turn_az     = config["detector_turn_az"].as<float>();
    config_is_skip_gamestart    = config["is_skip_gamestart"].as<bool>();
    config_is_scan              = config["is_scan"].as<bool>();
    config_max_scan_offset_yaw  = config["max_scan_offset_yaw"].as<float>();
    config_min_scan_offset_yaw  = config["min_scan_offset_yaw"].as<float>();
    config_detector_turn_time   = config["detector_turn_time"].as<int>();
    config_detector_again       = config["detector_again"].as<int>();
    config_pursuit_delay        = config["pursuit_delay"].as<int>();
    config_check_connect        = config["check_connect"].as<bool>();
    priority                    = config["priority"].as<int>();
    score_list_0                = config["score_list_0"].as<vector<int>>();
    score_list_1                = config["score_list_1"].as<vector<int>>();
    score_list_2                = config["score_list_2"].as<vector<int>>();
    score_list_3                = config["score_list_3"].as<vector<int>>();
    config_is_serial_hp         = config["is_accept_serial_hp"].as<bool>();
    target_list.push_back(score_list_0);
    target_list.push_back(score_list_1);
    target_list.push_back(score_list_2);  
    target_list.push_back(score_list_3);
    outpost.reset();
    detector_again={config_detector_again};
    detector_again.reset();
    fort_timer.reset();
    change_time.reset();
    detector_turn={config_detector_turn_time};
    power.reset();
    
}

inline bool decision_data::is_approachable(const Target &id) const
{
    switch (id)
    {
        case HERO:
            return enemy_hero_hp < 1000 && enemy_hero_hp > 0 ;
        case ENGINEER:
            return enemy_engineer_hp < 1000 && enemy_engineer_hp > 0;
        case FOOT_3:
            return enemy_foot_3_hp < 1000 && enemy_foot_3_hp > 0 ;
        case FOOT_4:
            return enemy_foot_4_hp < 1000 && enemy_foot_4_hp > 0 ;
        case SENTRY:
            return enemy_sentry_hp < 1001 && enemy_sentry_hp > 0 ;
        case OUTPOST:
            return enemy_outpost_hp > 0;
        default://默认基地不无敌
            return 1;
    }
}

inline vector<uint8_t> decision_data::approachable_state(const vector<int>&enemy_hp_) const
{
    int i=0;
    vector<uint8_t>is_approach;
    for(const auto &hp : enemy_hp_)
    {
        if(i==4 || i ==5 )
        {
            i++;
            is_approach.push_back(1);//前哨基地不无敌
            continue;
        }
        if(hp<1000 && hp >0)is_approach.push_back(1);
        else is_approach.push_back(0);
        i++;
    }
    return is_approach;
}

inline double decision_data::normalize_angle(const double &my_angle) const
{
    double angle = my_angle;
    // 将角度标准化到 -180 到 180 之间
    while (angle > 180.0) {
        angle -= 360.0;
    }
    while (angle < -180.0) {
        angle += 360.0;
    }
        return angle;
}

inline bool decision_data::is_arrived_point( const double& cur_x, const double& cur_y, const double& goal_x, const double& goal_y ) const
{
    double dis_to_goal;
    double x_dir = cur_x - goal_x;
    double y_dir = cur_y - goal_y;

    dis_to_goal = sqrt(x_dir*x_dir + y_dir*y_dir);

    double reach_goal_dis_ = 0.3;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"distence_to_goal: %f",dis_to_goal);
    if(dis_to_goal<reach_goal_dis_)
    {
        return true;
    }
    return false;
}

inline decision_data decisionData;       // 决策数据全局变量类对象
inline decision_data &DD = decisionData; // 决策数据全局变量类对象缩写