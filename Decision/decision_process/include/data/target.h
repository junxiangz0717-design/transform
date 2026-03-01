//
// Created by zh on 24-3-9.
//
#pragma once

#include "semantic_map/include/semantic_map.h"

#include <sys/types.h>
#include <vector>
#include "data/enum.h"

#include "tool/time/zh_timer.h"

const std::vector<string> TARGET_NAME = {"英雄", "工程", "3号步兵", "4号步兵", "前哨", "基地", "哨兵"};
// RMUL下需要保持与目标数量(7个)一致，未使用的名称用占位符填充

/**
 * @brief 目标数据结构
 */
struct Target_Data
{
    /** 自瞄 **/
    uint8_t Shoot_Mode = 0;
    uint8_t Target_Mode = 0;
    uint8_t autoaim_target = 7; //  自瞄目前目标
    uint8_t decision_target = 0;
    uint8_t autoaim_num = 0;
    Target pre_target = Target::NONE; // 上一次的目标

    std::vector<Target> targets_vector{};          // 自瞄目标
    std::vector<double> targets_distance_vector{}; // 自瞄目标距离
    std::vector<double> targets_x_y{};             // 自瞄目标坐标
    std::vector<PointPtr> targets_point{};         // 自瞄目标点(用于语义地图)
    /****/

    /** 感知 **/
    std::vector<Target> detector_targets_vector{};  // 感知目标
    std::vector<double> detector_distance_vector{}; // 感知目标距离
    std::vector<double> detector_targets_x_y{};     // 感知目标坐标
    std::vector<PointPtr> detector_targets_point{}; // 感知目标点(用于语义地图)
    std::vector<double> detector_angle{};           // 相对于车身正方向
    /****/

    /**其他 */

    std::vector<double> targets_disappear_x_y{};     // 自瞄目标消失坐标(暂时弃用)
    std::vector<PointPtr> targets_disappear_point{}; //
    std::vector<AreaPtr> targets_disappear_area{};
    std::vector<zh_timer> targets_disappear_timeout{
        zh_timer{450},
        zh_timer{450},
        zh_timer{450},
        zh_timer{450},
        zh_timer{450},
        zh_timer{450},
        zh_timer{450},
    }; // 目标消失计时器

    /** 雷达 **/
    std::vector<double> radar_x_y{};
    std::vector<PointPtr> radar_point{};

    /** 阵亡 **/
    // std::vector<double> targets_die_x_y{};
    // std::vector<PointPtr> targets_die_point{};

    Target_Data()
    {
        targets_distance_vector.assign(7, .0); // assign大小设置为7，每个默认为0
        detector_distance_vector.assign(7, .0);
        detector_angle.assign(7, .0);

        targets_x_y.assign(14, .0);
        detector_targets_x_y.assign(14, .0);
        targets_disappear_x_y.assign(14, .0);

        radar_x_y.assign(12, .0);
    }

    void register_to_manager()
    {
        // 初始化自瞄目标数据
        for (int i = 0; i < 14; i += 2)
        {
            auto target_point = make_shared<ref_Point>(targets_x_y[i], targets_x_y[i + 1]);
            PointManager::registerElement("自瞄" + TARGET_NAME[i / 2], target_point);
            targets_point.emplace_back(target_point);
        }
        // 初始化自瞄消失点
        for (int i = 0; i < 14; i += 2)
        {
            auto disappear_point = make_shared<ref_Point>(targets_disappear_x_y[i], targets_disappear_x_y[i + 1]);
            PointManager::registerElement("自瞄消失" + TARGET_NAME[i / 2], disappear_point);
            targets_disappear_point.emplace_back(disappear_point);
            CirclePtr target_area = make_shared<Circle>(disappear_point, 0.5);
            AreaManager::registerElement("目标消失范围", target_area);
            targets_disappear_area.emplace_back(target_area);
        }
        // 初始化雷达数据
        radar_x_y.assign(12, .0);
        for (int i = 0; i < 12; i += 2)
        {
            auto point = make_shared<ref_Point>(radar_x_y[i], radar_x_y[i + 1]);
            PointManager::registerElement("雷达" + TARGET_NAME[i / 2], point);
            radar_point.emplace_back(point);
        }

        // 初始化阵亡点
        // targets_die_x_y.assign(8, .0);
        // for (int i=0; i<8; i+=2)
        // {
        //     auto die_point = make_shared<ref_Point>(targets_die_x_y[i], targets_die_x_y[i+1]);
        //     PointManager::registerElement("阵亡" + TARGET_NAME[i / 2], die_point);
        //     targets_die_point.emplace_back(die_point);
        // }
    }
};