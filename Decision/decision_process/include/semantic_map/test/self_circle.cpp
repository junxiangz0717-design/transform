#include "semantic_map.h"
#include <iostream>
#include <string>

int main() {
    // 定义动态变化的坐标
    double x = 0.0;
    double y = 0.0;
    bool is_in_any_area =false;
    string area_name;

    ref_PointPtr self_pos = make_shared<ref_Point>(x, y);
    PointManager::registerElement("自身位置", self_pos);
    PointManager::registerElement("autoaim_pursuit", make_shared<const_Point>(.0, .0));
    
    CirclePtr too_near_area = make_shared<Circle>(self_pos, 0.5); // 目标过近区域
    CirclePtr temp_autoaim_area = make_shared<Circle>(self_pos, 5.5); 
    CirclePtr temp_too_far_area = make_shared<Circle>(self_pos, 7.5); 

    AreaManager::registerElement("目标过近区域", too_near_area);
    
    MixAreaPtr too_far_area = temp_too_far_area - temp_autoaim_area; // 目标过远区域
    AreaManager::registerElement("目标过远区域", too_far_area);
    
    MixAreaPtr autoaim_area = temp_autoaim_area - too_near_area; // 自瞄阈值区域
    AreaManager::registerElement("自瞄阈值区域", autoaim_area);
    
    AreaManager::printAll();
    // 测试区域是否包含某个点
    auto test_point = make_shared<ref_Point>(5.0, 5.0);
    if (test_point->is_in(area_manager["目标过近区域"])) {
        is_in_any_area = true;
        area_name = "目标过近区域";
    } 
    // 检查是否在 自瞄阈值区域
    else if (test_point->is_in(area_manager["自瞄阈值区域"])) {
        is_in_any_area = true;
        area_name = "自瞄阈值区域";
    } 
    // 检查是否在 目标过远区域
    else if (test_point->is_in(area_manager["目标过远区域"])) {
        is_in_any_area = true;
        area_name = "目标过远区域";
    }

    if (is_in_any_area) {
        std::cout << "Test point is inside the " << area_name << "." << std::endl;
        is_in_any_area=false;
    } else {
        std::cout << "不追击" << std::endl;
    }

    //模拟坐标变化
    x = 4.0;
    y = 4.0;

    if (test_point->is_in(area_manager["目标过近区域"])) {
        is_in_any_area = true;
        area_name = "目标过近区域";
    } 
    // 检查是否在 自瞄阈值区域
    else if (test_point->is_in(area_manager["自瞄阈值区域"])) {
        is_in_any_area = true;
        area_name = "自瞄阈值区域";
    } 
    // 检查是否在 目标过远区域
    else if (test_point->is_in(area_manager["目标过远区域"])) {
        is_in_any_area = true;
        area_name = "目标过远区域";
    }

    if (is_in_any_area) {
        std::cout << "Test point is inside the " << area_name << "." << std::endl;
        is_in_any_area=false;
    } else {
        std::cout << "不追击" << std::endl;
    }

    return 0;
}