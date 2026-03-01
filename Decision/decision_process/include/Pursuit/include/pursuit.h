#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <omp.h>
#include <vector>
#include <string>

using namespace cv;
using namespace std;


const int MAP_W = 290;
const int MAP_H = 160;
const int MAX_MOVE_RADIUS = 50;
const double MAP_W_REAL = 28.0;
const double MAP_H_REAL = 15.0;
/*qingqing地图的yaml配置*/
const double qingqing_origin_width  = 435;  
const double qingqing_origin_height = 197;
const double qingqing_real_width  = 21.75;  
const double qingqing_real_height = 9.85;
const double qingqing_origin_x = 0.0;
const double qingqing_origin_y = 0.0;
const double resolution = 0.05; 
const double new_res_x = 0.075;
const double new_res_y = 0.0615625;
/*white地图的yaml配置*/
const double white_origin_width = 0;
const double white_origin_height = 0;
const double white_res_x = 0.178;
const double white_res_y = 0.333;
struct ForbiddenZone 
{
    enum Type { RECTANGLE, CIRCLE };
    Type type;
    cv::Rect rect;        // 矩形参数：左上角坐标和宽高
    cv::Point center;     // 圆形参数：圆心
    int radius;           // 圆形参数：半径
};

enum class TrackingState 
{
    INITIALIZING,  // 初始化（寻找最优位置）
    STEADY,        // 稳定状态（保持当前位置）
    ADJUSTING      // 调整状态（重新计算最优位置）
};

/*地图基类*/
class map_handle
{
private:

    Mat gray;
    Mat binary;
    Mat static_map;
    Mat heatmap;
    Mat distField;
    vector<ForbiddenZone> zones; // 禁区列表
    int ul_offset_x = 0;        // 赛场白色区域偏移
    int ul_offset_y = 0;
    double ul_res_x = 0.05;     // 动态计算的分辨率
    double ul_res_y = 0.05;
public:
    /* 调用init函数初始化*/
    map_handle();
    /*初始化图像*/
    void init(const string& path);
    /*获取静态障碍图*/
    Mat get_static_map();
    /*获取二值图*/
    Mat get_binary_map();
    /*获取热力图*/ 
    Mat get_heatmap();
    /*获取灰度图*/
    Mat get_gray_map();
    /*获取地图行数*/
    float get_row();
    /*获取地图列数*/
    float get_col();
    /*以灰度图形式导入*/
    Mat load_as_grayscale(const std::string& path);
    /*自适应二值化*/
    Mat adaptive_binarize(const cv::Mat& gray);
    /*缩放到标准大小*/
    Mat resize_to_standard(const Mat& img);
    /*验证并修复*/
    void validate_and_repair(cv::Mat& binary);
    /*修复黑线*/
    Mat repair_black_lines(const Mat& binary_map);
    /*预选场生成*/
    Mat computeDistanceField();
    /*禁区*/
    bool forbidden(const cv::Point& p);
    /*RMUC地图坐标转像素坐标*/
    cv::Point map2pixel(const double& x, const double&y);
    cv::Point map2pixel(const pair<double, double>& p);
    /*像素坐标转RMUC地图坐标*/
    pair<double, double>pixel2map(const cv::Point& p);
    /*qingqing地图坐标转像素坐标*/
    cv::Point qingqing2pixel(const double& x, const double&y);
    /*像素坐标转qingqing地图坐标*/
    pair<double, double>pixel2qingqing(const cv::Point& p);
    /*white地图坐标转像素坐标*/
    cv::Point white2pixel(const double& x, const double&y);
    /*像素坐标转white地图坐标*/
    pair<double, double>pixel2white(const cv::Point& p);
    /*ul地图坐标转像素坐标*/
    cv::Point ul2pixel(const double& x, const double&y);
    /*像素坐标转ul地图坐标*/
    pair<double, double>pixel2ul(const cv::Point& p);

};

/*可见目标追击类*/
class visible_target :virtual public map_handle
{
private:
    TrackingState current_state = TrackingState::INITIALIZING;
    cv::Point last_position;         
public: 
    visible_target():map_handle(){};
    /*目标点选择*/
    cv::Point find_visible_target(const cv::Point& agent, const cv::Point& enemy);
    /*检查路径可达*/
    bool is_path_clear(const cv::Point& p1, const cv::Point& p2);
    /*检查点是否有效*/
    bool is_point_valid(const cv::Point& p);
    /*视野函数*/
    float calculate_view_score(const cv::Point& candidate, const cv::Point& enemy);
    /*视野评分函数*/
    float evaluate_line_black_ratio(const cv::Point& start, const cv::Point& end, int offset);
    /*计算雷达目标点*/
    cv::Point get_radar_target(const cv::Point& enemy);
};



/*调试类及总调用类*/
class relocator :public visible_target
{
private:
    int mouse_event = -1;
    bool is_agent_set = false;
    bool is_enemy_set = false;
    bool is_dynamics_set = false;
    cv::Point agent;
    cv::Point enemy;
    vector<cv::Point> dynamics;
    cv::Point mouse_click_point;
public:
    relocator():map_handle(),visible_target(){};
    /*鼠标事件*/
    void on_mouse(int event, int x, int y, int flags, void* param);
    /*静态鼠标事件*/
    static void staticOnMouse(int event, int x, int y, int flags, void* param);
    /*设置敌人*/
    bool set_enemy(const cv::Point& p);
    /*设置自身*/
    bool set_agent(const cv::Point& p);
    /*设置其他*/
    bool set_dynamics(const cv::Point& p);
    /*可见目标可视化*/
    void visible_visual();
    void visible_visual(const cv::Point& agent, const cv::Point& enemy);
    /*可见目标可视化点*/
    Mat visualize_points(const cv::Point& agent, const cv::Point& enemy);

};
inline relocator enemy_relocator;
inline relocator &ER = enemy_relocator;