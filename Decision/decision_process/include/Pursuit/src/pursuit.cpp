#include "pursuit.h"
#include "decision_package_path.h"
#include "time_measure.h"


map_handle::map_handle()
{
    omp_set_num_threads(8);
    heatmap = Mat::zeros(MAP_H, MAP_W, CV_32F);
    init("qingqing2026.pgm");
    /*添加禁区*/
        /*RMUC*/
    // ForbiddenZone rectZone_1,rectZone_2,rectZone_3;
    // rectZone_1.type = ForbiddenZone::RECTANGLE;
    // rectZone_1.rect = cv::Rect(130, 29, 65, 25);
    // zones.push_back(rectZone_1);
    // rectZone_2.type = ForbiddenZone::RECTANGLE;
    // rectZone_2.rect = cv::Rect(106, 108, 56, 22);
    // zones.push_back(rectZone_2);
    // rectZone_3.type = ForbiddenZone::RECTANGLE;
    // rectZone_3.rect = cv::Rect(37, 51, 23, 8);
    // zones.push_back(rectZone_3);    
    // ForbiddenZone circleZone_1, circleZone_2;
    // circleZone_1.type = ForbiddenZone::CIRCLE;
    // circleZone_1.center = cv::Point(212, 78);
    // circleZone_1.radius = 15;
    // circleZone_2.type = ForbiddenZone::CIRCLE;
    // circleZone_2.center = cv::Point(70, 77);
    // circleZone_2.radius = 15;
    // zones.push_back(circleZone_1);
    // zones.push_back(circleZone_2);
        /*RMUL 2026 维修区禁区 (场地12m×8m, 维修区1.5m×2m)*/
    ForbiddenZone repairZone_blue, repairZone_red;
    // 右下角蓝方维修区: x=[10.5,12]m, y=[0,2]m
    repairZone_blue.type = ForbiddenZone::RECTANGLE;
    repairZone_blue.rect = cv::Rect(253, 120, 37, 40);
    zones.push_back(repairZone_blue);
    // 左上角红方维修区: x=[0,1.5]m, y=[6,8]m
    repairZone_red.type = ForbiddenZone::RECTANGLE;
    repairZone_red.rect = cv::Rect(0, 0, 36, 40);
    zones.push_back(repairZone_red);
        /*qingqing*/
}
Mat map_handle::get_binary_map(){return binary;}
Mat map_handle::get_static_map(){return static_map;}
Mat map_handle::get_heatmap(){return heatmap;}
Mat map_handle::get_gray_map(){return gray;}
float map_handle::get_row(){return this->static_map.rows;}
float map_handle::get_col(){return this->static_map.cols;}
void map_handle::init(const string& path)
{
    // 读取地图
    //string map_path = PURSUIT_MAP_PATH + path; // 替换为实际地图路径
    string map_path = SENTRY_PURSUIT_MAP_PATH + path;
    gray = load_as_grayscale(map_path);
    // 自适应二值化
    binary = adaptive_binarize(gray);
    // // 验证并修复二值图
    // validate_and_repair(binary);//rmuc地图修复效果不佳
    // 修复黑线
    binary = repair_black_lines(binary);
    // 缩放到标准大小
    static_map = resize_to_standard(binary);

    // 自动寻找白色区域的完整边界 (仅在 UL 模式下)
    if (path.find("ul") != string::npos || path.find("UL") != string::npos) {
        int min_x = static_map.cols, max_x = 0;
        int min_y = static_map.rows, max_y = 0;
        bool found_any = false;

        for (int y = 0; y < static_map.rows; ++y) {
            for (int x = 0; x < static_map.cols; ++x) {
                if (static_map.at<uchar>(y, x) == 255) {
                    min_x = min(min_x, x);
                    max_x = max(max_x, x);
                    min_y = min(min_y, y);
                    max_y = max(max_y, y);
                    found_any = true;
                }
            }
        }

        if (found_any) {
            ul_offset_x = min_x;
            ul_offset_y = min_y;
            int white_w = max_x - min_x + 1;
            int white_h = max_y - min_y + 1;
            // 分辨率 = 实际长度 / 这里的白色区域像素宽度
            ul_res_x = 12.0 / static_cast<double>(white_w);
            ul_res_y = 8.0 / static_cast<double>(white_h);
            
            cout << "--- 赛场地图分析 (UL Mode) ---" << endl;
            cout << "白色区域起始点 (Offset): (" << ul_offset_x << ", " << ul_offset_y << ")" << endl;
            cout << "白色区域像素尺寸: " << white_w << " x " << white_h << endl;
            cout << "计算动态分辨率: x=" << ul_res_x << ", y=" << ul_res_y << endl;
            cout << "--------------------" << endl;
        }
    }

    // imshow("Resized Map", static_map);
    // imshow("Binary Map", binary);
    // imshow("Gray Map", gray);
    // waitKey(0); // 等待按键
}
Mat map_handle::load_as_grayscale(const std::string& path) 
{
    Mat img = imread(path, IMREAD_COLOR);
    if (img.empty()) 
    {
        cerr << "错误：无法读取地图文件 " << path << endl;
        exit(EXIT_FAILURE);
    }
    
    // 转换为灰度图（若输入为RGB）
    if (img.channels() == 3) 
    {
        cvtColor(img, gray, COLOR_BGR2GRAY);
    } else 
    {
        gray = img.clone();
    }
    return gray;
}
Mat map_handle::adaptive_binarize(const cv::Mat& gray) 
{
    // Otsu自动阈值（适合对比度较高的地图）
    threshold(gray, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
    return binary;
}
Mat map_handle::resize_to_standard(const Mat& img) 
{
    resize(img, static_map, Size(290, 160), 0, 0, INTER_NEAREST);
    return static_map;
}
void map_handle::validate_and_repair(cv::Mat& binary) 
{
    // 检查是否存在可行区域（白色像素）
    if (countNonZero(binary) == 0) 
    {
        cerr << "错误：地图无可行区域，请检查原始文件！" << endl;
        exit(EXIT_FAILURE);
    }
    //填充小孔洞
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(2.5,2.5));
    morphologyEx(binary, binary, MORPH_CLOSE, kernel);
}
Mat map_handle::repair_black_lines(const Mat& binary_map) 
{
    // 定义膨胀核（3x3十字形核，针对细线优化）
    Mat kernel = getStructuringElement(MORPH_CROSS, Size(5, 5));
    
    // 仅对黑色区域（障碍物）膨胀
    Mat dilated_black;
    dilate(binary_map == 0, dilated_black, kernel);  // 提取黑色区域并膨胀
    
    // 合并结果：原图白色区域 + 膨胀后的黑色区域
    Mat repaired_map = binary_map.clone();
    repaired_map.setTo(0, dilated_black);  // 将膨胀后的黑色区域覆盖到原图
    
    return repaired_map;
}
// Mat map_handle::computeDistanceField() {distanceTransform(static_map, distField, DIST_L2, 3);}
cv::Point map_handle::map2pixel(const double& x, const double&y){return cv::Point(static_cast<int>(x*10),160-static_cast<int>(y*10));}
cv::Point map_handle::map2pixel(const pair<double, double>& p){return cv::Point(static_cast<int>(p.first * 10), 160 - static_cast<int>(p.second * 10));}
pair<double, double> map_handle::pixel2map(const cv::Point& p){return pair<double, double>(static_cast<double>(p.x)/10.0, (160-static_cast<double>(p.y))/10.0);}
cv::Point map_handle::qingqing2pixel(const double& x, const double&y)
{
    cv::Point pixel;
    pixel.x = static_cast<int>((x - qingqing_origin_x) / new_res_x);
    pixel.y = static_cast<int>(160 - (y - qingqing_origin_y) / new_res_y);
    return pixel;
}
pair<double, double> map_handle::pixel2qingqing(const cv::Point& p)
{
    pair<double, double> map;
    map.first = p.x * new_res_x + qingqing_origin_x;
    map.second = (160 - p.y) * new_res_y + qingqing_origin_y;
    map.second = std::round(map.second * 100.0) / 100.0;
    return map;
}
cv::Point map_handle::white2pixel(const double& x, const double&y)
{
    cv::Point pixel;
    pixel.x = static_cast<int>(x / white_res_x);
    pixel.y = static_cast<int>(150 - (-y / white_res_y));
    return pixel;
}
pair<double, double> map_handle::pixel2white(const cv::Point& p)
{
    pair<double, double> map;
    map.first = p.x * white_res_x;
    map.second = -((150 - p.y) * white_res_y);
    return map;
}   

cv::Point map_handle::ul2pixel(const double& x, const double&y)
{
    cv::Point pixel;
    // 加上偏移量：像素 = (米 / 分辨率) + 赛场原点偏移
    pixel.x = static_cast<int>(x / ul_res_x) + ul_offset_x;
    pixel.y = static_cast<int>(-y / ul_res_y) + ul_offset_y;
    return pixel;
}

pair<double, double> map_handle::pixel2ul(const cv::Point& p)
{
    pair<double, double> map;
    // 减去偏移量：米 = (像素 - 赛场原点偏移) * 分辨率
    map.first = (p.x - ul_offset_x) * ul_res_x;
    map.second = -((p.y - ul_offset_y) * ul_res_y);
    return map;
}

bool map_handle::forbidden(const cv::Point& p) 
{
    for (const auto& zone : zones) {
        switch (zone.type) {
            case ForbiddenZone::RECTANGLE:
                // 判断点是否在矩形内
                if (zone.rect.contains(p)) {
                    return false;
                }
                break;
            case ForbiddenZone::CIRCLE: {
                // 提前计算半径的平方
                int squaredRadius = zone.radius * zone.radius;
                // 计算点与圆心的距离平方
                int dx = p.x - zone.center.x;
                int dy = p.y - zone.center.y;
                int squaredDist = dx * dx + dy * dy;
                // 判断距离是否小于等于半径平方
                if (squaredDist <= squaredRadius) {
                    return false;
                }
                break;
            }
            default:
                break;
        }
    }
    return true;
}    

Point visible_target::find_visible_target(const Point& agent, const Point& enemy)
{
    const int min_dist = 10;   // 最小搜索范围
    const int max_dist = 35;   // 最大搜索范围
    const int min_stay_dist = 10;   // 最小静止范围
    const int max_stay_dist = 40;   // 最大静止范围
    const int ideal_dist = 20; // 2米
    const int angle_steps = 36;
    const float view_weight = 0.3f; // 视野评分权重

    struct ThreadResult 
    {
        Point best_target;
        float best_score = -INFINITY;
        float best_agent_dist = INFINITY;
        Point nearest_target;
        float nearest_enemy_dist = INFINITY;
    };
    vector<ThreadResult> thread_results(omp_get_max_threads());
        // 检查当前状态
    if (current_state == TrackingState::STEADY && forbidden(agent)) 
    {
        float dist_to_enemy = norm(agent - enemy);
        if (dist_to_enemy >= min_stay_dist && dist_to_enemy <= max_stay_dist) {
            return agent;
        }
        current_state = TrackingState::ADJUSTING;
    }
    
    #pragma omp parallel for schedule(static)
    
    for (int i = 0; i < angle_steps; ++i) 
    {
        const float angle = 2 * CV_PI * i / angle_steps;
        auto& local_result = thread_results[omp_get_thread_num()];

        for (int dist = min_dist; dist <= max_dist; dist += 1) 
        {
            const Point candidate(
                enemy.x + dist * cos(angle),
                enemy.y + dist * sin(angle)
            );

            // 基础验证
            if (!is_point_valid(candidate)) continue;
            if (!is_path_clear(agent, candidate)) continue; 

            // 计算距离
            const float dist_to_enemy = norm(candidate - enemy);
            const float dist_to_agent = norm(candidate - agent);
            
            // 记录能到达的离敌人最近的点
            if (dist_to_enemy < local_result.nearest_enemy_dist) 
            {
                local_result.nearest_enemy_dist = dist_to_enemy;
                local_result.nearest_target = candidate;
            }

            // 1. 计算距离评分
            const float dist_score = 1.0f - abs(dist_to_enemy - ideal_dist) / (max_dist - min_dist);
            
            // 2. 计算视野评分
            const float view_score = calculate_view_score(candidate, enemy);
            
            // 3. 综合评分 = 距离评分(70%) + 视野评分(30%)
            const float total_score = dist_score * (1.0f - view_weight) + view_score * view_weight;
            
            // 更新最佳点
            if (total_score > local_result.best_score + 1e-5f) 
            {
                local_result.best_score = total_score;
                local_result.best_target = candidate;
                local_result.best_agent_dist = dist_to_agent;
            } 
            else if (abs(total_score - local_result.best_score) < 1e-5f) 
            {
                if (dist_to_agent < local_result.best_agent_dist)
                {
                    local_result.best_target = candidate;
                    local_result.best_agent_dist = dist_to_agent;
                }
            }
        }
    }

    // 合并结果
    Point final_target = agent;
    float global_best_score = -INFINITY;
    float global_best_agent_dist = INFINITY;
    float global_nearest_enemy_dist = INFINITY;

    for (const auto& res : thread_results)
    {
        if (res.best_score > global_best_score + 1e-5f) 
        {
            global_best_score = res.best_score;
            final_target = res.best_target;
            global_best_agent_dist = res.best_agent_dist;
        } 
        else if (abs(res.best_score - global_best_score) < 1e-5f) 
        {
            if (res.best_agent_dist < global_best_agent_dist) 
            {
                final_target = res.best_target;
                global_best_agent_dist = res.best_agent_dist;
            }
        }

        if (res.nearest_enemy_dist < global_nearest_enemy_dist) 
        {
            global_nearest_enemy_dist = res.nearest_enemy_dist;
            if (global_best_score == -INFINITY) 
            {
                final_target = res.nearest_target;
            }
        }
    }
    if (current_state == TrackingState::INITIALIZING || 
        current_state == TrackingState::ADJUSTING) 
    {
        // 若找到了有效点，切换到稳定状态
        if (global_best_score > -INFINITY) 
        {
            current_state = TrackingState::STEADY;
        }
    }
    // 更新状态
    return final_target;
}

float visible_target::calculate_view_score(const Point& candidate, const Point& enemy) 
{
    // 参数配置 
    const int offsets[] = {-6, -4, -2, 2, 4, 6};  // 平行线偏移量（像素）
    const float offset_weights[] = {0.1f, 0.15f, 0.25f, 0.25f, 0.15f, 0.1f}; // 偏移权重
    const float max_black_ratio = 0.3f; // 最大允许黑色像素比例
    
    // 计算主方向 
    const float dx = enemy.x - candidate.x;
    const float dy = enemy.y - candidate.y;
    const float dist = std::sqrt(dx*dx + dy*dy);
    
    // 处理候选点与目标点重合的特殊情况
    if (dist < 1e-5f) return 1.0f;
    
    // 单位方向向量
    const float ux = dx / dist;
    const float uy = dy / dist;
    
    // 垂直单位向量（用于平行线偏移）
    const float perp_ux = -uy;
    const float perp_uy = ux;
    
    // 主直线分析 
    float main_black_ratio = evaluate_line_black_ratio(candidate, enemy, 0);
    float main_line_score = 1.0f - main_black_ratio;
    
    // 平行线分析 
    float total_black_ratio = 0.0f;
    float total_weight = 0.0f;
    
    for (int i = 0; i < sizeof(offsets)/sizeof(offsets[0]); ++i) {
        const int offset = offsets[i];
        const float weight = offset_weights[i];
        
        const float black_ratio = evaluate_line_black_ratio(candidate, enemy, offset);
        total_black_ratio += black_ratio * weight;
        total_weight += weight;
    }
    
    // 计算加权平均黑色像素比例
    const float avg_black_ratio = total_black_ratio / total_weight;
    const float parallel_score = 1.0f - std::min(1.0f, avg_black_ratio / max_black_ratio);
    
    // 综合评分 
    return 0.6f * main_line_score + 0.4f * parallel_score;
}

// 计算单条直线上的黑色像素比例
float visible_target::evaluate_line_black_ratio(const Point& start, const Point& end, int offset) 
{
    // 计算方向向量和距离
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const float dist = std::sqrt(dx*dx + dy*dy);
    
    // 处理重合点
    if (dist < 1e-5f) return 0.0f;
    
    // 计算垂直偏移方向
    const float perp_ux = -dy / dist;  // 垂直单位向量X
    const float perp_uy = dx / dist;   // 垂直单位向量Y
    
    // 应用偏移
    const Point offset_start(
        start.x + offset * perp_ux,
        start.y + offset * perp_uy
    );
    
    const Point offset_end(
        end.x + offset * perp_ux,
        end.y + offset * perp_uy
    );
    
    // 创建线段迭代器
    LineIterator it(get_static_map(), offset_start, offset_end);
    if (it.count == 0) return 1.0f;  // 无效线段视为完全阻塞
    
    // 统计黑色像素
    int black_count = 0;
    int total_count = 0;
    
    Point prev_pos(-1, -1); // 记录上一个位置，避免重复计数
    
    for (int i = 0; i < it.count; ++i, ++it) {
        const Point pos = it.pos();
        
        // 跳过重复点（Bresenham算法可能产生重复）
        if (pos.x == prev_pos.x && pos.y == prev_pos.y) continue;
        prev_pos = pos;
        
        // 边界检查
        if (pos.x < 0 || pos.x >= get_static_map().cols ||
            pos.y < 0 || pos.y >= get_static_map().rows) {
            // 边界外视为障碍物
            black_count++;
            total_count++;
            continue;
        }
        
        // 检查像素
        if (get_static_map().at<uchar>(pos) == 0) {
            black_count++;
        }
        total_count++;
    }
    
    // 处理无线段点的情况
    if (total_count == 0) return 1.0f;
    
    return static_cast<float>(black_count) / total_count;
}

bool visible_target::is_path_clear(const Point& p1, const Point& p2)
{
    LineIterator it(this->get_static_map(), p1, p2);
    for (int i = 0; i < it.count; ++i, ++it) 
    {
        if (this->get_static_map().at<uchar>(it.pos()) == 0) 
        {
            return false;
        }
    }
    return true;
}
bool visible_target::is_point_valid(const Point& p) 
{
    if(p.x >= 0 && p.x < this->get_static_map().cols     && 
           p.y >= 0 && p.y < this->get_static_map().rows     &&
           this->get_static_map().at<uchar>(p.y, p.x) == 255 &&
           forbidden(p))
    {
        for (int y = p.y - 7; y <= p.y + 7; ++y) 
        {
            for (int x = p.x - 7; x <= p.x + 7; ++x) 
            {
                double distance = pow(x - p.x, 2) + pow(y - p.y, 2);
                if (distance <= 49) {if (this->get_static_map().at<uchar>(y, x) == 0) {return false;}}
            }
        }
        return true;
    }
    else{return false;}
}
cv::Point visible_target::get_radar_target(const cv::Point& p)
{
    if(p.x >= 0 && p.x < this->get_static_map().cols     && 
           p.y >= 0 && p.y < this->get_static_map().rows     &&
           this->get_static_map().at<uchar>(p.y, p.x) == 255 )
    {
        for (int y = p.y - 4; y <= p.y + 4; ++y) 
        {
            for (int x = p.x - 4; x <= p.x + 4; ++x) 
            {
                double distance = pow(x - p.x, 2) + pow(y - p.y, 2);
                if (distance <= 16) {if (this->get_static_map().at<uchar>(y, x) == 0) {return cv::Point(0,150);}}
            }
        }
        return p;
    }
    else{return cv::Point(0,150);}
}


void relocator::staticOnMouse(int event, int x, int y, int flags, void* param) 
{
    relocator* self = static_cast<relocator*>(param);
    if (self) self->on_mouse(event, x, y, flags, param);
    else std::cerr << "错误：无法获取鼠标事件的对象指针！" << std::endl;
}

void relocator::on_mouse(int event, int x, int y, int flags, void* param)
{
    relocator* self = static_cast<relocator*>(param);
    
    Rect image_roi = getWindowImageRect("Interactive Map");
    
    int img_cols = static_cast<int>(self->get_col());  // 290
    int img_rows = static_cast<int>(self->get_row());  // 160
    
    int map_x, map_y;
    
    // 检测 GTK 后端的特殊行为：
    // 当 ROI 尺寸远大于图像尺寸（比如放大窗口），但鼠标坐标仍在图像范围内
    // 这说明鼠标坐标已经是图像空间坐标，不需要缩放
    bool gtk_quirk = (image_roi.width > img_cols * 1.5 && x < img_cols * 1.5) ||
                     (image_roi.height > img_rows * 1.5 && y < img_rows * 1.5);
    
    if (gtk_quirk || image_roi.width <= 0 || image_roi.height <= 0) {
        // GTK 模式或无效 ROI：鼠标坐标已经是图像坐标，直接使用
        map_x = x;
        map_y = y;
    } else {
        // 正常模式：需要根据窗口/图像比例缩放
        float scale_x = (float)img_cols / image_roi.width;
        float scale_y = (float)img_rows / image_roi.height;
        map_x = static_cast<int>(x * scale_x);
        map_y = static_cast<int>(y * scale_y);
    }
    
    // 边界保护
    map_x = std::clamp(map_x, 0, img_cols - 1);
    map_y = std::clamp(map_y, 0, img_rows - 1);
    
    self->mouse_click_point = Point(map_x, map_y);

    if (event == EVENT_LBUTTONDOWN) self->mouse_event = 0;  // 左键
    if (event == EVENT_RBUTTONDOWN) self->mouse_event = 1;  // 右键
    if (event == EVENT_MBUTTONDOWN) self->mouse_event = 2;  // 中键
}

bool relocator::set_agent(const Point& p) 
{
    agent = p;
    is_agent_set = true;
    return true;
 }

bool relocator::set_enemy(const Point& p) 
{
    enemy = p;
    is_enemy_set = true;
    return true;
}

bool relocator::set_dynamics(const Point& p) 
{

    if(dynamics.size()==4){dynamics.clear();}
    is_dynamics_set = true;
    dynamics.push_back(p);
    return true;
}

Mat relocator::visualize_points(const Point& agent,const Point& enemy)
{
    time_measure tm;
    Mat color_map;
    cvtColor(get_static_map(), color_map, COLOR_GRAY2BGR);
    const Scalar agent_color(0, 255, 0);   // 绿色
    const Scalar enemy_color(0, 0, 255);   // 红色
    const Scalar target_color(255, 0, 0);  // 蓝色
    const int point_radius = 2;            
    if (is_agent_set)circle(color_map, agent, point_radius, agent_color, -1);
    if (is_enemy_set)circle(color_map, enemy, point_radius, enemy_color, -1);
    if (is_agent_set && is_enemy_set) 
    {
        //tm.start();
        Point target = find_visible_target(agent, enemy);

        //long long elapsed_time = tm.stop<chrono::microseconds>();
        //cout << "计算目标点耗时：" << elapsed_time << endl;
        if (target.x >= 0 && target.y >= 0)circle(color_map, target, point_radius, target_color, -1);
    }

    return color_map;
}
void relocator::visible_visual() 
{
    // 创建窗口
    namedWindow("Interactive Map", WINDOW_NORMAL);
    setMouseCallback("Interactive Map",&relocator::staticOnMouse, this);
    while (true) 
    {
        Mat visual_map = visualize_points(agent, enemy);
        imshow("Interactive Map", visual_map);

        // 处理鼠标事件
        if (mouse_event != -1) 
        {
            bool success = false;
            if (mouse_event == 0) 
            {  // 左键设置自身点
                success = set_agent(mouse_click_point);
                auto agent = pixel2qingqing(mouse_click_point);
                cout << "agent:" << mouse_click_point << " | actual: (" << agent.first << ", " << agent.second << ") m" << endl;
                if (!success) std::cout << "无效的自身点！" << std::endl;
            } else if (mouse_event == 1) 
            {  // 右键设置敌人点
                success = set_enemy(mouse_click_point);
                auto enemy = pixel2qingqing(mouse_click_point);
                cout << "enemy:" << mouse_click_point << " | actual: (" << enemy.first << ", " << enemy.second << ") m" << endl;
                if (!success) std::cout << "无效的敌人点！" << std::endl;
            }

            if (is_agent_set && is_enemy_set) 
            {
                cv::Point target_px = find_visible_target(agent, enemy);
                auto target = pixel2qingqing(target_px);
                cout << "Target Pixel: " << target_px << endl;
                cout << "Target World: (" << target.first << ", " << target.second << ") m" << endl;
            }

            mouse_event = -1;  // 重置事件
        }

        // 等待按键
        int key = waitKey(10);
        if (key == 27) break; // ESC键退出
    }
}
void relocator::visible_visual(const cv::Point& agent, const cv::Point& enemy)
{
    // 创建窗口
    namedWindow("Interactive Map", WINDOW_NORMAL);
    Mat visual_map = visualize_points(agent, enemy);
    // 等待按键
    while (true) 
    {
        int key = waitKey(10);
        if (key == 27) break; // ESC键退出
    }
}
