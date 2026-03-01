#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include "UniversalLogger.hpp"

class FMTRelocalizer {
public:
    struct RelocResult {
        double x, y, yaw_deg; 
        double score;         
        bool success;
        double uncertainty; // 估计的方差
    };

    FMTRelocalizer() = default;

    void SetInput(const cv::Mat& scan, const cv::Mat& map, double time) {
        current_scan_ = scan;
        current_map_  = map;
        current_time_ = time;
    }

    RelocResult Solve(double resolution) {
        ULOG_SET_TIME(current_time_);
        RelocResult final_res = {0, 0, 0, -1.0, false, 100.0};

        if (current_scan_.empty() || current_map_.empty()) return final_res;

        // [Step 1] 预处理 (不变) ...
        cv::Mat scan_L0 = PreprocessImage(current_scan_, false); 
        cv::Mat map_L0  = PreprocessImage(current_map_, false);
        cv::Mat scan_L1, map_L1;
        cv::pyrDown(scan_L0, scan_L1); cv::pyrDown(map_L0, map_L1);
        cv::dilate(scan_L1, scan_L1, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        cv::dilate(map_L1, map_L1, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        if (cv::countNonZero(scan_L1) < 10) return final_res;

        // [Step 2] 粗定位 (核心修改：获取第二高分)
        double second_score = 0.0;
        // 注意：传入 resolution * 2.0
        RelocResult coarse_res = GridSearch(scan_L1, map_L1, resolution * 2.0, 0.0, 180.0, 4.0, second_score); 
        
        ULOG_PLOT("Reloc_Result", "coarse_score", coarse_res.score);
        ULOG_PLOT("Reloc_Result", "second_score", second_score); // 记录第二高分

        if (coarse_res.score < 0.35) return final_res; 

        // ================= [新增：区分度检查 (Ambiguity Check)] =================
        // 计算区分度比率 (Ratio Test)
        // 如果 second 接近 best，ratio 接近 1.0 (区分度差)
        // 如果 second 很小，ratio 接近 0.0 (区分度好)
        double ambiguity_ratio = 0.0;
        if (coarse_res.score > 1e-6) {
            ambiguity_ratio = second_score / coarse_res.score;
        }
        
        // 如果两个峰值太接近（例如第二名也是 0.85，第一名 0.9），说明环境重复纹理严重
        // 此时我们应该极大增加不确定度，甚至直接判负
        // 阈值：Lowe's Ratio Test 通常取 0.7-0.8
        if (ambiguity_ratio > 0.9) { 
            // 严重混淆！
            final_res.score = coarse_res.score; // 记录分数但标记失败
            final_res.success = false;
            final_res.uncertainty = 100.0;
            // 可以在这里直接 return，或者留给后面加权处理
            return final_res; 
        }

        // [Step 3] 细定位 (GridSearch 签名变了，这里也要传一个 dummy 变量)
        cv::Mat scan_L0_rot;
        RotateImage(scan_L0, scan_L0_rot, coarse_res.yaw_deg);
        double dummy_score;
        RelocResult fine_res = GridSearch(scan_L0_rot, map_L0, resolution, 0.0, 8.0, 0.5, dummy_score);

        // [Step 4] 结果合成 ... (不变)
        final_res = fine_res;
        final_res.yaw_deg = coarse_res.yaw_deg + fine_res.yaw_deg;
        double rad = coarse_res.yaw_deg * M_PI / 180.0;
        double x_world = fine_res.x * std::cos(rad) - fine_res.y * std::sin(rad);
        double y_world = fine_res.x * std::sin(rad) + fine_res.y * std::cos(rad);
        final_res.x = coarse_res.x + x_world;
        final_res.y = coarse_res.y + y_world;
        if (final_res.yaw_deg > 180) final_res.yaw_deg -= 360;
        if (final_res.yaw_deg <= -180) final_res.yaw_deg += 360;

        // [Step 5] 综合评分与不确定度 (融入区分度)
        ULOG_PLOT("Reloc_Result", "best_score", final_res.score);
        
        // 综合质量 = 原始分数 * (1 - 区分度风险)
        // 如果 ambiguity_ratio = 0.9 (很像), distinctiveness = 0.1, 质量大打折扣
        // 采用 Sigmoid 进行平滑截断
        double distinctiveness = 1.0 - std::pow(ambiguity_ratio, 4.0); // 4次幂让高 Ratio 迅速衰减
        
        // 我们可以把这个 distinctiveness 乘到 score 里，骗过后面的逻辑
        // 或者显式地增加 uncertainty
        
        double final_quality = final_res.score * distinctiveness;
        
        if (final_quality > 0.35) final_res.success = true;
        else final_res.success = false;

        // 重新设计不确定度公式：
        // 分数越高 -> 不确定度越低
        // 区分度越差 (ratio越大) -> 不确定度越高
        final_res.uncertainty = 2.0 * std::exp(-8.0 * (final_res.score - 0.4));
        
        // 惩罚项：如果区分度不好，指数级放大不确定度
        if (ambiguity_ratio > 0.6) {
            final_res.uncertainty *= (1.0 + 10.0 * (ambiguity_ratio - 0.6));
        }
        
        // 将修改后的 "等效质量分" 存回去，方便外层 Alpha 计算
        // 这样外层的 CalculateDualLayerAlpha 里的 w_qual 就会自动变小
        final_res.score = final_quality; 

        return final_res;
    }
private:
    cv::Mat current_scan_, current_map_;
    double  current_time_ = 0.0;

// 修改 GridSearch 函数签名，增加一个输出参数 second_score
    RelocResult GridSearch(const cv::Mat& scan, const cv::Mat& map, double res_val, 
                           double center_yaw, double range, double step, 
                           double& out_second_score) // [新增参数]
    {
        RelocResult res = {0,0,0, -1.0, false, 100.0};
        out_second_score = 0.0; // 初始化

        int w = scan.cols; int h = scan.rows;
        int cx = w/2, cy = h/2;
        int crop = std::min(w,h)*0.65;
        int offset = crop / 2;
        
        double global_best_score = -1.0;
        double global_second_score = -1.0; // 记录全局第二高分
        cv::Point global_best_loc; 
        cv::Mat global_best_res_map;
        double global_best_yaw = 0.0;

        for (double dy = -range; dy <= range; dy += step) {
            double yaw = center_yaw + dy;
            cv::Mat rot; RotateImage(scan, rot, yaw);
            cv::Mat templ = rot(cv::Rect(cx-offset, cy-offset, crop, crop));
            if (templ.cols > map.cols || templ.rows > map.rows) continue;
            
            cv::Mat res_map;
            cv::matchTemplate(map, templ, res_map, cv::TM_CCOEFF_NORMED);
            
            double minv, maxv; cv::Point minl, maxl;
            cv::minMaxLoc(res_map, &minv, &maxv, &minl, &maxl);

            // [核心修改]：不仅记录第一，还要尝试找这一张图里的第二
            // 为了简化计算，我们假设不同角度之间的局部极大值竞争由外层循环处理
            // 但更严谨的做法是：先记录每一层的 maxv，最后在所有层的结果中找 Top 1 和 Top 2
            
            if (maxv > global_best_score) {
                // 原来的第一名变成了第二名 (简单的冒泡逻辑，不够严谨但够快)
                // 注意：这种简单的冒泡无法处理 "最佳yaw" 和 "次佳yaw" 的竞争
                // 如果 yaw=0度 score=0.9, yaw=1度 score=0.89，这不算误匹配，这是峰宽。
                // 我们真正怕的是：yaw=0度, pos=(10,10) score=0.9
                //              yaw=0度, pos=(50,50) score=0.89
                
                // 所以我们要在 *当前的 res_map* 里找第二峰值
                // 1. 临时屏蔽最大值附近 (例如 2米范围内)
                cv::Mat mask = cv::Mat::ones(res_map.size(), CV_8UC1);
                int suppression_r = 2.0 / res_val; // 屏蔽半径 2米
                cv::circle(mask, maxl, suppression_r, cv::Scalar(0), -1);
                
                double sec_minv, sec_maxv; cv::Point sec_minl, sec_maxl;
                // 在屏蔽后的图中找最大值，即为第二峰值
                cv::minMaxLoc(res_map, &sec_minv, &sec_maxv, &sec_minl, &sec_maxl, mask);
                
                // 更新全局状态
                global_second_score = std::max(global_second_score, sec_maxv); // 之前帧的best可能是现在的second
                if (global_best_score > 0) global_second_score = std::max(global_second_score, global_best_score);

                global_best_score = maxv;
                global_best_yaw = yaw;
                global_best_loc = maxl;
                global_best_res_map = res_map;
            } else {
                // 当前帧的 maxv 可能是全局第二
                global_second_score = std::max(global_second_score, maxv);
            }
        }

        // 填充结果
        if (global_best_score > 0) {
            res.score = global_best_score;
            res.yaw_deg = global_best_yaw;
            res.x = (global_best_loc.x + offset - cx) * res_val;
            res.y = (global_best_loc.y + offset - cy) * res_val;
            
            // 输出第二高分
            out_second_score = global_second_score;
        }

        // 亚像素优化 (保持不变)
        if (global_best_score > 0.3 && !global_best_res_map.empty()) {
            double sx=0, sy=0;
            SubPixelPeak(global_best_res_map, global_best_loc, sx, sy);
            res.x += sx * res_val; res.y += sy * res_val;
        }
        return res;
    }
    cv::Mat PreprocessImage(const cv::Mat& src, bool use_dilate) {
        cv::Mat dst;
        if (src.type() != CV_8UC1) src.convertTo(dst, CV_8UC1); else src.copyTo(dst);
        
        // 二值化：过滤噪声
        cv::threshold(dst, dst, 10, 255, cv::THRESH_BINARY);
        
        // 粗定位使用膨胀
        if (use_dilate) {
            cv::dilate(dst, dst, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        } else {
            // [新增] 细定位使用微量模糊 (3x3, Sigma=0.5)
            // 这能让细定位的能量峰稍微宽一点，接住粗定位的结果，避免匹配分归零
            cv::GaussianBlur(dst, dst, cv::Size(5, 5), 1.5);
        }
        
        return dst;
    }
    void RotateImage(const cv::Mat& src, cv::Mat& dst, double angle) {
        cv::Point2f pt(src.cols/2.0f, src.rows/2.0f);
        cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
        cv::warpAffine(src, dst, r, src.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
    }

    void SubPixelPeak(const cv::Mat& r, const cv::Point& p, double& sx, double& sy) {
        if (p.x<1 || p.x>=r.cols-1 || p.y<1 || p.y>=r.rows-1) return;
        float c0 = r.at<float>(p.y, p.x-1); float c2 = r.at<float>(p.y, p.x+1);
        float r0 = r.at<float>(p.y-1, p.x); float r2 = r.at<float>(p.y+1, p.x);
        float c1 = r.at<float>(p.y, p.x);
        if (std::abs(c0-2*c1+c2)>1e-5) sx = 0.5*(c0-c2)/(c0-2*c1+c2);
        if (std::abs(r0-2*c1+r2)>1e-5) sy = 0.5*(r0-r2)/(r0-2*c1+r2);
        sx = std::max(-0.7, std::min(0.7, sx)); sy = std::max(-0.7, std::min(0.7, sy));
    }
};