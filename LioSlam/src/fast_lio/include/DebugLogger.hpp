#ifndef DEBUG_LOGGER_HPP
#define DEBUG_LOGGER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <mutex>
#include <Eigen/Dense>

class DebugLogger {
private:
    std::ofstream log_file_;
    std::mutex mtx_;
    int frame_count_ = 0;
    bool is_active_ = false;
    
    // [核心设计] 全局时间缓存
    // 所有宏都会读取这个变量，无需在每个函数里定义 current_time
    double current_time_buffer_ = 0.0;

    DebugLogger() {} 

public:
    static DebugLogger& getInstance() {
        static DebugLogger instance;
        return instance;
    }

    // 初始化：确保表头干净，无省略号
    void init(const std::string& path) {
        std::lock_guard<std::mutex> lock(mtx_);
        log_file_.open(path, std::ios::out | std::ios::trunc);
        if (log_file_.is_open()) {
            is_active_ = true;
            log_file_ << "FRAME_ID,STEP_NAME,TIME,DATA_LABEL,VALUE_1,VALUE_2,VALUE_3" << std::endl;
            // std::cout << "[DebugLogger] Log file initialized: " << path << std::endl;
        } else {
            std::cerr << "[DebugLogger] Error: Cannot open file " << path << std::endl;
        }
    }

    // [核心设计] 更新全系统时间
    void updateTime(double t) {
        std::lock_guard<std::mutex> lock(mtx_);
        current_time_buffer_ = t;
    }

    void startNewFrame() {
        std::lock_guard<std::mutex> lock(mtx_);
        frame_count_++;
    }

    // 基础数值记录
    void log_val(const std::string& step, const std::string& label, double v1, double v2 = 0, double v3 = 0) {
        if (!is_active_) return;
        std::lock_guard<std::mutex> lock(mtx_);
        // 格式严格：逗号后无空格，确保 Python 解析准确
        log_file_ << frame_count_ << "," << step << "," 
                  << std::fixed << std::setprecision(6) << current_time_buffer_ << "," 
                  << label << "," << v1 << "," << v2 << "," << v3 << std::endl;
    }

    // =========================================================
    // [新增] 专门用于处理 LOG_MSG 的字符串记录函数
    // =========================================================
    void log_msg(const std::string& step, const std::string& label, const std::string& msg) {
        if (!is_active_) return;
        std::lock_guard<std::mutex> lock(mtx_);
        // 逻辑：将 msg 字符串写入 VALUE_1 的位置，后续两列填 0
        // 这样 CSV 依然保持 7 列结构，不会破坏文件格式
        log_file_ << frame_count_ << "," << step << "," 
                  << std::fixed << std::setprecision(6) << current_time_buffer_ << "," 
                  << label << "," << msg << ",0,0" << std::endl;
    }

    // 向量记录
    void log_vec(const std::string& step, const std::string& label, const Eigen::Vector3d& v) {
        log_val(step, label, v.x(), v.y(), v.z());
    }

    // [核心设计] 状态记录模板
    // 只要传入的结构体有 pos, vel, bg, ba 成员即可，不依赖特定头文件
    template <typename StateType>
    void log_state_t(const std::string& step, const StateType& s) {
        log_vec(step, "STATE_POS", s.pos);
        log_vec(step, "STATE_VEL", s.vel);
        log_vec(step, "STATE_BG", s.bg);
        log_vec(step, "STATE_BA", s.ba);
    }
};

// ==================== 宏定义全集 ====================

// 1. 初始化与帧控制
#define LOG_INIT(path) DebugLogger::getInstance().init(path)
#define LOG_FRAME() DebugLogger::getInstance().startNewFrame()

// 2. 时间同步 (最关键的宏)
#define LOG_UPDATE_TIME(t) DebugLogger::getInstance().updateTime(t)

// 3. 数据记录 (自动使用已同步的时间)
#define LOG_VAL(step, label, val) DebugLogger::getInstance().log_val(step, label, val)
#define LOG_VEC(step, label, vec) DebugLogger::getInstance().log_vec(step, label, vec)
#define LOG_STATE(step, state) DebugLogger::getInstance().log_state_t(step, state)

// [修复] 现在类中有 log_msg 函数了，这个宏可以正常工作了
#define LOG_MSG(step, label, msg) DebugLogger::getInstance().log_msg(step, label, msg)

#endif