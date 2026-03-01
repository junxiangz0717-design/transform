#ifndef UNIVERSAL_LOGGER_HPP
#define UNIVERSAL_LOGGER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <iomanip>
#include <Eigen/Dense>
// ==================== [新增] 点云 NaN 深度检查工具 ====================

// 引入必要的 PCL 头文件 (确保你的工程能找到 PCL)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath> // for std::isnan, std::isinf
class UniversalLogger {
private:
    std::ofstream log_file_;
    std::mutex mtx_;
    bool is_initialized_ = false;

    // [核心改变] 只有线程局部时间，没有全局时间了
    // 每个线程（主线程、重定位线程）看到的都是自己的副本，默认是 0
    static thread_local double thread_current_time_;

    UniversalLogger() {}

public:
    static UniversalLogger& getInstance() {
        static UniversalLogger instance;
        return instance;
    }

    void init(const std::string& path) {
        std::lock_guard<std::mutex> lock(mtx_);
        log_file_.open(path, std::ios::out | std::ios::trunc);
        log_file_ << "timestamp,plot_group,series_name,value" << std::endl;
        is_initialized_ = true;
    }

    // [统一接口] 设置当前线程的时间
    // 无论是主线程还是子线程，都调这个
    void setTime(double t) {
        thread_current_time_ = t;
    }
    double getTime() const {
        return thread_current_time_;
    }
    // 写日志
    void write(const std::string& group, const std::string& name, double val) {
        if (!is_initialized_) return;
        std::lock_guard<std::mutex> lock(mtx_);
        // 直接使用当前线程的时间副本
        log_file_ << std::fixed << std::setprecision(6) << thread_current_time_ << ","
                  << group << "," << name << "," << val << std::endl;
    }

    void write(const std::string& group, const std::string& name_prefix, const Eigen::Vector3d& vec) {
        write(group, name_prefix + "_x", vec.x());
        write(group, name_prefix + "_y", vec.y());
        write(group, name_prefix + "_z", vec.z());
    }
};

// 静态成员定义
inline thread_local double UniversalLogger::thread_current_time_ = 0.0;

// ==================== 极简宏定义 ====================

#define ULOG_INIT(path) UniversalLogger::getInstance().init(path)

// [统一时间宏] 无论在哪，只要想更新当前上下文的时间，就调它
#define ULOG_SET_TIME(t) UniversalLogger::getInstance().setTime(t)

// [绘图宏] 保持不变
#define ULOG_PLOT(group, name, val) UniversalLogger::getInstance().write(group, name, val)
// [修改后] 定义一个通用的模板函数，直接接受点云指针类型
template <typename CloudPtrT> 
void CheckPointCloudHealth(const CloudPtrT& cloud, const std::string& stage_name) {
    // 1. 检查指针是否为空
    if (!cloud || cloud->empty()) {
        ULOG_PLOT("Data_Health", stage_name + "_status", -1.0);
        return;
    }

    bool has_nan = false;
    size_t nan_index = 0;
    
    // 2. 遍历检查 (使用 auto 推导点类型，无需知道具体 PointT)
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& p = cloud->points[i]; // 自动推导点的类型
        
        // 检查 X, Y, Z 是否为 NaN 或 Inf
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            has_nan = true;
            nan_index = i;
            break; 
        }
    }

    if (has_nan) {
        ULOG_PLOT("Data_Health", stage_name + "_status", 1.0);
        
        // 打印错误点
        const auto& p = cloud->points[nan_index];
        std::cerr << "\033[1;31m[CRITICAL WARNING] NaN Found in [" << stage_name << "] at Time: " 
                  << UniversalLogger::getInstance().getTime() << "\033[0m" << std::endl;
        std::cerr << "--> Bad Point Index: " << nan_index << std::endl;
        std::cerr << "--> Values: x=" << p.x << ", y=" << p.y << ", z=" << p.z << std::endl;
    } else {
        ULOG_PLOT("Data_Health", stage_name + "_status", 0.0);
    }
}

// 供外部调用的宏
// 用法: ULOG_CHECK_PCL(feats_undistort, "1_Raw_Input");
#define ULOG_CHECK_PCL(cloud_ptr, name) CheckPointCloudHealth(cloud_ptr, name)

#endif // UNIVERSAL_LOGGER_HPP
