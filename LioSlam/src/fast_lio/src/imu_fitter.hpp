#ifndef IMU_FITTER_HPP
#define IMU_FITTER_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <numeric>

struct MotionCoeffs {
    Eigen::Vector3d w_c0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_c1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_c0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_c1 = Eigen::Vector3d::Zero();
    double t_start = 0.0;
    bool valid = false;
    
    // 拟合均方根误差 (RMSE)，用于衡量"这次拟合有多烂"
    // 如果 RMSE 很大，说明是非线性的剧烈冲击，需膨胀 Q 阵
    double fit_error_acc = 0.0;
    double fit_error_gyr = 0.0;
};

class RobustImuFitter {
public:
    struct Config {
        int max_iter = 4;
        
        // 基础阈值 (Huber Loss 的拐点)
        double acc_delta_base = 3.0; // m/s^2
        double gyr_delta_base = 0.5; // rad/s
        
        // [新增] 轴向敏感度权重 (1.0 = 正常, 0.1 = 极度迟钝/强行平滑)
        // 策略：对 Z 轴 (索引2) 施加极强的抑制，忽略减速带颠簸
        // X, Y 轴保持高灵敏度，捕捉加减速和转弯
        Eigen::Vector3d acc_axis_weight = Eigen::Vector3d(1.0, 1.0, 0.8); 
        
        // [新增] 自适应增益 (Gain)
        // 阈值 = Base + Gain * |当前平均值|
        // 用于在剧烈旋转（高向心加速度）时自动放宽阈值，防止误杀真实信号
        double acc_adaptive_gain = 0.3; 
        double gyr_adaptive_gain = 0.2;

        // [新增] 绝对撞击阈值 (Impact Threshold)
        // 超过这个值的加速度直接视为物理撞击，不参与拟合，防止拉偏直线
        double impact_threshold = 20.0; 
    } config_;

    RobustImuFitter() {}

    // Huber 权重计算函数
    double get_huber_weight(double residual, double delta) {
        double abs_r = std::abs(residual);
        return (abs_r <= delta) ? 1.0 : 0.1 * (delta / abs_r) / abs_r; // 远处的点权重衰减极快
    }

    // 计算均方根误差 (RMSE)
    double calc_rmse(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::Vector2d& x) {
        Eigen::VectorXd r = b - A * x;
        return std::sqrt(r.squaredNorm() / r.size());
    }

    // 自适应一维拟合核心逻辑
    // mean_val: 当前这一段数据的平均绝对值（衡量运动强度）
    // out_rmse: 输出拟合误差
    Eigen::Vector2d fit_1d_adaptive(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, 
                                   double base_delta, double adaptive_gain, 
                                   double mean_val, double& out_rmse) {
        int N = A.rows();
        Eigen::VectorXd W = Eigen::VectorXd::Ones(N);
        Eigen::Vector2d x = A.colPivHouseholderQr().solve(b); // 初始最小二乘解

        // [核心] 自适应阈值计算
        // 动态阈值 = 基础值 * 轴权重 + 增益 * 当前运动强度
        // 注意：这里 base_delta 已经在外部乘过轴权重了
        double dynamic_delta = base_delta + adaptive_gain * mean_val;

        // 迭代加权最小二乘 (IRLS)
        for (int iter = 0; iter < config_.max_iter; ++iter) {
            Eigen::VectorXd residuals = b - A * x;
            
            for (int i = 0; i < N; ++i) {
                // 如果残差过大（比如撞击尖峰），Huber 权重会接近 0，相当于剔除该点
                W(i) = get_huber_weight(residuals(i), dynamic_delta);
            }
            
            // 应用权重
            Eigen::MatrixXd A_weighted = A;
            Eigen::VectorXd b_weighted = b;
            for(int i=0; i<N; ++i) {
                double w_sqrt = std::sqrt(W(i));
                A_weighted.row(i) *= w_sqrt;
                b_weighted(i) *= w_sqrt;
            }
            
            // 求解加权后的方程
            x = A_weighted.colPivHouseholderQr().solve(b_weighted);
        }
        
        // 计算最终的拟合误差 (RMSE)，这个值稍后会传给 EKF 用于判断是否要信 IMU
        out_rmse = calc_rmse(A, b, x);
        return x;
    }

    MotionCoeffs fit(const std::vector<double>& times, 
                     const std::vector<Eigen::Vector3d>& gyros,
                     const std::vector<Eigen::Vector3d>& accs) {
        MotionCoeffs coeffs;
        int N = times.size();
        if (N < 5) { coeffs.valid = false; return coeffs; }

        coeffs.t_start = times.front();
        coeffs.valid = true;

        // 构建时间矩阵 A = [1, t]
        Eigen::MatrixXd A(N, 2);
        for (int i = 0; i < N; ++i) A(i, 0) = 1.0, A(i, 1) = times[i] - coeffs.t_start;

        // 计算运动强度平均值 (用于自适应增益)
        Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyr_mean = Eigen::Vector3d::Zero();
        for(int i=0; i<N; ++i) {
            acc_mean += accs[i].cwiseAbs();
            gyr_mean += gyros[i].cwiseAbs();
        }
        acc_mean /= N;
        gyr_mean /= N;

        double total_rmse_acc = 0;
        double total_rmse_gyr = 0;

        // 逐轴拟合
        for (int i = 0; i < 3; ++i) {
            Eigen::VectorXd b_acc(N), b_gyr(N);
            
            // 数据拷贝与预过滤
            for(int k=0; k<N; ++k) {
                // [撞击保护] 如果数值超过绝对物理极限，截断它，防止拉偏初始解
                double raw_acc = accs[k](i);
                if (std::abs(raw_acc) > config_.impact_threshold) {
                    raw_acc = (raw_acc > 0 ? 1.0 : -1.0) * config_.impact_threshold;
                }
                b_acc(k) = raw_acc;
                b_gyr(k) = gyros[k](i);
            }

            double rmse_a, rmse_g;
            
            // [关键修改] 应用轴向权重 (Axis-Decoupled Cleansing)
            // 对于 Z 轴 (i=2)，acc_axis_weight 为 0.1
            // 这会导致 base_delta 变得极小，使得 Z 轴上的颠簸(减速带)更容易触发 Huber 降权
            // 从而强行拟合出一条"无视颠簸"的平滑直线
            double axis_base_delta = config_.acc_delta_base * config_.acc_axis_weight(i);
            
            // 拟合加速度
            Eigen::Vector2d res_acc = fit_1d_adaptive(A, b_acc, axis_base_delta, config_.acc_adaptive_gain, acc_mean(i), rmse_a);
            coeffs.a_c0(i) = res_acc(0);
            coeffs.a_c1(i) = res_acc(1);
            total_rmse_acc += rmse_a;

            // 拟合角速度 (角速度通常不需要轴向解耦，保持默认)
            Eigen::Vector2d res_gyr = fit_1d_adaptive(A, b_gyr, config_.gyr_delta_base, config_.gyr_adaptive_gain, gyr_mean(i), rmse_g);
            coeffs.w_c0(i) = res_gyr(0);
            coeffs.w_c1(i) = res_gyr(1);
            total_rmse_gyr += rmse_g;
        }
        
        coeffs.fit_error_acc = total_rmse_acc / 3.0;
        coeffs.fit_error_gyr = total_rmse_gyr / 3.0;
        if (coeffs.fit_error_acc > 2.0) {
            coeffs.valid = false;
        }
        return coeffs;
    }
    
    // 获取任意时刻的预测值
    void predict(const MotionCoeffs& c, double t, Eigen::Vector3d& out_acc, Eigen::Vector3d& out_gyr) {
        if (!c.valid) return;
        double dt = t - c.t_start;
        out_acc = c.a_c0 + c.a_c1 * dt;
        out_gyr = c.w_c0 + c.w_c1 * dt;
    }
};

#endif