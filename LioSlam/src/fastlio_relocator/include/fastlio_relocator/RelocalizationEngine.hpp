#ifndef FASTLIO_RELOCATOR_RELOCALIZATION_ENGINE_HPP_
#define FASTLIO_RELOCATOR_RELOCALIZATION_ENGINE_HPP_
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

// 引入我们上一节设计的极简重定位引擎头文件 (假设名为 RelocalizationEngine.hpp)
#include "RelocalizationEngine.hpp" 
#include <iostream>
#include <functional>
#include <memory>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 为了演示，引入 PCL 常用的定位算法库
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

// 统一使用的点云类型
using PointT = pcl::PointXYZ;
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

class RelocalizationEngine {
public:
    // ==========================================
    // 【3个外界输入的成员 (Inputs)】
    // ==========================================
    PointCloudPtr map_cloud;
    PointCloudPtr scan_cloud;
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();

    // ==========================================
    // 【1个输出信息的成员 (Output)】
    // ==========================================
    Eigen::Matrix4f final_transform = Eigen::Matrix4f::Identity();

    // ==========================================
    // 【万能算法插槽 (Plugs)】
    // 定义标准的函数签名：输入(源点云, 目标点云, 初始位姿) -> 输出(新位姿)
    // ==========================================
    using SolverFunction = std::function<Eigen::Matrix4f(PointCloudPtr, PointCloudPtr, Eigen::Matrix4f)>;
    
    SolverFunction coarse_solver_ = nullptr;
    SolverFunction fine_solver_ = nullptr;

    // ==========================================
    // 🌟 核心黑魔法：内部模板适配器 (Template Adapters)
    // 只要是 PCL 体系内的算法，不管你是 NDT 还是 ICP，统统直接吞入！
    // ==========================================
    template<typename RegistrationAlgoPtr>
    void setCoarseSolver(RegistrationAlgoPtr algo) {
        // 利用 Lambda 捕获传入的算法对象，并在内部抹平接口差异
        coarse_solver_ = [algo](PointCloudPtr src, PointCloudPtr tgt, Eigen::Matrix4f guess) -> Eigen::Matrix4f {
            algo->setInputSource(src);
            algo->setInputTarget(tgt);
            pcl::PointCloud<PointT> dummy_output;
            algo->align(dummy_output, guess); // 执行算法
            return algo->getFinalTransformation();
        };
    }

    template<typename RegistrationAlgoPtr>
    void setFineSolver(RegistrationAlgoPtr algo) {
        fine_solver_ = [algo](PointCloudPtr src, PointCloudPtr tgt, Eigen::Matrix4f guess) -> Eigen::Matrix4f {
            algo->setInputSource(src);
            algo->setInputTarget(tgt);
            pcl::PointCloud<PointT> dummy_output;
            algo->align(dummy_output, guess);
            return algo->getFinalTransformation();
        };
    }

    // ==========================================
    // 引擎执行方法 (内部封闭逻辑)
    // ==========================================
    Eigen::Matrix4f execute() {
        if (!map_cloud || !scan_cloud) {
            std::cerr << "Error: Map or Scan cloud is empty!" << std::endl;
            return initial_guess;
        }

        Eigen::Matrix4f current_pose = initial_guess;

        // 阶段 1：粗定位
        if (coarse_solver_) {
            current_pose = coarse_solver_(scan_cloud, map_cloud, current_pose);
        }

        // 阶段 2：细定位
        if (fine_solver_) {
            current_pose = fine_solver_(scan_cloud, map_cloud, current_pose);
        }

        final_transform = current_pose;
        return final_transform;
    }
};

#endif // FASTLIO_RELOCATOR_RELOCALIZATION_ENGINE_HPP_