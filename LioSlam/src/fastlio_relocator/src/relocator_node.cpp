#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

// 将原来的 #include "RelocalizationEngine.hpp" 替换为：
#include "fastlio_relocator/RelocalizationEngine.hpp"

class FastLioNavInterface : public rclcpp::Node {
public:
    FastLioNavInterface() : Node("fastlio_nav_interface"), has_relocalized_(false) {
        // 1. 声明并获取参数 (全局地图路径)
        this->declare_parameter<std::string>("map_pcd_path", "");
        std::string map_path = this->get_parameter("map_pcd_path").as_string();

        // 2. 初始化重定位引擎并加载地图
        engine_.map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *engine_.map_cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "无法读取全局地图: %s", map_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "全局地图加载成功，点数: %zu", engine_.map_cloud->size());

        // 3. 初始化静态 TF 广播器 (核心：用于发布一次即可永久生效的 map -> camera_init)
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // 4. 订阅 FAST-LIO 的标准话题
        // /Odometry: 获取 camera_init -> body 的实时相对位姿
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&FastLioNavInterface::odom_callback, this, std::placeholders::_1));
        
        // /cloud_registered: 获取当前帧去畸变后的点云 (通常在 body 坐标系下)
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 10, std::bind(&FastLioNavInterface::cloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "接口节点启动，等待 FAST-LIO 数据以执行单次重定位...");
    }

private:
    bool has_relocalized_; // 状态锁：确保只运行一次
    nav_msgs::msg::Odometry::SharedPtr latest_odom_; // 缓存最新里程计
    RelocalizationEngine engine_; // 我们的黑盒引擎
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    // 缓存最新的里程计
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
    }

    // 点云回调：触发重定位逻辑
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 【防御机制】：如果已经重定位成功，直接丢弃后续点云，节省 100% 的 CPU
        if (has_relocalized_) {
            return; 
        }

        // 必须等到有里程计数据才能计算相对关系
        if (!latest_odom_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "等待 /Odometry 数据...");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "收到首帧数据，开始执行黑盒重定位...");

        // 1. 将 ROS 点云转换为 PCL 点云传入引擎
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *scan_cloud);
        engine_.scan_cloud = scan_cloud;

        // 2. 随时挂载成熟的定位算法插头 (例如 ICP)
        auto icp = std::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
        icp->setMaxCorrespondenceDistance(1.0);
        icp->setMaximumIterations(50);
        engine_.setFineSolver(icp); // 外部插拔算法

        // 3. 执行引擎计算，获得 map -> body 的位姿 T_map_body
        Eigen::Matrix4f T_map_body = engine_.execute();

        // 4. 数学计算推导 T_map_camera_init
        // 已知:
        // T_map_body = 引擎配准结果
        // T_camera_init_body = 来自 latest_odom_
        // 推导: T_map_body = T_map_camera_init * T_camera_init_body
        // 结论: T_map_camera_init = T_map_body * T_camera_init_body.inverse()
        
        Eigen::Isometry3f T_camera_init_body = Eigen::Isometry3f::Identity();
        T_camera_init_body.translation() << latest_odom_->pose.pose.position.x, 
                                            latest_odom_->pose.pose.position.y, 
                                            latest_odom_->pose.pose.position.z;
        Eigen::Quaternionf q(latest_odom_->pose.pose.orientation.w, 
                             latest_odom_->pose.pose.orientation.x, 
                             latest_odom_->pose.pose.orientation.y, 
                             latest_odom_->pose.pose.orientation.z);
        T_camera_init_body.rotate(q);

        Eigen::Matrix4f T_map_camera_init = T_map_body * T_camera_init_body.matrix().inverse();

        // 5. 将计算结果转化为 ROS 的 TF 消息并静态发布
        publish_static_tf(T_map_camera_init, msg->header.stamp);

        // 6. 锁死状态机，宣布重定位圆满结束
        has_relocalized_ = true;
        
        // 优雅操作：甚至可以直接取消订阅，彻底释放资源！
        cloud_sub_.reset(); 
        odom_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "重定位成功！静态 TF 已发布，接口节点进入低功耗休眠模式。");
    }

    void publish_static_tf(const Eigen::Matrix4f& transform_matrix, rclcpp::Time stamp) {
        geometry_msgs::msg::TransformStamped t;
        
        // 设置时间戳和坐标系名称
        t.header.stamp = stamp;
        t.header.frame_id = "map";           // 全局固定坐标系
        t.child_frame_id = "camera_init";    // FAST-LIO 的里程计原点坐标系

        // 矩阵提取平移
        t.transform.translation.x = transform_matrix(0, 3);
        t.transform.translation.y = transform_matrix(1, 3);
        t.transform.translation.z = transform_matrix(2, 3);

        // 矩阵提取旋转 (转为四元数)
        Eigen::Matrix3f rotation_matrix = transform_matrix.block<3, 3>(0, 0);
        Eigen::Quaternionf q(rotation_matrix);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // 广播静态 TF！(ROS 2 底层会接管它，即使节点什么都不干，这个 TF 也会一直被 Nav2 读到)
        static_tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLioNavInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}