#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// PCL 库
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

// TF2 库
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

class MultiFrameObstacleFilter : public rclcpp::Node
{
public:
    MultiFrameObstacleFilter() : Node("pointcloud_fusion_node")
    {
        // --- 参数声明与获取 ---
        this->declare_parameter<std::string>("input_topic", "/livox/lidar");
        this->declare_parameter<std::string>("output_topic", "/lidar/points_fused");
        this->declare_parameter<std::string>("odom_topic", "/odin1/odometry_highfreq");
        this->declare_parameter<std::string>("fixed_frame", "odom");
        this->declare_parameter<std::string>("robot_frame", "odin1_base_link");
        this->declare_parameter<double>("min_height", 0.1);    // 相对于地面高度
        this->declare_parameter<double>("max_height", 1.5); 
        this->declare_parameter<double>("min_range", 1.0);     // 距离车体中心的最小半径
        this->declare_parameter<double>("max_range", 10.0);    // 距离车体中心的最大半径
        this->declare_parameter<double>("voxel_size", 0.05);   
        this->declare_parameter<int>("history_frames", 15);    

        fixed_frame_ = this->get_parameter("fixed_frame").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();
        min_range_ = this->get_parameter("min_range").as_double();
        max_range_ = this->get_parameter("max_range").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        history_frames_ = this->get_parameter("history_frames").as_int();

        // --- TF2 初始化 ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- 发布与订阅 ---
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("output_topic").as_string(), 10);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string(), 100,
            std::bind(&MultiFrameObstacleFilter::odom_callback, this, std::placeholders::_1));

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("input_topic").as_string(), rclcpp::SensorDataQoS(),
            std::bind(&MultiFrameObstacleFilter::pointcloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Multi-Frame Obstacle Filter Node Initialized.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        odom_buffer_.push_back(msg);
        if (odom_buffer_.size() > 500) odom_buffer_.pop_front();
    }

    bool get_interpolated_odom(const rclcpp::Time& time, Eigen::Isometry3d& transform)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (odom_buffer_.size() < 2) return false;

        auto it = std::lower_bound(odom_buffer_.begin(), odom_buffer_.end(), time,
            [](const nav_msgs::msg::Odometry::SharedPtr& s, const rclcpp::Time& t) {
                return rclcpp::Time(s->header.stamp) < t;
            });

        if (it == odom_buffer_.end() || it == odom_buffer_.begin()) return false;

        auto msg_after = *it;
        auto msg_before = *(--it);

        double ratio = (time.seconds() - rclcpp::Time(msg_before->header.stamp).seconds()) /
                       (rclcpp::Time(msg_after->header.stamp).seconds() - rclcpp::Time(msg_before->header.stamp).seconds());

        Eigen::Vector3d p0(msg_before->pose.pose.position.x, msg_before->pose.pose.position.y, msg_before->pose.pose.position.z);
        Eigen::Vector3d p1(msg_after->pose.pose.position.x, msg_after->pose.pose.position.y, msg_after->pose.pose.position.z);
        
        Eigen::Quaterniond q0, q1;
        tf2::fromMsg(msg_before->pose.pose.orientation, q0);
        tf2::fromMsg(msg_after->pose.pose.orientation, q1);

        transform = Eigen::Isometry3d::Identity();
        transform.translation() = p0 + ratio * (p1 - p0);
        transform.rotate(q0.slerp(ratio, q1));
        return true;
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1. 获取 Lidar -> Base 静态变换
        Eigen::Isometry3d lidar_to_base;
        try {
            auto tf_stamped = tf_buffer_->lookupTransform(robot_frame_, msg->header.frame_id, 
                                                         msg->header.stamp, tf2::durationFromSec(0.05));
            lidar_to_base = tf2::transformToEigen(tf_stamped);
        } catch (tf2::TransformException &ex) { return; }

        // 2. 获取 Base -> Odom 高频插值变换
        Eigen::Isometry3d base_to_odom;
        if (!get_interpolated_odom(msg->header.stamp, base_to_odom)) return;

        // 3. 转换点云至 Odom 系
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *raw_pcl);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr odom_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        Eigen::Matrix4f T_lidar_to_odom = (base_to_odom * lidar_to_base).matrix().cast<float>();
        pcl::transformPointCloud(*raw_pcl, *odom_pcl, T_lidar_to_odom);

        // 4. 滑动窗口维护
        cloud_buffer_.push_back(odom_pcl);
        if (cloud_buffer_.size() > (size_t)history_frames_) cloud_buffer_.pop_front();

        // 5. 拼接与降采样
        pcl::PointCloud<pcl::PointXYZI>::Ptr combined(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& cloud : cloud_buffer_) *combined += *cloud;

        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(combined);
        sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        sor.filter(*downsampled);

        // 6. 过滤逻辑：高度(绝对高度) & 距离(相对于当前车体中心)
        pcl::PointCloud<pcl::PointXYZI>::Ptr final_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        Eigen::Vector3d current_pos = base_to_odom.translation(); // 机器人在 odom 中的位置

        for (const auto& pt : downsampled->points) {
            // 高度过滤
            if (pt.z < min_height_ || pt.z > max_height_) continue;

            // 距离过滤：以当前车体中心为原点
            double dx = pt.x - current_pos.x();
            double dy = pt.y - current_pos.y();
            double d2 = dx*dx + dy*dy; // 此处使用 2D 距离过滤，如需 3D 请加上 dz*dz

            if (d2 >= min_range_ * min_range_ && d2 <= max_range_ * max_range_) {
                final_pcl->points.push_back(pt);
            }
        }

        // 7. 发布
        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(*final_pcl, out);
        out.header.frame_id = fixed_frame_;
        out.header.stamp = msg->header.stamp;
        publisher_->publish(out);
    }

    // 变量声明
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_buffer_;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_buffer_;
    std::mutex odom_mutex_;
    std::string fixed_frame_, robot_frame_;
    double min_height_, max_height_, min_range_, max_range_, voxel_size_;
    int history_frames_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiFrameObstacleFilter>());
    rclcpp::shutdown();
    return 0;
}