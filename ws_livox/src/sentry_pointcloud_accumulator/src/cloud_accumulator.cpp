/**
 * @file cloud_accumulator.cpp
 * @brief ROS 2 Humble 版本：基于里程计的滑动窗口点云累积器
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp> 
#include <deque>
#include <memory>

class CloudAccumulator : public rclcpp::Node {
public:
    CloudAccumulator() : Node("cloud_accumulator_node") {
        // 1. 声明并获取参数
        this->declare_parameter<std::string>("input_topic", "/terrain_map");
        this->declare_parameter<std::string>("output_topic", "/local_map/accumulated");
        this->declare_parameter<std::string>("fixed_frame", "odom");
        this->declare_parameter<double>("window_time", 2.0);
        this->declare_parameter<double>("leaf_size", 0.1);

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        fixed_frame_ = this->get_parameter("fixed_frame").as_string();
        window_time_ = this->get_parameter("window_time").as_double();
        leaf_size_ = this->get_parameter("leaf_size").as_double();

        // 2. 初始化 TF 监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. 订阅与发布
        // 使用 SensorDataQoS (Best Effort) 或 SystemDefault (Reliable) 视上游而定，这里用 SystemDefault
        rclcpp::QoS qos(10);
        
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos, std::bind(&CloudAccumulator::cloudCb, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

        RCLCPP_INFO(this->get_logger(), "Cloud Accumulator (ROS 2) Started. Window: %.1fs, Frame: %s", 
                    window_time_, fixed_frame_.c_str());
    }

private:
    std::string input_topic_, output_topic_, fixed_frame_;
    double window_time_, leaf_size_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    struct StoredCloud {
        rclcpp::Time timestamp;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    };
    std::deque<StoredCloud> cloud_queue_;

    void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. ROS 2 消息转 PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *current_cloud);

        // 2. 获取 TF 变换
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            // 尝试查询对应时间的变换
            if (!tf_buffer_->canTransform(fixed_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1))) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF: %s -> %s", 
                    msg->header.frame_id.c_str(), fixed_frame_.c_str());
                return;
            }
            transform_stamped = tf_buffer_->lookupTransform(fixed_frame_, msg->header.frame_id, msg->header.stamp);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            return;
        }

        // 3. 坐标转换
        Eigen::Affine3d affine = tf2::transformToEigen(transform_stamped.transform);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*current_cloud, *transformed_cloud, affine);

        // 4. 存入队列
        StoredCloud sc;
        sc.timestamp = msg->header.stamp;
        sc.cloud = transformed_cloud;
        cloud_queue_.push_back(sc);

        // 5. 滑动窗口清理 (Pruning)
        rclcpp::Time current_time = this->get_clock()->now();
        // 注意：这里使用当前系统时间来判断过期，如果仿真请确保 use_sim_time 为 true
        while (!cloud_queue_.empty()) {
            double age = (current_time - cloud_queue_.front().timestamp).seconds();
            if (age > window_time_) {
                cloud_queue_.pop_front();
            } else {
                break;
            }
        }

        // 6. 累积与滤波
        pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& item : cloud_queue_) {
            *accumulated_cloud += *item.cloud;
        }

        // 降采样 (VoxelGrid)
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(accumulated_cloud);
        sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        sor.filter(*cloud_filtered);

        // 7. 发布
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header.frame_id = fixed_frame_; // odom
        output_msg.header.stamp = this->get_clock()->now(); 
        pub_->publish(output_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudAccumulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}