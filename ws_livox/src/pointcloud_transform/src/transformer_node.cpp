#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp" // 关键头文件：用于点云变换
#include "geometry_msgs/msg/transform_stamped.hpp"

// PCL 库头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/pass_through.h>

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("pointcloud_transformer_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("input_topic", "/livox/lidar");
        this->declare_parameter<std::string>("output_topic", "/lidar/points");
        this->declare_parameter<std::string>("target_frame", "lidar");
        this->declare_parameter<double>("min_height", -0.5);
        this->declare_parameter<double>("max_height", 1.5);
        // 获取参数
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();

        RCLCPP_INFO(this->get_logger(), "Listening to: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Transforming to frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Height Filter: [%.2f, %.2f] meters", min_height_, max_height_);

        // 初始化 TF 监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建发布者和订阅者
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10,
            std::bind(&PointCloudTransformer::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud2 cloud_transformed;
        geometry_msgs::msg::TransformStamped transform_stamped;

        try
        {
            // 1. 先进行坐标变换 (ROS 方式)
            if (msg->header.frame_id == target_frame_) {
                cloud_transformed = *msg;
            } else {
                transform_stamped = tf_buffer_->lookupTransform(
                    target_frame_,
                    msg->header.frame_id,
                    rclcpp::Time(0));
                tf2::doTransform(*msg, cloud_transformed, transform_stamped);
            }

            // 2. ROS msg -> PCL PointCloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(cloud_transformed, *pcl_cloud);

            // 3. PCL 过滤 (PassThrough)
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(pcl_cloud);
            pass.setFilterFieldName("z");           // 设置过滤字段为 Z 轴
            pass.setFilterLimits(min_height_, max_height_); // 设置范围
            // pass.setFilterLimitsNegative(true);  // 如果想保留范围*外*的点，开启此行

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            pass.filter(*cloud_filtered);

            // 4. PCL PointCloud -> ROS msg
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*cloud_filtered, output_msg);
            
            // 修正 header (pcl转换后可能会丢失部分 header 信息，安全起见重新赋值)
            output_msg.header.frame_id = target_frame_;
            output_msg.header.stamp = cloud_transformed.header.stamp;

            publisher_->publish(output_msg);
        }
        catch (tf2::TransformException &ex)
        {
            // 降低日志频率，避免刷屏
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "Could not transform point cloud: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
    double min_height_;
    double max_height_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTransformer>());
    rclcpp::shutdown();
    return 0;
}