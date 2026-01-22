import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 静态 TF 发布 (lidar -> livox_frame, 旋转 -45 度)
        # 如果你已经在别的地方启动了这个，可以注释掉下面这部分
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '-0.785', 'lidar', 'livox_frame'],
        ),

        # 2. 点云转换节点
        Node(
            package='pointcloud_transform',
            executable='transformer_node',
            name='pc_transformer',
            output='screen',
            parameters=[{
                'input_topic': '/livox/lidar',     # 修改为你雷达的实际 Topic
                'output_topic': '/lidar/points',   # 转换后的 Topic，发给 Patchwork++ 用
                'target_frame': 'lidar'            # 目标坐标系（Z 轴朝上的那个）
                'min_height': -1.0,  # 可以根据需要添加
                'max_height': 2.0
            }]
        )
    ])