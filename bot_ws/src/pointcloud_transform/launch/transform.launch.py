import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 点云处理节点
        Node(
            package='pointcloud_transform',
            executable='transformer_node',
            name='pc_transformer',
            output='screen',
            parameters=[{
                'input_topic': '/odin1/cloud_raw',     # 修改为你雷达的实际 Topic
                'output_topic': '/pointclouds',   # 转换后的 Topic，发给 Patchwork++ 用
                'min_height': -0.5, 
                'max_height': 1.0, 
                'max_range': 6.0, 
                'min_range': 1.0,
                'voxel_size': 0.1,
                'history_frames':18
            }]
        )
    ])