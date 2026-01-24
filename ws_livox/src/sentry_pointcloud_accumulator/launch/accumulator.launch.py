import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sentry_pointcloud_accumulator'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='sentry_pointcloud_accumulator',
            executable='cloud_accumulator_node',
            name='cloud_accumulator',
            output='screen',
            parameters=[{
                'input_topic': '/terrain_map',        # 输入点云话题
                'output_topic': '/local_map/accumulated', # 输出给 Nav2 的话题
                'fixed_frame': 'odom',               # 累积的参考系
                'window_time': 2.0,                  # 保留时间 (秒)
                'leaf_size': 0.1                     # 降采样网格 (米)
            }]
        )
    ])