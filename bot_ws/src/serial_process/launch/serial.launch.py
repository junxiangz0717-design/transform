from launch import LaunchDescription
import launch_ros

def generate_launch_description():
    return LaunchDescription([
        # 启动 send_process 节点
        launch_ros.actions.Node(
            package='serial_process',
            executable='send_process',
            name='send_process',
            output='screen',
            respawn=True, 
        )
    ])