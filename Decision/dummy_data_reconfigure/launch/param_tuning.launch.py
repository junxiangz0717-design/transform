from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dummy_data_reconfigure',
            executable='autoaim_config',
            name='dummy_autoaim',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='dummy_data_reconfigure',
            executable='serial_config',
            name='dummy_serial',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_reconfigure',
            output='screen'
        )
    ])
