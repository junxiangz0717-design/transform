import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ================= 1. 基础配置路径获取 =================
    package_name = 'fast_lio'
    pkg_share = FindPackageShare(package_name)
    livox_driver_share = FindPackageShare('livox_ros_driver2')

    # 获取 yaml 配置文件路径 (对应你 XML 中的 <param from...>)
    config_file_path = PathJoinSubstitution([pkg_share, 'config', 'mid360.yaml'])
    
    # 获取 rviz 配置文件路径
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz_cfg', 'loam_livox.rviz'])

    # 获取绘图脚本路径 (确保你修改了 CMakeLists.txt 安装了 scripts)
    plot_script_path = PathJoinSubstitution([pkg_share, 'scripts', 'auto_plot.py'])

    # ================= 2. 定义参数 (Arg) =================
    # 对应 <arg name="rviz" default="true" />
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    # ================= 3. 包含其他 Launch (Include) =================
    # 对应 <include file="$(find-pkg-share livox_ros_driver2)..." />
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([livox_driver_share, 'launch_ROS2', 'msg_MID360_launch.py'])
        ])
    )

    # ================= 4. 定义定位节点 (Node) =================
    mapping_node = Node(
        package=package_name,
        executable='fastlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            config_file_path,  # 加载 yaml 文件
            {   # 对应你 XML 中的手动 param 覆盖
                'feature_extract_enable': False,
                'point_filter_num': 3,
                'max_iteration': 3,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False
            }
        ]
    )

    # ================= 5. 定义 RViz 节点 =================
    # 对应 <group if="$(var rviz)"> ...
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # ================= 6. [核心] 退出时自动绘图逻辑 =================
    plot_on_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mapping_node, # 监听 laserMapping 节点
            on_exit=[
                LogInfo(msg='=============== 节点已退出，正在生成分析图表 ==============='),
                ExecuteProcess(
                    cmd=['/usr/bin/python3', plot_script_path],
                    output='screen'
                )
            ]
        )
    )

    # ================= 7. 返回描述 =================
    return LaunchDescription([
        rviz_arg,
        livox_driver,
        mapping_node,
        rviz_node,
        plot_on_exit # 加入自动绘图功能
    ])