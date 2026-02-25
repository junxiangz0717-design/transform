import launch
import launch_ros

def generate_launch_description():
    
    # 地面分割节点
    terrain_analysis_node=launch_ros.actions.Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        name='terrain_analysis',
        output='screen',
        remappings=[
            ('/state_estimation', '/odin1/odometry'),  
            ('/registered_scan', '/odin1/cloud_slam')
        ],
        parameters=[
            {'scanVoxelSize': 0.05},
            {'decayTime': 2.8 }, #2.0
            {'noDecayDis': 1.0},
            {'clearingDis': 8.0},
            {'useSorting': True},
            {'quantileZ': 0.10}, #0.25
            {'considerDrop': True},
            {'limitGroundLift': False},
            {'maxGroundLift': 0.15},
            {'clearDyObs': False},
            {'minDyObsDis': 0.3},
            {'minDyObsAngle': 0.0},
            {'minDyObsRelZ': -0.5},
            {'absDyObsRelZThre': 0.2},
            {'minDyObsVFOV': -16.0},
            {'maxDyObsVFOV': 16.0},
            {'minDyObsPointNum': 1},
            {'noDataObstacle': False},
            {'noDataBlockSkipNum': 0},
            {'minBlockPointNum': 10},
            {'vehicleHeight': 1.0},
            {'minObstacleHeight': 0.12},
            {'voxelPointUpdateThre': 50},
            {'voxelTimeUpdateThre': 1.0},
            {'minRelZ': -1.0},
            {'maxRelZ': 1.0},
            {'minScanDis': 0.85},
            {'disRatioZ': 0.2}
        ]
    )

    return launch.LaunchDescription([
        terrain_analysis_node
    ])