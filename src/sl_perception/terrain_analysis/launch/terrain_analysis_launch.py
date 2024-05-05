from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    terrainAnalysis_node = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        name='terrainAnalysis',
        namespace=namespace,
        output='screen',
        parameters=[{
            'scanVoxelSize': 0.05,
            'decayTime': 2.0,
            'noDecayDis': 2.0,
            'clearingDis': 4.0,
            'useSorting': True,
            'quantileZ': 0.25,
            'considerDrop': False,
            'limitGroundLift': False,
            'maxGroundLift': 0.15,
            'clearDyObs': True,
            'minDyObsDis': 0.3,
            'minDyObsAngle': 0.0,
            'minDyObsRelZ': -0.5,
            'absDyObsRelZThre': 0.2,
            'minDyObsVFOV': -16.0,
            'maxDyObsVFOV': 16.0,
            'minDyObsPointNum': 1,
            'noDataObstacle': False,
            'noDataBlockSkipNum': 0,
            'minBlockPointNum': 10,
            'vehicleHeight': 0.15,
            'voxelPointUpdateThre': 100,
            'voxelTimeUpdateThre': 2.0,
            'minRelZ': -0.15,
            'maxRelZ': 0.15,
            'disRatioZ': 0.2
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(terrainAnalysis_node)

    return ld