from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_checkTerrainConn_cmd = DeclareLaunchArgument(
        'checkTerrainConn',
        default_value='false',
        description='Check terrain connectivity')

    terrainAnalysisExt_node = Node(
        package='terrain_analysis_ext',
        executable='terrainAnalysisExt',
        name='terrainAnalysisExt',
        namespace=namespace,
        output='screen',
        parameters=[{
            'scanVoxelSize': 0.1,
            'decayTime': 10.0,
            'noDecayDis': 0.0,
            'clearingDis': 30.0,
            'useSorting': False,
            'quantileZ': 0.1,
            'vehicleHeight': 1.5,
            'voxelPointUpdateThre': 100,
            'voxelTimeUpdateThre': 2.0,
            'lowerBoundZ': -2.5,
            'upperBoundZ': 1.0,
            'disRatioZ': 0.1,
            'checkTerrainConn': checkTerrainConn,
            'terrainConnThre': 0.5,
            'terrainUnderVehicle': -0.75,
            'ceilingFilteringThre': 2.0,
            'localTerrainMapRadius': 4.0
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_checkTerrainConn_cmd)
    ld.add_action(terrainAnalysisExt_node)

    return ld