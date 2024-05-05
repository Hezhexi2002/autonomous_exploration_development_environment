from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world_name')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='garage',
        description='Name of the world')

    visualizationTools_node = Node(
        package='visualization_tools',
        executable='visualizationTools',
        name='visualizationTools',
        namespace=namespace,
        output='screen',
        parameters=[{
            'metricFile': '$(find-pkg-prefix vehicle_simulator)/log/metrics',
            'trajFile': '$(find-pkg-prefix vehicle_simulator)/log/trajectory',
            'mapFile': '$(find-pkg-share vehicle_simulator)/mesh/$(var world_name)/preview/pointcloud.ply',
            'overallMapVoxelSize': 0.5,
            'exploredAreaVoxelSize': 0.3,
            'exploredVolumeVoxelSize': 0.5,
            'transInterval': 0.2,
            'yawInterval': 10.0,
            'overallMapDisplayInterval': 2,
            'exploredAreaDisplayInterval': 1
        }]
    )

    realTimePlot_node = Node(
        package='visualization_tools',
        executable='realTimePlot.py',
        name='realTimePlot',
        namespace=namespace,
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(visualizationTools_node)
    ld.add_action(realTimePlot_node)

    return ld