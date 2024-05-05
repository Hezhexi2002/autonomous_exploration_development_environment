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

    loamInterface_node = Node(
        package='loam_interface',
        executable='loamInterface',
        name='loamInterface',
        namespace=namespace,
        output='screen',
        parameters=[{
            'stateEstimationTopic': '/Odometry',
            'registeredScanTopic': '/cloud_registered',
            'flipStateEstimation': False,
            'flipRegisteredScan': False,
            'sendTF': True,
            'reverseTF': False
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(loamInterface_node)

    return ld