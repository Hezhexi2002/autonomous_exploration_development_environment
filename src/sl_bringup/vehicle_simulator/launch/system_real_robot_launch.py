import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():
  world_name = LaunchConfiguration('world_name')
  cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
  vehicleX = LaunchConfiguration('vehicleX')
  vehicleY = LaunchConfiguration('vehicleY')
  checkTerrainConn = LaunchConfiguration('checkTerrainConn')
  robot_id = LaunchConfiguration('robot_id')
  
  declare_world_name = DeclareLaunchArgument('world_name', default_value='campus', description='')
  declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='')
  declare_vehicleX = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
  declare_vehicleY = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
  declare_checkTerrainConn = DeclareLaunchArgument('checkTerrainConn', default_value='true', description='')
  declare_robot_id = DeclareLaunchArgument('robot_id', default_value='robot_0', description='')
  
  
  twist_unstamper = Node(
        package='twist_stamper',
        executable='twist_unstamper',
        name='twist_unstamper',
        namespace=robot_id,
        remappings=[
            ('cmd_vel_in', 'cmd_vel_stamped'),  # 输入主题重映射
            ('cmd_vel_out', 'cmd_vel')  # 输出主题重映射为 /cmd_vel
        ]
    )
  
  start_local_planner = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')
    ),
    launch_arguments={
      'namespace': robot_id,
      'cameraOffsetZ': cameraOffsetZ,
      'goalX': vehicleX,
      'goalY': vehicleY,
    }.items()
  )
  
  start_sl_navigation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('sl_navigation'), 'launch', 'bringup_launch.py')
    ),
    launch_arguments={
      'namespace': robot_id,
    }.items()
  )
  
  start_sl_mapping = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('rtabmap_livox'), 'launch', 'rtabmap_livox.launch.py')
    ),
    launch_arguments={
      'namespace': robot_id,
    }.items()
  )
  
  start_pointcloud_to_laserscan = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('pointcloud_to_laserscan'), 'launch', 'pointcloud_to_laserscan_launch.py')
    ),
    launch_arguments={
        'namespace': robot_id,
    }.items()
)

  start_terrain_analysis = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis_launch.py')
    ),
    launch_arguments={
      'namespace': robot_id,
    }.items()
  )

  # start_terrain_analysis = IncludeLaunchDescription(
  #   FrontendLaunchDescriptionSource(os.path.join(
  #     get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
  #   ),
  #   launch_arguments={
  #     'namespace': robot_id,
  #   }.items()
  # )
  
  start_terrain_analysis_ext = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext_launch.py')
    ),
    launch_arguments={
      'namespace': robot_id,
      'checkTerrainConn': checkTerrainConn,
    }.items()
  )
  
  # start_terrain_analysis_ext = IncludeLaunchDescription(
  #   FrontendLaunchDescriptionSource(os.path.join(
  #     get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
  #   ),
  #   launch_arguments={
  #     'namespace': robot_id,
  #     'checkTerrainConn': checkTerrainConn,
  #   }.items()
  # )
  
  start_sensor_scan_generation = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch')
    ),
    launch_arguments={
      'namespace': robot_id,
    }.items()
  )
  
  start_visualization_tools = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')
    ),
    launch_arguments={
      'namespace': robot_id,
      'world_name': world_name,
    }.items()
  )

  start_loam_interface = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('loam_interface'), 'launch', 'loam_interface.launch')
    ),
    launch_arguments={
      'namespace': robot_id,
    }.items()
  )

  start_livox_driver = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(
          get_package_share_directory('livox_ros_driver2'), 'launch', 'msg_MID360_launch.py'
      )),
    launch_arguments={
      'namespace': robot_id,
    }.items()
  )
  
  start_fast_lio = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(
          get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py'
      ))
  )
  
  rviz_config_file = os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'vehicle_simulator.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
  )

  delayed_start_rviz = TimerAction(
    period=8.0,
    actions=[
      start_rviz
    ]
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_world_name)
  ld.add_action(declare_cameraOffsetZ)
  ld.add_action(declare_vehicleX)
  ld.add_action(declare_vehicleY)
  ld.add_action(declare_checkTerrainConn)
  ld.add_action(declare_robot_id)

  

  ld.add_action(start_livox_driver)  # 添加雷达驱动启动
  ld.add_action(start_fast_lio)  # 添加 FastLIO 启动
  ld.add_action(start_loam_interface)
  ld.add_action(start_sensor_scan_generation)
  ld.add_action(start_terrain_analysis)
  ld.add_action(start_terrain_analysis_ext)
  ld.add_action(start_local_planner)
  # ld.add_action(start_pointcloud_to_laserscan)
  # ld.add_action(start_sl_navigation)
  #　ld.add_action(start_sl_mapping)
  ld.add_action(start_visualization_tools)
  ld.add_action(twist_unstamper)
  ld.add_action(delayed_start_rviz)

  return ld