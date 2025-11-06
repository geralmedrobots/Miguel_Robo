"""
Cartographer SLAM launch file for Ultrabot AGV.
Alternative SLAM implementation using Google Cartographer.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('somanet')
    cartographer_ros_prefix = get_package_share_directory('cartographer_ros')
    
    # Configuration files
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    configuration_basename = 'cartographer_config.lua'
    robot_config_file = os.path.join(pkg_dir, 'config', 'robot_config.yaml')
    safety_params_file = os.path.join(pkg_dir, 'config', 'safety_params.yaml')
    ekf_params_file = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'ultrabot.urdf.xacro')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of a grid cell in the published occupancy grid')
    
    declare_publish_period_sec_cmd = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='OccupancyGrid publishing period')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RVIZ2 for visualization')
    
    # Somanet driver node (EtherCAT + odometry)
    # Robot description (URDF + static TF)
    robot_description_content = Command(['xacro ', urdf_file])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    driver_node = Node(
        package='somanet',
        executable='main',
        name='somanet_driver',
        output='screen',
        parameters=[robot_config_file, {'use_sim_time': use_sim_time}]
    )
    
    # EKF sensor fusion (optional - provides /odom_filtered)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odom_filtered')]
    )
    
    # Safety supervisor node
    safety_supervisor = Node(
        package='somanet',
        executable='safety_supervisor_node',
        name='safety_supervisor',
        output='screen',
        parameters=[safety_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # Command arbitrator node
    command_arbitrator = Node(
        package='somanet',
        executable='command_arbitrator_node',
        name='command_arbitrator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Teleoperation
    teleop_joy = Node(
        package='somanet',
        executable='teleop_joy',
        name='teleop_joy',
        output='screen',
        parameters=[safety_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom'),
            ('/imu', '/imu')
        ]
    )
    
    # Cartographer occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': resolution},
            {'publish_period_sec': publish_period_sec}
        ]
    )
    
    # RVIZ2
    rviz_config_file = os.path.join(pkg_dir, 'config', 'slam_rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    ld = LaunchDescription()
    
    # Launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_resolution_cmd)
    ld.add_action(declare_publish_period_sec_cmd)
    ld.add_action(declare_use_rviz_cmd)
    
    # Nodes (URDF must be first for TF tree)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(driver_node)
    ld.add_action(ekf_node)
    ld.add_action(safety_supervisor)
    ld.add_action(command_arbitrator)
    ld.add_action(teleop_joy)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)
    
    return ld
