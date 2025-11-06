"""
Full autonomous navigation with SLAM for Ultrabot AGV.
Combines SLAM mapping with Nav2 navigation - explore and navigate simultaneously.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('somanet')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration files
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    robot_config_file = os.path.join(pkg_dir, 'config', 'robot_config.yaml')
    safety_params_file = os.path.join(pkg_dir, 'config', 'safety_params.yaml')
    ekf_params_file = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'ultrabot.urdf.xacro')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_mode = LaunchConfiguration('slam_mode')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RVIZ2 for visualization')
    
    declare_slam_mode_cmd = DeclareLaunchArgument(
        'slam_mode',
        default_value='slam_toolbox',
        choices=['slam_toolbox', 'cartographer'],
        description='SLAM algorithm to use')
    
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
    
    # Somanet driver node (EtherCAT + odometry)
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
    
    # Safety supervisor node (ISO 13849-1)
    safety_supervisor = Node(
        package='somanet',
        executable='safety_supervisor_node',
        name='safety_supervisor',
        output='screen',
        parameters=[safety_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # Command mux node (teleop vs autonomous)
    command_mux = Node(
        package='somanet',
        executable='command_mux_node',
        name='command_mux',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'teleop_timeout': 1.0,
            'nav2_timeout': 1.0,
            'other_timeout': 1.0
        }]
    )
    
    # Command arbitrator node (4-priority selection)
    command_arbitrator = Node(
        package='somanet',
        executable='command_arbitrator_node',
        name='command_arbitrator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel_auto', '/cmd_vel_mux')
        ]
    )
    
    # Teleoperation (joystick)
    teleop_joy = Node(
        package='somanet',
        executable='teleop_joy',
        name='teleop_joy',
        output='screen',
        parameters=[safety_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_teleop')
        ]
    )
    
    # SLAM Toolbox (async mode for online SLAM)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # Nav2 bringup (without map_server and AMCL - using SLAM instead)
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file
        }.items()
    )
    
    # RVIZ2 for visualization
    rviz_config_file = os.path.join(pkg_dir, 'config', 'slam_nav_rviz_config.rviz')
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
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_slam_mode_cmd)
    
    # Robot description (URDF) - MUST be first
    ld.add_action(robot_state_publisher_node)
    
    # Hardware and safety
    ld.add_action(driver_node)
    ld.add_action(ekf_node)
    ld.add_action(safety_supervisor)
    
    # Command arbitration
    ld.add_action(command_mux)
    ld.add_action(command_arbitrator)
    
    # Teleoperation
    ld.add_action(teleop_joy)
    
    # SLAM
    ld.add_action(slam_toolbox)
    
    # Navigation
    ld.add_action(nav2_bringup)
    
    # Visualization
    ld.add_action(rviz_node)
    
    return ld
