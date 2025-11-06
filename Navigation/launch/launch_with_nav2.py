#!/usr/bin/env python3
"""
Navigation launch with Nav2 integration for Ultrabot AGV.
Complete autonomous navigation stack with safety supervision.
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
    # Package directories
    pkg_dir = get_package_share_directory('somanet')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Configuration files
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    robot_config_file = os.path.join(pkg_dir, 'config', 'robot_config.yaml')
    safety_params_file = os.path.join(pkg_dir, 'config', 'safety_params.yaml')
    ekf_params_file = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'ultrabot.urdf.xacro')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load (leave empty to skip map_server)')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')
    
    # Set environment for logging
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    # ========================================================================
    # HARDWARE LAYER: EtherCAT Driver + Odometry
    # ========================================================================
    somanet_driver_node = Node(
        package='somanet',
        executable='main',
        name='somanet_driver',
        output='screen',
        parameters=[robot_config_file],
        arguments=['--ros-args', '--log-level', 'info'],
        respawn=False,
        respawn_delay=2.0
    )
    
    # ========================================================================
    # ROBOT DESCRIPTION: URDF + Static TF Publisher
    # ========================================================================
    # Process URDF/xacro file to generate robot_description parameter
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
    
    # ========================================================================
    # SENSOR FUSION: EKF for Odometry Filtering
    # ========================================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[
            ('/odometry/filtered', '/odom_filtered')
        ]
    )
    
    # ========================================================================
    # SAFETY LAYER: Independent Safety Supervisor (ISO 13849-1)
    # ========================================================================
    safety_supervisor_node = Node(
        package='somanet',
        executable='safety_supervisor_node',
        name='safety_supervisor',
        output='screen',
        parameters=[safety_params_file],
        arguments=['--ros-args', '--log-level', 'info'],
        respawn=True,
        respawn_delay=1.0
    )
    
    # ========================================================================
    # COMMAND ARBITRATION LAYER
    # ========================================================================
    
    # Command Mux: Consolidates autonomous command sources
    command_mux_node = Node(
        package='somanet',
        executable='command_mux_node',
        name='command_mux',
        output='screen',
        parameters=[{
            'default_timeout': 1.0,
            'teleop_timeout': 1.0,
            'nav2_timeout': 1.0  # Increased: Nav2 may skip cycles during replanning
        }],
        remappings=[
            # Output goes to command_arbitrator's 'auto' input
            ('cmd_vel_mux', 'cmd_vel_auto')
        ]
    )
    
    # Command Arbitrator: Final safety-aware multiplexing with priority
    command_arbitrator_node = Node(
        package='somanet',
        executable='command_arbitrator_node',
        name='command_arbitrator',
        output='screen',
        parameters=[{
            'default_timeout': 0.5,
            'require_deadman_for_manual': False  # Set True if using deadman switch
        }],
        arguments=['--autostart'],  # Auto-activate lifecycle
        respawn=True,
        respawn_delay=1.0
    )
    
    # ========================================================================
    # TELEOP LAYER: Joystick + Keyboard
    # ========================================================================
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }]
    )
    
    teleop_joy_node = Node(
        package='somanet',
        executable='teleop_joy',
        name='teleop_joy',
        output='screen',
        parameters=[robot_config_file],
        remappings=[
            # Output to command_mux teleop input
            ('cmd_vel', 'cmd_vel_teleop')
        ]
    )
    
    teleop_keyboard_node = Node(
        package='somanet',
        executable='teleop_keyboard_safe.py',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal
        remappings=[
            ('cmd_vel', 'cmd_vel_teleop')
        ]
    )
    
    # ========================================================================
    # NAV2 STACK: Autonomous Navigation
    # ========================================================================
    
    # Include Nav2 bringup (controller, planner, bt_navigator, behaviors, etc.)
    # NOTE: Remapping is handled via nav2_params.yaml (controller_server remaps cmd_vel)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file
        }.items()
    )
    
    # Map server (optional - only if map file provided)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }],
        condition=IfCondition(LaunchConfiguration('map', default=''))
    )
    
    # Lifecycle manager for map server
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server']
        }],
        condition=IfCondition(LaunchConfiguration('map', default=''))
    )
    
    # AMCL (Adaptive Monte Carlo Localization) - optional
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('map', default=''))
    )
    
    # RVIZ for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    ld = LaunchDescription()
    
    # Environment
    ld.add_action(stdout_linebuf_envvar)
    
    # Arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    
    # Robot description (URDF) - MUST start first for TF tree
    ld.add_action(robot_state_publisher_node)
    
    # Hardware + Safety (CRITICAL - start first)
    ld.add_action(somanet_driver_node)
    ld.add_action(ekf_node)
    ld.add_action(safety_supervisor_node)
    
    # Command arbitration
    ld.add_action(command_mux_node)
    ld.add_action(command_arbitrator_node)
    
    # Teleop
    ld.add_action(joy_node)
    ld.add_action(teleop_joy_node)
    # ld.add_action(teleop_keyboard_node)  # Uncomment if needed (requires xterm)
    
    # Nav2 stack
    ld.add_action(nav2_bringup)
    
    # Localization (if map provided)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_localization)
    ld.add_action(amcl_node)
    
    # Visualization
    ld.add_action(rviz_node)
    
    return ld
