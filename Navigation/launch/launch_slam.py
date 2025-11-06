"""
SLAM launch file for Ultrabot AGV.
Uses slam_toolbox for online SLAM and map creation.
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
    
    # Configuration files
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    robot_config_file = os.path.join(pkg_dir, 'config', 'robot_config.yaml')
    safety_params_file = os.path.join(pkg_dir, 'config', 'safety_params.yaml')
    ekf_params_file = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'ultrabot.urdf.xacro')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file_arg = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Full path to SLAM configuration file')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RVIZ2 for visualization')
    
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
        parameters=[robot_config_file, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'info']
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
    
    # Command arbitrator node
    command_arbitrator = Node(
        package='somanet',
        executable='command_arbitrator_node',
        name='command_arbitrator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Teleoperation (joystick)
    teleop_joy = Node(
        package='somanet',
        executable='teleop_joy',
        name='teleop_joy',
        output='screen',
        parameters=[safety_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # SLAM Toolbox (async mode for online SLAM)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file_arg, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/odom', '/odom')
        ]
    )
    
    # RVIZ2 for visualization
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
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    
    # Add nodes (URDF must be first for TF tree)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(driver_node)
    ld.add_action(ekf_node)
    ld.add_action(safety_supervisor)
    ld.add_action(command_arbitrator)
    ld.add_action(teleop_joy)
    ld.add_action(slam_toolbox)
    ld.add_action(rviz_node)
    
    return ld
