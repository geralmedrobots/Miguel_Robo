#!/usr/bin/env python3
"""
Sensor stack launch for Ultrabot AGV.
Launches all robot sensors: 2D LiDARs, 3D LiDAR, RealSense cameras, IMU.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('somanet')
    sensors_config = os.path.join(pkg_dir, 'config', 'sensors_config.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_lidar_front = LaunchConfiguration('enable_lidar_front')
    enable_lidar_rear = LaunchConfiguration('enable_lidar_rear')
    enable_ouster = LaunchConfiguration('enable_ouster')
    enable_realsense_left = LaunchConfiguration('enable_realsense_left')
    enable_realsense_right = LaunchConfiguration('enable_realsense_right')
    enable_imu = LaunchConfiguration('enable_imu')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_enable_lidar_front_cmd = DeclareLaunchArgument(
        'enable_lidar_front',
        default_value='true',
        description='Enable front 2D LiDAR')
    
    declare_enable_lidar_rear_cmd = DeclareLaunchArgument(
        'enable_lidar_rear',
        default_value='true',
        description='Enable rear 2D LiDAR')
    
    declare_enable_ouster_cmd = DeclareLaunchArgument(
        'enable_ouster',
        default_value='false',
        description='Enable Ouster 3D LiDAR (PointCloud2)')
    
    declare_enable_realsense_left_cmd = DeclareLaunchArgument(
        'enable_realsense_left',
        default_value='false',
        description='Enable front-left RealSense D455')
    
    declare_enable_realsense_right_cmd = DeclareLaunchArgument(
        'enable_realsense_right',
        default_value='false',
        description='Enable front-right RealSense D455')
    
    declare_enable_imu_cmd = DeclareLaunchArgument(
        'enable_imu',
        default_value='false',
        description='Enable IMU sensor')
    
    # ========================================================================
    # FRONT 2D LIDAR (Primary Navigation Sensor)
    # ========================================================================
    # Example for SICK TiM571 - replace with your LiDAR driver
    lidar_front_node = Node(
        package='sick_scan',  # or 'urg_node', 'rplidar_ros'
        executable='sick_generic_caller',
        name='lidar_front',
        output='screen',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.1.11',  # Or serial: '/dev/ttyUSB0'
            'port': '2112',
            'frame_id': 'scan_front_link',
            'scan_topic': '/scan_front',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/scan', '/scan_front')
        ],
        condition=IfCondition(enable_lidar_front)
    )
    
    # ========================================================================
    # REAR 2D LIDAR (Rear Coverage)
    # ========================================================================
    lidar_rear_node = Node(
        package='sick_scan',
        executable='sick_generic_caller',
        name='lidar_rear',
        output='screen',
        parameters=[{
            'scanner_type': 'sick_tim_5xx',
            'hostname': '192.168.1.12',
            'port': '2112',
            'frame_id': 'scan_rear_link',
            'scan_topic': '/scan_rear',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/scan', '/scan_rear')
        ],
        condition=IfCondition(enable_lidar_rear)
    )
    
    # ========================================================================
    # LASERSCAN MERGE (Combine Front + Rear → /scan for Nav2)
    # ========================================================================
    laserscan_merger_node = Node(
        package='ira_laser_tools',  # Or 'laserscan_multi_merger'
        executable='laserscan_multi_merger',
        name='laserscan_merger',
        output='screen',
        parameters=[{
            'destination_frame': 'base_link',
            'cloud_destination_topic': '/scan_merged_cloud',
            'scan_destination_topic': '/scan',
            'laserscan_topics': '/scan_front /scan_rear',
            'angle_min': -3.14159,  # -180°
            'angle_max': 3.14159,   # +180°
            'angle_increment': 0.00436,  # ~0.25°
            'range_min': 0.05,
            'range_max': 25.0,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(enable_lidar_front)  # At least front LiDAR needed
    )
    
    # ========================================================================
    # OUSTER 3D LIDAR (OS1-64/OS1-128)
    # ========================================================================
    ouster_driver_node = Node(
        package='ouster_ros',
        executable='ouster_driver_node',
        name='ouster_driver',
        output='screen',
        parameters=[{
            'lidar_ip': '192.168.1.100',
            'computer_ip': '192.168.1.10',
            'lidar_mode': '1024x10',
            'timestamp_mode': 'TIME_FROM_PTP_1588',
            'frame_id': 'ouster_link',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/points', '/ouster/points')
        ],
        condition=IfCondition(enable_ouster)
    )
    
    # Voxel grid filter for Ouster (downsample for Nav2)
    ouster_voxel_filter_node = Node(
        package='pcl_ros',
        executable='voxel_grid',
        name='ouster_voxel_filter',
        output='screen',
        parameters=[{
            'leaf_size': 0.05,  # 5cm voxels
            'input_frame': 'ouster_link',
            'output_frame': 'ouster_link',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/input', '/ouster/points'),
            ('/output', '/ouster/points_filtered')
        ],
        condition=IfCondition(enable_ouster)
    )
    
    # ========================================================================
    # REALSENSE D455 - FRONT LEFT & RIGHT
    # ========================================================================
    try:
        realsense_pkg_dir = get_package_share_directory('realsense2_camera')
        
        realsense_front_left = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_pkg_dir, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'serial_no': '123456789012',  # Get with: rs-enumerate-devices
                'camera_name': 'camera_front_left',
                'camera_namespace': 'camera_front_left',
                'depth_module.depth_profile': '640x480x30',
                'rgb_camera.color_profile': '640x480x30',
                'enable_depth': 'true',
                'enable_color': 'true',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_pointcloud': 'true',
                'pointcloud.ordered_pc': 'false',
                'align_depth.enable': 'true',
                'spatial_filter.enable': 'true',
                'temporal_filter.enable': 'true',
                'hole_filling_filter.enable': 'true',
                'decimation_filter.enable': 'true',
                'base_frame_id': 'camera_front_left_link',
                'depth_optical_frame_id': 'camera_front_left_depth_optical_frame',
                'use_sim_time': use_sim_time
            }.items(),
            condition=IfCondition(enable_realsense_left)
        )
        
        realsense_front_right = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_pkg_dir, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'serial_no': '210987654321',
                'camera_name': 'camera_front_right',
                'camera_namespace': 'camera_front_right',
                'depth_module.depth_profile': '640x480x30',
                'rgb_camera.color_profile': '640x480x30',
                'enable_depth': 'true',
                'enable_color': 'true',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_pointcloud': 'true',
                'pointcloud.ordered_pc': 'false',
                'align_depth.enable': 'true',
                'spatial_filter.enable': 'true',
                'temporal_filter.enable': 'true',
                'hole_filling_filter.enable': 'true',
                'decimation_filter.enable': 'true',
                'base_frame_id': 'camera_front_right_link',
                'depth_optical_frame_id': 'camera_front_right_depth_optical_frame',
                'use_sim_time': use_sim_time
            }.items(),
            condition=IfCondition(enable_realsense_right)
        )
    except Exception as e:
        print(f"⚠️  realsense2_camera package not found - RealSense disabled: {e}")
        print("   Install with: sudo apt install ros-${ROS_DISTRO}-realsense2-camera")
        realsense_front_left = GroupAction([])  # Empty action
        realsense_front_right = GroupAction([])
    
    # ========================================================================
    # POINTCLOUD TO LASERSCAN (RealSense Depth → 2D Scan for Nav2)
    # ========================================================================
    pointcloud_to_laserscan_left = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_left',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': -0.2,
            'max_height': 0.5,
            'angle_min': -0.785398,  # -45°
            'angle_max': 0.785398,   # +45°
            'angle_increment': 0.0087,  # ~0.5°
            'scan_time': 0.033,  # 30 Hz
            'range_min': 0.3,
            'range_max': 6.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/cloud_in', '/camera_front_left/depth/color/points'),
            ('/scan', '/scan_realsense_left')
        ],
        condition=IfCondition(enable_realsense_left)
    )
    
    pointcloud_to_laserscan_right = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_right',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': -0.2,
            'max_height': 0.5,
            'angle_min': -0.785398,
            'angle_max': 0.785398,
            'angle_increment': 0.0087,
            'scan_time': 0.033,
            'range_min': 0.3,
            'range_max': 6.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/cloud_in', '/camera_front_right/depth/color/points'),
            ('/scan', '/scan_realsense_right')
        ],
        condition=IfCondition(enable_realsense_right)
    )
    
    # ========================================================================
    # IMU SENSOR (9-DOF for EKF Sensor Fusion)
    # ========================================================================
    # Example for MPU9250 - replace with your IMU driver
    imu_node = Node(
        package='mpu9250_driver',  # or 'bno055', 'vectornav', 'phidgets_imu'
        executable='mpu9250_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'frame_id': 'imu_link',
            'frequency': 100,  # Hz
            'port': '/dev/ttyUSB2',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/imu', '/imu/data')
        ],
        condition=IfCondition(enable_imu)
    )
    
    # ========================================================================
    # SENSOR DIAGNOSTICS AGGREGATOR
    # ========================================================================
    diagnostic_aggregator_node = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        parameters=[{
            'analyzers': {
                'sensors': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'Sensors',
                    'contains': ['lidar', 'camera', 'imu', 'ouster']
                }
            },
            'use_sim_time': use_sim_time
        }]
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    ld = LaunchDescription()
    
    # Arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_enable_lidar_front_cmd)
    ld.add_action(declare_enable_lidar_rear_cmd)
    ld.add_action(declare_enable_ouster_cmd)
    ld.add_action(declare_enable_realsense_left_cmd)
    ld.add_action(declare_enable_realsense_right_cmd)
    ld.add_action(declare_enable_imu_cmd)
    
    # 2D LiDARs
    ld.add_action(lidar_front_node)
    ld.add_action(lidar_rear_node)
    ld.add_action(laserscan_merger_node)
    
    # 3D LiDAR
    ld.add_action(ouster_driver_node)
    ld.add_action(ouster_voxel_filter_node)
    
    # RealSense cameras
    ld.add_action(realsense_front_left)
    ld.add_action(realsense_front_right)
    ld.add_action(pointcloud_to_laserscan_left)
    ld.add_action(pointcloud_to_laserscan_right)
    
    # IMU
    ld.add_action(imu_node)
    
    # Diagnostics
    ld.add_action(diagnostic_aggregator_node)
    
    return ld
