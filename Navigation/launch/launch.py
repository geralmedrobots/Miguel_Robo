import os

from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler, TimerAction  # type: ignore[import-not-found]
from launch.substitutions import Command  # type: ignore[import-not-found]
from launch_ros.actions import LifecycleNode, Node  # type: ignore[import-not-found]
from launch_ros.event_handlers import OnStateTransition  # type: ignore[import-not-found]
from launch_ros.events.lifecycle import ChangeState, matches_action  # type: ignore[import-not-found]
from ament_index_python.packages import get_package_share_directory  # type: ignore[import-not-found]
from lifecycle_msgs.msg import Transition  # type: ignore[import-not-found]


def generate_launch_description():
    pkg_share = get_package_share_directory('somanet')

    odometry_params = os.path.join(pkg_share, 'config', 'odometry_params.yaml')
    safety_params = os.path.join(pkg_share, 'config', 'safety_params.yaml')
    robot_params = os.path.join(pkg_share, 'config', 'robot_config.yaml')
    ekf_params = os.path.join(pkg_share, 'config', 'ekf_params.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'ultrabot.urdf.xacro')
    
    # ========================================================================
    # FIX PROBLEM 3: ETHERCAT INTERFACE CONFIGURATION
    # ========================================================================
    # Interface name is now REQUIRED - no hardcoded defaults
    # Load from robot_config.yaml or environment variable
    ethercat_interface = os.environ.get('ETHERCAT_INTERFACE', '')  # e.g., export ETHERCAT_INTERFACE=eth0

    # ========================================================================
    # SROS2 SECURITY CONFIGURATION (ISO 27001, IEC 62443)
    # ========================================================================
    # Enable secure communication if environment variables are set:
    #   ROS_SECURITY_ENABLE=true
    #   ROS_SECURITY_KEYSTORE=/path/to/sros2_keystore
    #   ROS_SECURITY_STRATEGY=Enforce
    #
    # Security enclaves provide:
    #   - Authentication: Only authorized nodes can communicate
    #   - Encryption: All topic/service data is encrypted with DDS Security
    #   - Access control: Nodes can only pub/sub to allowed topics
    #
    # To set up SROS2:
    #   1. Run: scripts/setup_sros2.sh
    #   2. Export environment variables
    #   3. Launch normally - security is automatic
    # ========================================================================
    
    use_sros2 = os.environ.get('ROS_SECURITY_ENABLE', 'false').lower() == 'true'
    
    if use_sros2:
        sros2_keystore = os.environ.get('ROS_SECURITY_KEYSTORE', '')
        sros2_strategy = os.environ.get('ROS_SECURITY_STRATEGY', 'Enforce')
        
        if not sros2_keystore:
            print("⚠️  WARNING: ROS_SECURITY_ENABLE=true but ROS_SECURITY_KEYSTORE not set!")
            print("   Security will be disabled. Run scripts/setup_sros2.sh first.")
            use_sros2 = False
        else:
            print(f"🔒 SROS2 ENABLED - Secure communication active")
            print(f"   Keystore: {sros2_keystore}")
            print(f"   Strategy: {sros2_strategy}")
    else:
        print("⚠️  SROS2 DISABLED - Communication is NOT encrypted")
        print("   For production, enable SROS2 with: scripts/setup_sros2.sh")
    
    # Define security enclaves for each node (only used if SROS2 enabled)
    # Security enclave format: /namespace/node_name
    safety_enclave = '/somanet/safety_supervisor' if use_sros2 else None
    driver_enclave = '/somanet/somanet_driver' if use_sros2 else None
    teleop_enclave = '/somanet/teleop_joy' if use_sros2 else None

    safety_supervisor = LifecycleNode(
        package='somanet',
        executable='safety_supervisor_node',
        name='safety_supervisor',
        namespace='',
        output='screen',
        parameters=[safety_params],
        additional_env={'ROS_SECURITY_ENCLAVE_OVERRIDE': safety_enclave} if safety_enclave else None
    )

    # ========================================================================
    # FIX PROBLEM 9: ENSURE YAML CONFIG ALWAYS LOADED
    # ========================================================================
    # Build driver parameters (YAML files + optional environment overrides)
    # Priority: Environment variable > Launch parameter > YAML config file
    
    driver_params_list = [odometry_params, robot_params]  # Always load base config
    
    if ethercat_interface:
        # Environment variable OVERRIDES config file value
        driver_params_list.append({'ethercat_interface': ethercat_interface})
        print(f"🔌 EtherCAT interface override from environment: {ethercat_interface}")
        print(f"   (supersedes value in {robot_params})")
    else:
        print(f"ℹ️  Loading EtherCAT interface from: {robot_params}")
        print("   Override with: export ETHERCAT_INTERFACE=eth0")

    driver_node = LifecycleNode(
        package='somanet',
        executable='main',
        name='somanet_driver',
        namespace='',
        output='screen',
        parameters=driver_params_list,
        additional_env={'ROS_SECURITY_ENCLAVE_OVERRIDE': driver_enclave} if driver_enclave else None
    )

    teleop_joy = Node(
        package='somanet',
        executable='teleop_joy',
        name='teleop_joy',
        namespace='',
        output='screen',
        parameters=[robot_params],
        additional_env={'ROS_SECURITY_ENCLAVE_OVERRIDE': teleop_enclave} if teleop_enclave else None
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
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
            'use_sim_time': False
        }]
    )

    # ========================================================================
    # EKF SENSOR FUSION (robot_localization)
    # ========================================================================
    # Extended Kalman Filter for odometry fusion
    # Inputs: /odom (wheel encoders from somanet_driver)
    # Output: /odometry/filtered (fused estimate)
    # TF: Publishes odom -> base_link transform
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params],
        remappings=[
            ('/odometry/filtered', '/odom_filtered')
        ]
    )

    configure_safety = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(safety_supervisor),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_safety = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(safety_supervisor),
            transition_id=Transition.TRANSITION_ACTIVATE
        )
    )

    configure_driver = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_driver = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver_node),
            transition_id=Transition.TRANSITION_ACTIVATE
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,  # URDF/TF tree (MUST start first)
        joy_node,
        teleop_joy,
        ekf_node,
        safety_supervisor,
        driver_node,

        TimerAction(period=0.1, actions=[
            LogInfo(msg='[launch] Configuring safety supervisor...'),
            configure_safety,
        ]),

        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=safety_supervisor,
                goal_state='inactive',
                entities=[
                    LogInfo(msg='[launch] Safety supervisor configured, activating...'),
                    activate_safety,
                ]
            )
        ),

        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=safety_supervisor,
                goal_state='active',
                entities=[
                    LogInfo(msg='[launch] Safety supervisor active, configuring driver...'),
                    configure_driver,
                ]
            )
        ),

        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=driver_node,
                goal_state='inactive',
                entities=[
                    LogInfo(msg='[launch] Driver configured, activating...'),
                    activate_driver,
                ]
            )
        ),

        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=driver_node,
                goal_state='active',
                entities=[
                    LogInfo(msg='[launch] Driver active – system ready for operation.'),
                ]
            )
        ),
    ])
