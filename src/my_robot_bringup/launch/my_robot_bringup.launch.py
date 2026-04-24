import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- Path Setup ---
    pkg_share = get_package_share_directory('my_robot_bringup')
    control_pkg_share = get_package_share_directory('robo_cayote_control')
    realsense_share = get_package_share_directory('realsense2_camera')
    
    # Config Paths
    nvblox_config = os.path.join(pkg_share, 'config', 'nvblox_params.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robo_cayote.urdf')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    nav2_config = os.path.join(pkg_share, 'config', 'costmap.yaml') # Your costmap.yaml now holds ALL nav2 params
    mqtt_config = os.path.join(control_pkg_share, 'config', 'mqtt_params.yaml')

    # --- Launch Configuration ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_ris_stream = LaunchConfiguration('enable_ris_stream', default='false')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # --- Node Definitions ---

    # 1. RealSense (Visual Sensors)
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'enable_color': 'true', 
            'enable_depth': 'true', 
            'enable_sync': 'true',
            'enable_infra1': 'true', 
            'enable_infra2': 'true',
            'enable_gyro': 'false', 
            'enable_accel': 'false', # Witmotion handles IMU
            'align_depth.enable': 'true', 
            'use_sim_time': use_sim_time,
            # --- New Bandwidth & Stability Fixes ---
            'initial_reset': 'true',
            'depth_module.depth_profile': '848x480x30',
            'depth_module.infra_profile': '848x480x30',
            'rgb_camera.color_profile': '640x480x30'
        }.items()
    )

    # 2. State Publishers
    state_publishers = GroupAction([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': use_sim_time}])
    ])

    # 3. Isaac ROS Perception (VSLAM & Nvblox)
    isaac_stack = ComposableNodeContainer(
        name='isaac_ros_container', namespace='', package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam', 
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam',
                parameters=[{
                    'publish_odom_to_base_tf': False, 
                    'publish_map_to_odom_tf': False,
                    'base_frame': 'base_footprint', 
                    'use_sim_time': use_sim_time,
                    'input_left_camera_frame': 'camera_infra1_optical_frame', 
                    'input_right_camera_frame': 'camera_infra2_optical_frame', 
                    'input_imu_frame': 'imu'
                }],
                remappings=[
                    ('/visual_slam/image_0', '/camera/camera/infra1/image_rect_raw'),
                    ('/visual_slam/image_1', '/camera/camera/infra2/image_rect_raw'),
                    ('/visual_slam/camera_info_0', '/camera/camera/infra1/camera_info'),
                    ('/visual_slam/camera_info_1', '/camera/camera/infra2/camera_info'),
                    ('/visual_slam/imu', '/witmotion_imu/imu')
                ]
            ),
            ComposableNode(
                package='nvblox_ros', plugin='nvblox::NvbloxNode', name='nvblox_node',
                parameters=[nvblox_config, {'use_sim_time': use_sim_time}],
                remappings=[('depth/image', '/camera/camera/depth/image_rect_raw')]
            ),
        ]
    )

    # 4. Localization (EKF & GPS)
    localization_stack = GroupAction([
        Node(package='robot_localization', executable='ekf_node', name='ekf_local',
             parameters=[ekf_config, {'use_sim_time': use_sim_time}],
             remappings=[('odometry/filtered', '/odometry/local')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_global',
             parameters=[ekf_config, {'use_sim_time': use_sim_time}],
             remappings=[('odometry/filtered', '/odometry/global')]),
        Node(package='robot_localization', executable='navsat_transform_node', name='navsat_transform',
             parameters=[ekf_config, {'use_sim_time': use_sim_time}],
             remappings=[('imu', '/witmotion_imu/imu'), ('gps/fix', '/gps/fix'), ('odometry/filtered', '/odometry/local')])
    ])

    # 5. Full Nav2 Stack
    nav2_stack = GroupAction([
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[nav2_config, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[nav2_config, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[nav2_config, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
             parameters=[nav2_config, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[nav2_config, {'use_sim_time': use_sim_time}]),
    ])

    # 6. Lifecycle Manager (Wakes everything up in order)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'autostart': True, 'use_sim_time': use_sim_time,
                     'node_names': ['controller_server', 'planner_server', 
                                   'bt_navigator', 'waypoint_follower', 'behavior_server']}]
    )

    # 7. Custom Cayote Nodes
    custom_logic = GroupAction([
        # Mission Controller (The Bridge)
        Node(package='robo_cayote_control', executable='mission_controller', 
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='robo_cayote_control', executable='mqtt_ack_node', 
             parameters=[mqtt_config, {'use_sim_time': use_sim_time}]),
        Node(package='robo_cayote_control', executable='cayote_rl_brain', 
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='robo_cayote_control', executable='arduino_motor_driver',
             parameters=[{'use_sim_time': use_sim_time,
                          'serial_port': '/dev/ttyACM1'}]),
        # Merged Witmotion IMU Node
        Node(package='witmotion_ros2', executable='witmotion_ros2', name='witmotion_imu',
             parameters=[{
                 'port': '/dev/ttyUSB0', 
                 'baud_rate': 115200, 
                 'use_sim_time': use_sim_time
             }],
             remappings=[('imu', '/witmotion_imu/imu')])
    ])
    
    gps_bridge_node = Node(
        package='pico_comms',
        executable='gps_bridge', # This must match what is in your setup.py
        name='gps_bridge_node',
        output='screen'
    )

    sensor_pub_node = Node(
        package='pico_comms',
        executable='sensor_pub', 
        name='ultrasonic_sensor_node',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_ris_stream', default_value='false'),
        
        realsense_node,
        state_publishers,
        isaac_stack,
        localization_stack,
        nav2_stack,
        TimerAction(period=20.0, actions=[lifecycle_manager]), # Wait for nodes to load before waking them
        custom_logic,
        gps_bridge_node,
        sensor_pub_node,
    ])