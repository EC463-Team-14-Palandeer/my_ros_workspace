import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    nvblox_config_path = os.path.join(pkg_share, 'config', 'nvblox_params.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robo_cayote.urdf')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    costmap_config_path = os.path.join(pkg_share, 'config', 'costmap.yaml')
    mqtt_config_path = os.path.join(control_pkg_share, 'config', 'mqtt_params.yaml')
    ris_default_topic = '/camera/camera/color/image_raw'
    ris_default_rtsp = 'rtsp://127.0.0.1:8554/robot_raw'

    # --- Launch Configuration ---
    enable_ris_stream = LaunchConfiguration('enable_ris_stream', default='false')
    ris_image_topic = LaunchConfiguration('ris_image_topic', default=ris_default_topic)
    ris_rtsp_url = LaunchConfiguration('ris_rtsp_url', default=ris_default_rtsp)
    ris_width = LaunchConfiguration('ris_width', default='0')
    ris_height = LaunchConfiguration('ris_height', default='0')
    ris_fps = LaunchConfiguration('ris_fps', default='30')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument('enable_ris_stream', default_value='false', description='Start the RealSense to go2rtc streamer'),
        DeclareLaunchArgument('ris_image_topic', default_value=ris_default_topic, description='ROS image topic forwarded to go2rtc'),
        DeclareLaunchArgument('ris_rtsp_url', default_value=ris_default_rtsp, description='RTSP endpoint published for go2rtc'),
        DeclareLaunchArgument('ris_width', default_value='0', description='Force stream width; 0 uses incoming frame width'),
        DeclareLaunchArgument('ris_height', default_value='0', description='Force stream height; 0 uses incoming frame height'),
        DeclareLaunchArgument('ris_fps', default_value='30', description='Output stream frame rate for ffmpeg'),

        # 0. Intel RealSense Camera Driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_share, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_infra1': 'true',      # Required for Visual SLAM
                'enable_infra2': 'true',      # Required for Visual SLAM
                'enable_gyro': 'false',       # Disable IMU to prevent D435 crash
                'enable_accel': 'false',      # Disable IMU to prevent D435 crash
                'align_depth.enable': 'true',
                'enable_sync': 'true',
            }.items()
        ),

        # 1. State Publishers
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[]
        ),

        # 2. Isaac ROS Perception Stack
        ComposableNodeContainer(
            name='isaac_ros_container',
            namespace='',
            package='rclcpp_components',
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
                        'input_left_camera_frame': 'camera_infra1_optical_frame', # Updated to match RealSense TF
                        'input_right_camera_frame': 'camera_infra2_optical_frame', # Updated to match RealSense TF
                        'input_imu_frame': 'imu',
                        'enable_imu_fusion': False,
                        'enable_rectified_pose': True,
                        'image_jitter_threshold_ms': 75.0,
                    }],
                    remappings=[
                        ('/visual_slam/image_0', '/camera/camera/infra1/image_rect_raw'),
                        ('/visual_slam/camera_info_0', '/camera/camera/infra1/camera_info'),
                        ('/visual_slam/image_1', '/camera/camera/infra2/image_rect_raw'),
                        ('/visual_slam/camera_info_1', '/camera/camera/infra2/camera_info'),
                        ('/visual_slam/imu', '/imu/data')
                    ]
                ),
                ComposableNode(
                    package='nvblox_ros',
                    plugin='nvblox::NvbloxNode',
                    name='nvblox_node',
                    parameters=[nvblox_config_path],
                    remappings=[
                        ('depth/image', '/camera/camera/depth/image_rect_raw'),
                        ('depth/camera_info', '/camera/camera/depth/camera_info'),
                    ]
                ),
            ],
            output='screen',
        ),

        # 3. Localization Stack (EKF & NavSat)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[('odometry/filtered', '/odometry/local')]
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[('odometry/filtered', '/odometry/global')]
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/local'), 
                ('odometry/gps', '/odometry/gps')
            ]
        ),
        
        # 4. Standalone Local Costmap
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            output='screen',
            parameters=[costmap_config_path]
        ),
        
        # Wrapped the Lifecycle Manager in a TimerAction to prevent the crash
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[
                        {'autostart': True},
                        {'node_names': ['costmap/costmap']},
                        {'bond_timeout': 0.0},
                        {'attempt_respawn_reconnection': False}
                    ]
                )
            ]
        ),
        
        Node(
            package='witmotion_ros2',
            executable='witmotion_ros2',
            name='witmotion_imu',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud_rate': 9600,
            }],
            remappings=[
                ('imu', '/imu/data'),
                ('/witmotion_imu/imu', '/imu/data')
            ]
        ),

        # 5. Custom Logic
        Node(
            package='robo_cayote_control',
            executable='mqtt_ack_node',
            name='mqtt_ack_node',
            output='screen',
            parameters=[mqtt_config_path],
        ),
        Node(
            package='robo_cayote_control',
            executable='ris_go2rtc_node',
            name='ris_go2rtc_node',
            output='screen',
            condition=IfCondition(enable_ris_stream),
            parameters=[
                {
                    'image_topic': ris_image_topic,
                    'rtsp_url': ris_rtsp_url,
                    'width': ParameterValue(ris_width, value_type=int),
                    'height': ParameterValue(ris_height, value_type=int),
                    'fps': ParameterValue(ris_fps, value_type=int),
                }
            ],
        ),
        # --- Cayote RL Brain ---
        Node(
            package='robo_cayote_control',
            executable='cayote_rl_brain',
            name='cayote_rl_brain',
            output='screen'
        ),
        
        # 1. YOLO Vision Processor Node
        Node(
            package='robo_cayote_control',
            executable='yolo_processor',
            name='yolo_processor',
            output='screen'
        ),
    ])