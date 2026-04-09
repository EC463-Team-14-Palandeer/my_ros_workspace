import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robo_cayote_control")
    mqtt_config_path = os.path.join(pkg_share, "config", "mqtt_params.yaml")

    return LaunchDescription(
        [
            Node(
                package="robo_cayote_control",
                executable="mqtt_ack_node",
                name="mqtt_ack_node",
                output="screen",
                parameters=[mqtt_config_path],
            )
        ]
    )
