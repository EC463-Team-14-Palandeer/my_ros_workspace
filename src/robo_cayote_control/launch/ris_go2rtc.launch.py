from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    rtsp_url = LaunchConfiguration("rtsp_url")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    codec = LaunchConfiguration("codec")
    preset = LaunchConfiguration("preset")
    tune = LaunchConfiguration("tune")
    bitrate = LaunchConfiguration("bitrate")
    gop = LaunchConfiguration("gop")
    rtsp_transport = LaunchConfiguration("rtsp_transport")
    log_ffmpeg_stderr = LaunchConfiguration("log_ffmpeg_stderr")
    restart_on_failure = LaunchConfiguration("restart_on_failure")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "rtsp_url",
                default_value="rtsp://127.0.0.1:8554/robot_raw",
            ),
            DeclareLaunchArgument("width", default_value="0"),
            DeclareLaunchArgument("height", default_value="0"),
            DeclareLaunchArgument("fps", default_value="30"),
            DeclareLaunchArgument("codec", default_value="libx264"),
            DeclareLaunchArgument("preset", default_value="ultrafast"),
            DeclareLaunchArgument("tune", default_value="zerolatency"),
            DeclareLaunchArgument("bitrate", default_value="2M"),
            DeclareLaunchArgument("gop", default_value="30"),
            DeclareLaunchArgument("rtsp_transport", default_value="tcp"),
            DeclareLaunchArgument("log_ffmpeg_stderr", default_value="false"),
            DeclareLaunchArgument("restart_on_failure", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robo_cayote_control",
                executable="ris_go2rtc_node",
                name="ris_go2rtc_node",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "rtsp_url": rtsp_url,
                        "width": ParameterValue(width, value_type=int),
                        "height": ParameterValue(height, value_type=int),
                        "fps": ParameterValue(fps, value_type=int),
                        "codec": codec,
                        "preset": preset,
                        "tune": tune,
                        "bitrate": bitrate,
                        "gop": ParameterValue(gop, value_type=int),
                        "rtsp_transport": rtsp_transport,
                        "log_ffmpeg_stderr": ParameterValue(
                            log_ffmpeg_stderr,
                            value_type=bool,
                        ),
                        "restart_on_failure": ParameterValue(
                            restart_on_failure,
                            value_type=bool,
                        ),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            ),
        ]
    )
