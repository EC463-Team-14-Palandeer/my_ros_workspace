import shutil
import subprocess
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


DEFAULTS = {
    "image_topic": "/camera/camera/color/image_raw",
    "rtsp_url": "rtsp://127.0.0.1:8554/robot_raw",
    "width": 0,
    "height": 0,
    "fps": 30,
    "codec": "libx264",
    "preset": "ultrafast",
    "tune": "zerolatency",
    "bitrate": "2M",
    "gop": 30,
    "rtsp_transport": "tcp",
    "log_ffmpeg_stderr": False,
    "restart_on_failure": True,
}


class RisGo2RtcNode(Node):
    def __init__(self):
        super().__init__("ris_go2rtc_node")
        self._declare_parameters()

        self.image_topic = self.get_parameter("image_topic").value
        self.rtsp_url = self.get_parameter("rtsp_url").value
        self.configured_width = int(self.get_parameter("width").value)
        self.configured_height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.codec = self.get_parameter("codec").value
        self.preset = self.get_parameter("preset").value
        self.tune = self.get_parameter("tune").value
        self.bitrate = self.get_parameter("bitrate").value
        self.gop = int(self.get_parameter("gop").value)
        self.rtsp_transport = self.get_parameter("rtsp_transport").value
        self.log_ffmpeg_stderr = bool(self.get_parameter("log_ffmpeg_stderr").value)
        self.restart_on_failure = bool(self.get_parameter("restart_on_failure").value)

        self.process: Optional[subprocess.Popen] = None
        self.stream_width: Optional[int] = None
        self.stream_height: Optional[int] = None
        self._logged_missing_ffmpeg = False

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )
        self.get_logger().info(
            f"RIS go2rtc streamer subscribed to {self.image_topic}, target {self.rtsp_url}"
        )

    def _declare_parameters(self):
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)

    def _resolve_frame_size(self, msg: Image) -> tuple[int, int]:
        width = self.configured_width or int(msg.width)
        height = self.configured_height or int(msg.height)
        return width, height

    def _build_ffmpeg_command(self, width: int, height: int) -> list[str]:
        return [
            "ffmpeg",
            "-loglevel",
            "warning",
            "-nostdin",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-f",
            "rawvideo",
            "-vcodec",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{width}x{height}",
            "-r",
            str(self.fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            self.codec,
            "-preset",
            self.preset,
            "-tune",
            self.tune,
            "-b:v",
            self.bitrate,
            "-g",
            str(self.gop),
            "-f",
            "rtsp",
            "-rtsp_transport",
            self.rtsp_transport,
            self.rtsp_url,
        ]

    def _image_to_bgr24(self, msg: Image) -> bytes:
        if msg.height <= 0 or msg.width <= 0:
            raise ValueError("Image has invalid dimensions.")

        encoding = msg.encoding.lower()
        if encoding not in {"bgr8", "rgb8", "bgra8", "rgba8"}:
            raise ValueError(f"Unsupported image encoding '{msg.encoding}'.")

        channels = 4 if encoding.endswith("a8") else 3
        min_step = msg.width * channels
        if msg.step < min_step:
            raise ValueError(
                f"Image step {msg.step} is smaller than expected row size {min_step}."
            )

        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = data.reshape((msg.height, msg.step))
        pixels = frame[:, :min_step].reshape((msg.height, msg.width, channels))

        if encoding == "bgr8":
            bgr_pixels = pixels
        elif encoding == "rgb8":
            bgr_pixels = pixels[:, :, ::-1]
        elif encoding == "bgra8":
            bgr_pixels = pixels[:, :, :3]
        else:
            bgr_pixels = pixels[:, :, [2, 1, 0]]

        return np.ascontiguousarray(bgr_pixels).tobytes()

    def _start_ffmpeg(self, width: int, height: int) -> bool:
        ffmpeg_path = shutil.which("ffmpeg")
        if ffmpeg_path is None:
            if not self._logged_missing_ffmpeg:
                self.get_logger().error("ffmpeg not found on PATH; streamer is idle.")
                self._logged_missing_ffmpeg = True
            return False

        command = self._build_ffmpeg_command(width, height)
        stderr_stream = None if self.log_ffmpeg_stderr else subprocess.DEVNULL
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stderr=stderr_stream,
        )
        self.stream_width = width
        self.stream_height = height
        self.get_logger().info(
            f"Started ffmpeg for {width}x{height}@{self.fps} -> {self.rtsp_url}"
        )
        return True

    def _stop_ffmpeg(self):
        if self.process is None:
            return

        try:
            if self.process.stdin:
                self.process.stdin.close()
        except Exception:
            pass

        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)

        self.process = None

    def _ensure_ffmpeg(self, width: int, height: int) -> bool:
        if self.process is not None:
            return True
        return self._start_ffmpeg(width, height)

    def _handle_process_failure(self):
        if self.process is None:
            return

        return_code = self.process.poll()
        if return_code is None:
            return

        self.get_logger().error(f"ffmpeg exited with code {return_code}")
        self._stop_ffmpeg()

    def image_callback(self, msg: Image):
        self._handle_process_failure()

        width, height = self._resolve_frame_size(msg)
        if width <= 0 or height <= 0:
            self.get_logger().error("Invalid stream dimensions; set width/height parameters.")
            return

        if (
            self.stream_width is not None
            and self.stream_height is not None
            and (width != self.stream_width or height != self.stream_height)
        ):
            self.get_logger().warn(
                "Incoming frame size changed; restarting ffmpeg with new dimensions."
            )
            self._stop_ffmpeg()

        if not self._ensure_ffmpeg(width, height):
            return

        try:
            frame_bytes = self._image_to_bgr24(msg)
        except ValueError as exc:
            self.get_logger().error(f"Failed to normalize ROS image: {exc}")
            return

        if self.process is None or self.process.stdin is None:
            self.get_logger().error("ffmpeg stdin is unavailable.")
            return

        try:
            self.process.stdin.write(frame_bytes)
            self.process.stdin.flush()
        except BrokenPipeError:
            self.get_logger().error("ffmpeg pipe broke while writing a frame.")
            self._stop_ffmpeg()
            if self.restart_on_failure:
                self._ensure_ffmpeg(width, height)
        except Exception as exc:
            self.get_logger().error(f"Failed to forward frame to ffmpeg: {exc}")
            self._stop_ffmpeg()

    def destroy_node(self):
        self._stop_ffmpeg()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RisGo2RtcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
