#!/usr/bin/env python3
"""Cayote RL brain, V2 (initial raw, costmap-aware).

The V2 brain is a lean successor to :mod:`cayote_rl_brain` (V1). It keeps
the parts of V1 that still make sense on the current robot and drops the
parts that created operational headaches:

Kept from V1:
  - 3 vision scalars from /yolo/human_data (found, offset, area/10000).
  - 6 sonar ranges (front + front_L/R + rear + rear_L/R).
  - 20x20 robot-centered crop of /local_costmap/costmap, normalized to
    [0, 1] and flattened, for a total of 400 costmap cells.
  - The costmap-ready gate that prevents inference from running before
    any OccupancyGrid has arrived.
  - The numpy._core compatibility shim required by stable_baselines3 on
    the Jetson's torch/numpy combo.

Dropped from V1:
  - /odometry/local + /local_plan subscriptions (target_distance /
    target_angle / current_speed / current_steer). These coupled the
    brain to EKF + Nav2 + a live planner, which made it unusable until
    the entire stack was healthy.

Observation layout (total = 409 dim):
  [0:3]     found, offset, area/10000          (from yolo_processor)
  [3:9]     sonars: front, front_L, front_R,
            rear, rear_L, rear_R               (from pico_comms/sensor_pub)
  [9:409]   20x20 flattened normalized costmap (from Nav2 local costmap)

Control contract with :mod:`arduino_motor_driver`:
  Publish raw SAC action on /cmd_vel as geometry_msgs/Twist, with
    linear.x  = action[0] in [-1, 1]
    angular.z = action[1] in [-1, 1]
  The motor driver is responsible for all scaling and char translation.

Model-shape caveat:
  The default `model_path` points at ``sac_cayote_v1_compat`` which was
  trained on a 413-dim observation (this layout + 4 odom/plan fields).
  Loading succeeds but `model.predict` will raise on the first tick due
  to the shape mismatch. Retrain SAC against the 409-dim layout above,
  or override ``model_path`` at launch time.
"""

import math
import sys
from functools import partial

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray

# Compatibility shim: stable_baselines3's unpickler looks for
# `numpy._core` which was renamed in newer numpy builds. Mirror the
# current numpy.core module under the older name *before* importing SB3.
import numpy.core  # noqa: F401  -- ensures numpy.core is populated first
sys.modules.setdefault("numpy._core", sys.modules["numpy.core"])

from stable_baselines3 import SAC  # noqa: E402  -- after the shim


# Observation layout constants.
NUM_VISION = 3
NUM_SONARS = 6
COSTMAP_CROP = 10  # half-width in cells -> 20x20 window
COSTMAP_CELLS = (COSTMAP_CROP * 2) ** 2  # 400
OBS_DIM = NUM_VISION + NUM_SONARS + COSTMAP_CELLS  # 409

SONAR_TOPICS = [
    '/ultrasound/front',
    '/ultrasound/front_left',
    '/ultrasound/front_right',
    '/ultrasound/rear',
    '/ultrasound/rear_left',
    '/ultrasound/rear_right',
]

# Default "clear path" reading for sonars that haven't reported yet.
# Matches the Jetson V3 fallback; avoids the V3 bug where the frame stack
# was pre-filled with zeros (interpreted by the model as imminent crash).
SONAR_CLEAR_M = 10.0


class CayoteRlBrainV2(Node):
    """Costmap-aware SAC inference node publishing raw actions to /cmd_vel."""

    def __init__(self):
        super().__init__('cayote_rl_brain_v2')

        # --- Parameters ---------------------------------------------------
        self.declare_parameter(
            'model_path',
            '/workspaces/isaac_ros-dev/src/robo_cayote_control/models/sac_cayote_v1_compat',
        )
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('yolo_topic', '/yolo/human_data')
        self.declare_parameter('inference_period', 0.1)  # 10 Hz

        self.model_path = self.get_parameter('model_path').value
        costmap_topic = self.get_parameter('costmap_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        yolo_topic = self.get_parameter('yolo_topic').value
        inference_period = float(self.get_parameter('inference_period').value)

        # --- Model load ---------------------------------------------------
        self.get_logger().info(
            f"Loading SAC model from {self.model_path} (expected obs dim {OBS_DIM})"
        )
        try:
            custom_objects = {
                "lr_schedule": lambda _: 0.0,
                "learning_rate": 0.0,
            }
            self.model = SAC.load(self.model_path, custom_objects=custom_objects)
        except Exception as exc:
            self.get_logger().error(f"Failed to load SAC model: {exc}")
            raise
        self.get_logger().info("SAC model loaded.")

        # --- State caches -------------------------------------------------
        self.found = 0.0
        self.offset = 0.0
        self.human_area_norm = 0.0
        self.sonar_ranges = {topic: SONAR_CLEAR_M for topic in SONAR_TOPICS}
        # Empty until first costmap; inference is gated on non-empty.
        self.latest_costmap_flat = np.zeros(0, dtype=np.float32)

        # --- Publisher ----------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # --- Subscriptions ------------------------------------------------
        self.create_subscription(
            Float32MultiArray, yolo_topic, self._yolo_cb, 10
        )
        for topic in SONAR_TOPICS:
            self.create_subscription(
                Range,
                topic,
                partial(self._sonar_cb, topic_name=topic),
                10,
            )
        self.create_subscription(
            OccupancyGrid, costmap_topic, self._costmap_cb, 10
        )

        # --- Inference timer ---------------------------------------------
        self.create_timer(inference_period, self._inference_tick)

        self.get_logger().info(
            f"cayote_rl_brain_v2 ready: {1.0 / inference_period:.1f} Hz, "
            f"obs_dim={OBS_DIM}, costmap_topic={costmap_topic}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _yolo_cb(self, msg: Float32MultiArray):
        """Expect ``[found, offset, area]`` from :mod:`yolo_processor`.

        Normalize area the same way V1/V3 did (divide by 10 000).
        """
        if len(msg.data) < 3:
            return
        self.found = float(msg.data[0])
        self.offset = float(msg.data[1])
        self.human_area_norm = float(msg.data[2]) / 10000.0

    def _sonar_cb(self, msg: Range, topic_name: str):
        # Guard against NaN/inf sneaking into the obs vector.
        r = float(msg.range)
        if not math.isfinite(r):
            r = SONAR_CLEAR_M
        self.sonar_ranges[topic_name] = r

    def _costmap_cb(self, msg: OccupancyGrid):
        """Crop a 20x20 window around the robot center, normalize, flatten.

        OccupancyGrid convention: values in [0, 100] are confidence of
        occupancy, -1 is unknown. For safety, treat unknown as fully
        occupied (100). Normalize into [0, 1] to match training.
        """
        w = msg.info.width
        h = msg.info.height
        if w == 0 or h == 0:
            return

        grid = np.array(msg.data, dtype=np.float32).reshape(h, w)
        grid[grid == -1] = 100.0

        cy = h // 2
        cx = w // 2
        y1 = max(0, cy - COSTMAP_CROP)
        y2 = min(h, cy + COSTMAP_CROP)
        x1 = max(0, cx - COSTMAP_CROP)
        x2 = min(w, cx + COSTMAP_CROP)

        mini = np.zeros((COSTMAP_CROP * 2, COSTMAP_CROP * 2), dtype=np.float32)
        sliced = grid[y1:y2, x1:x2]
        sy, sx = sliced.shape
        mini[:sy, :sx] = sliced

        self.latest_costmap_flat = (mini.flatten() / 100.0).astype(np.float32)

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------
    def _build_observation(self) -> np.ndarray:
        """Assemble the 409-dim observation vector in the documented order."""
        base = np.array(
            [
                self.found,
                self.offset,
                self.human_area_norm,
                self.sonar_ranges['/ultrasound/front'],
                self.sonar_ranges['/ultrasound/front_left'],
                self.sonar_ranges['/ultrasound/front_right'],
                self.sonar_ranges['/ultrasound/rear'],
                self.sonar_ranges['/ultrasound/rear_left'],
                self.sonar_ranges['/ultrasound/rear_right'],
            ],
            dtype=np.float32,
        )
        return np.concatenate((base, self.latest_costmap_flat)).astype(np.float32)

    def _inference_tick(self):
        """One SAC forward pass; gated on costmap having populated at least once."""
        if self.latest_costmap_flat.size == 0:
            return

        obs = self._build_observation()
        if obs.shape[0] != OBS_DIM:
            self.get_logger().error(
                f"Observation dim mismatch: got {obs.shape[0]}, expected {OBS_DIM}"
            )
            return

        try:
            action, _ = self.model.predict(obs, deterministic=True)
        except Exception as exc:
            self.get_logger().error(
                f"SAC predict failed (likely obs-shape mismatch against "
                f"the loaded model): {exc}"
            )
            return

        speed = float(np.clip(action[0], -1.0, 1.0))
        steer = float(np.clip(action[1], -1.0, 1.0))

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steer
        self.cmd_vel_pub.publish(twist)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def destroy_node(self):
        """Publish a zeroed Twist so the motor driver settles to neutral.

        The motor driver will then also apply its own 'e' on its own
        shutdown, but zeroing here makes the handoff deterministic.
        """
        try:
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CayoteRlBrainV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("cayote_rl_brain_v2 shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
