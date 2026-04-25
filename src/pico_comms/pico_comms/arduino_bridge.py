#!/usr/bin/env python3
"""Arduino bridge node.

Single ROS2 serial adapter that owns the robot Arduino on ``/dev/ttyACM0`` by
default. The current Arduino firmware is a combined device: it reports sonar
JSON to the Jetson and listens for one-byte motor/steering commands on the same
USB serial link. This node replaces the old split between
``pico_comms.sensor_pub`` and
``robo_cayote_control.arduino_motor_driver`` so only one process opens the
Arduino serial port.

Responsibilities:

1. Translate ``geometry_msgs/Twist`` messages on ``/cmd_vel`` (raw SAC action
   in ``[-1, 1]`` for both linear.x and angular.z) into the single-character
   commands the Arduino firmware understands::

       speed:  'd' forward  |  's' stop  |  'v' reverse
       steer:  'r' right    |  'f' straight  |  'l' left
       special: 'e' emergency stop

2. Honor the safety gates:

   a. Latched remote e-stop received over ``/robo_cayote/navigation/estop``
      (bridged from MQTT by :mod:`robo_cayote_control.mqtt_ack_node`).
      While latched, ``/cmd_vel`` is ignored and ``'e'`` is re-pulsed by the
      watchdog timer.
   b. ``/cmd_vel`` watchdog: if no command arrives within
      ``cmd_vel_timeout`` seconds, fail safe to neutral (``'s' + 'f'``).
   c. Soft front-sonar e-stop: if ``/ultrasound/front`` drops below
      ``emergency_stop_distance``, clamp forward speed to zero before writing.

3. Release the e-stop latch when ``/robo_cayote/navigation/go`` carries a
   ``{"command": "GO", ...}`` payload (or when an e-stop message arrives with
   ``estop=false``).

4. Read newline-delimited Arduino JSON of the form
   ``{"front": 1.23, "front_left": 1.20, ...}`` and publish each recognized
   field as ``sensor_msgs/Range`` on the existing ``/ultrasound/*`` topics.
   Topic names, frame IDs, and range metadata are kept compatible with the old
   ``sensor_pub`` node. Side sonar fields are ignored by default because the
   current robot has no reliable side sensors; set ``publish_side_sensors`` to
   true if those sensors are restored.

5. Keep retrying the serial connection if the Arduino is unplugged, late to
   enumerate, or reset by the USB open. Reads and writes share one locked
   serial handle so ROS can safely use the Arduino as a half-duplex command and
   telemetry device.

6. On shutdown push ``'e'`` explicitly so the firmware's ``emergencyStop()``
   routine neutralizes both the ESC and the steering servo.
"""

import json
import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String


CMD_SPEED_FORWARD = 'd'
CMD_SPEED_REVERSE = 'v'
CMD_SPEED_STOP = 's'
CMD_STEER_RIGHT = 'r'
CMD_STEER_LEFT = 'l'
CMD_STEER_STRAIGHT = 'f'
CMD_EMERGENCY_STOP = 'e'

SONAR_CLEAR_M = 10.0

# Keep these specs in one place so the Arduino only has to report distances.
# ROS consumers still get proper Range metadata for each physical sensor type.
SENSOR_SPECS = {
    'mb1010': {
        'fov': 0.785,
        'min_range': 0.15,
        'max_range': 6.45,
    },
    'hc_sr04': {
        'fov': 0.261,
        'min_range': 0.02,
        'max_range': 4.0,
    },
}

# Arduino JSON key -> (ROS topic, TF frame, sensor spec key). The JSON labels
# must match the firmware exactly; the topic/frame names preserve old users.
SENSOR_CONFIG = {
    'front': ('ultrasound/front', 'ultrasonic_front_link', 'mb1010'),
    'front_left': ('ultrasound/front_left', 'ultrasonic_front_left_link', 'mb1010'),
    'front_right': ('ultrasound/front_right', 'ultrasonic_front_right_link', 'mb1010'),
    'side_right_front': (
        'ultrasound/side_right_front',
        'ultrasonic_side_right_front_link',
        'hc_sr04',
    ),
    'side_right_back': (
        'ultrasound/side_right_back',
        'ultrasonic_side_right_back_link',
        'hc_sr04',
    ),
    'side_left_front': (
        'ultrasound/side_left_front',
        'ultrasonic_side_left_front_link',
        'hc_sr04',
    ),
    'side_left_back': (
        'ultrasound/side_left_back',
        'ultrasonic_side_left_back_link',
        'hc_sr04',
    ),
    'rear': ('ultrasound/rear', 'ultrasonic_rear_link', 'mb1010'),
    'rear_left': ('ultrasound/rear_left', 'ultrasonic_rear_left_link', 'mb1010'),
    'rear_right': ('ultrasound/rear_right', 'ultrasonic_rear_right_link', 'mb1010'),
}

SIDE_SENSOR_KEYS = {
    'side_right_front',
    'side_right_back',
    'side_left_front',
    'side_left_back',
}


class ArduinoBridge(Node):
    """ROS2 bridge for sonar JSON input and motor command output."""

    def __init__(self):
        super().__init__('arduino_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_connect_retry_period', 1.0)
        self.declare_parameter('serial_connect_max_period', 10.0)

        self.declare_parameter('speed_forward_threshold', 0.3)
        self.declare_parameter('speed_reverse_threshold', -0.3)
        self.declare_parameter('steer_right_threshold', 0.3)
        self.declare_parameter('steer_left_threshold', -0.3)

        self.declare_parameter('emergency_stop_distance', 0.45)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('watchdog_period', 0.2)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('front_range_topic', '/ultrasound/front')
        self.declare_parameter('estop_topic', '/robo_cayote/navigation/estop')
        self.declare_parameter('go_topic', '/robo_cayote/navigation/go')
        self.declare_parameter('publish_side_sensors', False)

        p = self.get_parameter
        self.serial_port = p('serial_port').value
        self.baud_rate = int(p('baud_rate').value)
        self.retry_period = float(p('serial_connect_retry_period').value)
        self.retry_max = float(p('serial_connect_max_period').value)

        self.speed_forward_threshold = float(p('speed_forward_threshold').value)
        self.speed_reverse_threshold = float(p('speed_reverse_threshold').value)
        self.steer_right_threshold = float(p('steer_right_threshold').value)
        self.steer_left_threshold = float(p('steer_left_threshold').value)

        self.emergency_stop_distance = float(p('emergency_stop_distance').value)
        self.cmd_vel_timeout = float(p('cmd_vel_timeout').value)
        self.watchdog_period = float(p('watchdog_period').value)
        self.publish_side_sensors = bool(p('publish_side_sensors').value)

        # Start clear so the soft e-stop does not block forward test commands
        # before the first front sonar reading arrives.
        self.front_range = SONAR_CLEAR_M
        self.estop_active = False
        self.last_cmd_time = self.get_clock().now()
        # Last pair written to the Arduino. Dedupe prevents repeating neutral
        # commands every watchdog tick and keeps the UART readable in tests.
        self._last_chars = (None, None)
        # One lock protects both read and write access to the shared serial
        # object because this node intentionally owns a single half-duplex link.
        self._serial_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._last_log_times = {}
        self.ser = None

        self.sonar_publishers = {}
        for key, (topic, _frame_id, _sensor_type) in SENSOR_CONFIG.items():
            self.sonar_publishers[key] = self.create_publisher(Range, topic, 10)

        self.create_subscription(
            Twist, p('cmd_vel_topic').value, self._cmd_vel_cb, 10
        )
        # The front Range subscription is still kept even though this node also
        # publishes it; it preserves the old motor-driver parameter contract.
        self.create_subscription(
            Range, p('front_range_topic').value, self._front_range_cb, 10
        )
        self.create_subscription(
            String, p('estop_topic').value, self._estop_cb, 10
        )
        self.create_subscription(
            String, p('go_topic').value, self._go_cb, 10
        )

        # Safety behavior is timer-driven so it still works when /cmd_vel stops.
        self.create_timer(self.watchdog_period, self._watchdog_tick)

        self._serial_thread = threading.Thread(
            target=self._serial_read_loop, daemon=True
        )
        self._serial_thread.start()

        self.get_logger().info(
            f"arduino_bridge ready: port={self.serial_port} baud={self.baud_rate} "
            f"cmd_vel_topic={p('cmd_vel_topic').value}"
        )

    def _log_throttled(self, level: str, key: str, message: str, period: float = 5.0):
        now = time.monotonic()
        last = self._last_log_times.get(key, 0.0)
        if now - last < period:
            return
        self._last_log_times[key] = now
        getattr(self.get_logger(), level)(message)

    def _serial_read_loop(self):
        delay = self.retry_period
        while rclpy.ok() and not self._shutdown.is_set():
            try:
                ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=0.1,
                )
                # Opening USB serial resets many Arduinos via DTR. Wait before
                # trusting reads/writes so bootloader noise does not become JSON.
                time.sleep(2.0)
                with self._serial_lock:
                    self.ser = ser
                    self._last_chars = (None, None)
                self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
                delay = self.retry_period

                while rclpy.ok() and not self._shutdown.is_set():
                    with self._serial_lock:
                        active_ser = self.ser
                    if active_ser is None:
                        break

                    # Firmware emits one JSON object per line. A short timeout
                    # lets shutdown and reconnect checks run without blocking.
                    line = active_ser.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        self._handle_sonar_line(line)

            except Exception as exc:
                self._log_throttled(
                    'warn',
                    'serial_open_or_read',
                    f"Arduino serial unavailable on {self.serial_port}: {exc}. "
                    f"Retrying in {delay:.1f}s",
                    period=2.0,
                )
                self._close_serial()
                time.sleep(delay)
                delay = min(delay * 2.0, self.retry_max)

    def _close_serial(self):
        with self._serial_lock:
            ser = self.ser
            self.ser = None
            # Force the next command to be written after reconnect, even if it
            # matches the last command sent before the serial fault.
            self._last_chars = (None, None)
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _handle_sonar_line(self, line: str):
        # Bad lines are expected during Arduino reset or if command bytes echo
        # during bench testing, so they are throttled rather than spammed.
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            self._log_throttled(
                'warn',
                'bad_sonar_json',
                f"Received non-JSON Arduino sonar line: {line!r}",
            )
            return

        if not isinstance(data, dict):
            self._log_throttled(
                'warn',
                'bad_sonar_type',
                f"Expected Arduino sonar JSON object, got {type(data).__name__}",
            )
            return

        now = self.get_clock().now().to_msg()
        for key, distance_value in data.items():
            if key not in SENSOR_CONFIG:
                continue
            if key in SIDE_SENSOR_KEYS and not self.publish_side_sensors:
                # The current robot has no reliable side sonars. Keep parsing
                # the Arduino packet, but do not feed placeholder side readings
                # into ROS consumers unless the launch explicitly opts in.
                continue

            try:
                distance = float(distance_value)
            except (TypeError, ValueError):
                self._log_throttled(
                    'warn',
                    f'bad_sonar_value_{key}',
                    f"Bad sonar value for {key}: {distance_value!r}",
                )
                continue

            if not math.isfinite(distance):
                self._log_throttled(
                    'warn',
                    f'bad_sonar_finite_{key}',
                    f"Non-finite sonar value for {key}: {distance_value!r}",
                )
                continue

            topic, frame_id, sensor_type = SENSOR_CONFIG[key]
            specs = SENSOR_SPECS[sensor_type]

            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = specs['fov']
            msg.min_range = specs['min_range']
            msg.max_range = specs['max_range']
            msg.range = distance

            self.sonar_publishers[key].publish(msg)
            if topic == 'ultrasound/front':
                # Keep the safety cache in sync immediately; do not wait for
                # our own /ultrasound/front subscription to loop the message.
                self.front_range = distance

    def _write_chars(self, chars: str) -> bool:
        with self._serial_lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(chars.encode('ascii'))
                return True
            except Exception as exc:
                self.get_logger().error(f"Serial write failed: {exc}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                # The reader thread notices the missing handle and reopens it.
                self.ser = None
                self._last_chars = (None, None)
                return False

    def _write_pair_if_changed(self, speed: str, steer: str):
        if (speed, steer) == self._last_chars:
            return
        if self._write_chars(speed + steer):
            self._last_chars = (speed, steer)

    def _cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        if self.estop_active:
            # Keep the e-stop latch authoritative; GO or estop=false must clear
            # it before autonomous/manual velocity commands can move the robot.
            return
        speed, steer = self._twist_to_chars(msg.linear.x, msg.angular.z)
        self._write_pair_if_changed(speed, steer)

    def _front_range_cb(self, msg: Range):
        self.front_range = float(msg.range)

    def _estop_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Bad estop JSON: {exc}; payload={msg.data!r}")
            return

        if bool(data.get('estop')):
            self.estop_active = True
            self._write_chars(CMD_EMERGENCY_STOP)
            self._last_chars = (None, None)
            self.get_logger().warn(
                f"Remote ESTOP latched (source={data.get('source')})"
            )
        else:
            if self.estop_active:
                self.get_logger().info("Remote ESTOP cleared via estop=false payload")
            self.estop_active = False

    def _go_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Bad go JSON: {exc}; payload={msg.data!r}")
            return

        if data.get('command') == 'GO':
            if self.estop_active:
                self.get_logger().info(
                    f"GO received (source={data.get('source')}); "
                    "releasing estop latch"
                )
            self.estop_active = False

    def _twist_to_chars(self, linear_x: float, angular_z: float):
        # The firmware only understands discrete commands, so raw RL actions are
        # thresholded into speed and steering buckets here.
        if self.front_range < self.emergency_stop_distance and linear_x > 0.0:
            linear_x = 0.0

        if linear_x > self.speed_forward_threshold:
            speed = CMD_SPEED_FORWARD
        elif linear_x < self.speed_reverse_threshold:
            speed = CMD_SPEED_REVERSE
        else:
            speed = CMD_SPEED_STOP

        if angular_z > self.steer_right_threshold:
            steer = CMD_STEER_RIGHT
        elif angular_z < self.steer_left_threshold:
            steer = CMD_STEER_LEFT
        else:
            steer = CMD_STEER_STRAIGHT

        return speed, steer

    def _watchdog_tick(self):
        if self.estop_active:
            # Re-pulse e-stop so a transient serial reconnect cannot leave the
            # Arduino in the last non-stop command state.
            self._write_chars(CMD_EMERGENCY_STOP)
            self._last_chars = (None, None)
            return

        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_vel_timeout:
            self._write_pair_if_changed(CMD_SPEED_STOP, CMD_STEER_STRAIGHT)

    def destroy_node(self):
        self._shutdown.set()
        try:
            self._write_chars(CMD_EMERGENCY_STOP)
        except Exception:
            pass
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("arduino_bridge shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
