#!/usr/bin/env python3
"""Arduino motor driver node.

Thin ROS2 serial adapter that owns the motor-controller Arduino on
``/dev/ttyACM1`` (a second Arduino, separate from the sonar Arduino on
``/dev/ttyACM0`` that :mod:`pico_comms.sensor_pub` owns).

Responsibilities, ordered by priority:

1. Translate ``geometry_msgs/Twist`` messages on ``/cmd_vel`` (raw SAC action
   in ``[-1, 1]`` for both linear.x and angular.z) into the single-character
   commands the Arduino firmware understands::

       speed:  'd' forward  |  's' stop  |  'v' reverse
       steer:  'r' right    |  'f' straight  |  'l' left
       special: 'e' emergency stop

2. Honor three safety gates with this precedence (highest first):

   a. Latched remote e-stop received over ``/robo_cayote/navigation/estop``
      (bridged from MQTT by :mod:`robo_cayote_control.mqtt_ack_node`).
      While latched we ignore ``/cmd_vel`` entirely and re-pulse ``'e'`` on
      the periodic state-push timer.
   b. ``/cmd_vel`` watchdog: if no command arrives within
      ``cmd_vel_timeout`` seconds we fail safe to neutral (``'s' + 'f'``).
   c. Soft front-sonar e-stop: if ``/ultrasound/front`` drops below
      ``emergency_stop_distance`` we zero forward speed before writing.

3. Release the e-stop latch when ``/robo_cayote/navigation/go`` carries a
   ``{"command": "GO", ...}`` payload (or, more rarely, when an estop
   message arrives with ``estop=false``).

4. On shutdown push ``'e'`` explicitly so the firmware's ``emergencyStop()``
   routine neutralizes both the ESC and the steering servo — the original
   Jetson command-center script only sent ``'s' + 'f'`` which left the
   firmware in a "commanded stop" rather than a true emergency-stop state.
"""

import json
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String


# Firmware character protocol (see Arduino sketch `handleCommand`).
CMD_SPEED_FORWARD = 'd'
CMD_SPEED_REVERSE = 'v'
CMD_SPEED_STOP = 's'
CMD_STEER_RIGHT = 'r'
CMD_STEER_LEFT = 'l'
CMD_STEER_STRAIGHT = 'f'
CMD_EMERGENCY_STOP = 'e'


class ArduinoMotorDriver(Node):
    """ROS2 <-> serial bridge for the motor controller Arduino."""

    def __init__(self):
        super().__init__('arduino_motor_driver')

        # --- Parameters ---------------------------------------------------
        # Serial link
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_connect_retry_period', 1.0)
        self.declare_parameter('serial_connect_max_period', 10.0)

        # Twist -> char thresholds. Corrects the Jetson-era bug where the
        # steer thresholds made the 'f' (straight) branch unreachable.
        self.declare_parameter('speed_forward_threshold', 0.3)
        self.declare_parameter('speed_reverse_threshold', -0.3)
        self.declare_parameter('steer_right_threshold', 0.3)
        self.declare_parameter('steer_left_threshold', -0.3)

        # Safety gates
        self.declare_parameter('emergency_stop_distance', 0.45)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('watchdog_period', 0.2)

        # Topic names (parameterized so they can be remapped without edits).
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('front_range_topic', '/ultrasound/front')
        self.declare_parameter('estop_topic', '/robo_cayote/navigation/estop')
        self.declare_parameter('go_topic', '/robo_cayote/navigation/go')

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

        # --- State --------------------------------------------------------
        # Default front range to 10 m (clear path) so we don't trigger the
        # soft e-stop before the first real Range message lands.
        self.front_range = 10.0
        self.estop_active = False
        self.last_cmd_time = self.get_clock().now()
        # Cache of the last (speed_char, steer_char) tuple pushed to the
        # serial line; used to dedupe writes and avoid hammering the UART.
        self._last_chars = (None, None)
        self._serial_lock = threading.Lock()
        self.ser = None

        # --- Serial link --------------------------------------------------
        # Block briefly waiting for the device; never crash the launch if
        # the Arduino enumerates late — just keep retrying in a thread.
        self._serial_thread = threading.Thread(
            target=self._connect_serial_forever, daemon=True
        )
        self._serial_thread.start()

        # --- Subscriptions ------------------------------------------------
        self.create_subscription(
            Twist, p('cmd_vel_topic').value, self._cmd_vel_cb, 10
        )
        self.create_subscription(
            Range, p('front_range_topic').value, self._front_range_cb, 10
        )
        self.create_subscription(
            String, p('estop_topic').value, self._estop_cb, 10
        )
        self.create_subscription(
            String, p('go_topic').value, self._go_cb, 10
        )

        # --- Watchdog timer -----------------------------------------------
        # Runs at `watchdog_period`. If we have no recent /cmd_vel and no
        # active estop, we push a neutral (s, f). If estop is latched we
        # re-pulse 'e' so a transient serial drop still results in a stop.
        self.create_timer(self.watchdog_period, self._watchdog_tick)

        self.get_logger().info(
            f"arduino_motor_driver ready: port={self.serial_port} "
            f"baud={self.baud_rate} estop_topic={p('estop_topic').value} "
            f"go_topic={p('go_topic').value}"
        )

    # ------------------------------------------------------------------
    # Serial lifecycle
    # ------------------------------------------------------------------
    def _connect_serial_forever(self):
        """Open the serial link, retrying forever with exponential backoff.

        Runs in a background thread so rclpy.spin can proceed even if the
        motor Arduino is temporarily absent (e.g. enumerating late, unplugged
        for reflash). While `self.ser is None`, all writes are no-ops.
        """
        delay = self.retry_period
        while rclpy.ok():
            try:
                ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=0.1,
                )
                # Give the Arduino bootloader a moment to settle after
                # the DTR-triggered reset on open.
                time.sleep(2.0)
                with self._serial_lock:
                    self.ser = ser
                self.get_logger().info(
                    f"Connected to motor Arduino on {self.serial_port}"
                )
                return
            except Exception as exc:
                self.get_logger().warn(
                    f"Serial open failed for {self.serial_port}: {exc}. "
                    f"Retrying in {delay:.1f}s"
                )
                time.sleep(delay)
                delay = min(delay * 2.0, self.retry_max)

    def _write_chars(self, chars: str):
        """Write one or more raw chars to the Arduino. Thread-safe, no-op
        when the serial link is not yet open.
        """
        with self._serial_lock:
            if self.ser is None:
                return
            try:
                self.ser.write(chars.encode('ascii'))
            except Exception as exc:
                self.get_logger().error(f"Serial write failed: {exc}")
                # Drop the handle so the reconnect thread can re-open.
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self._last_chars = (None, None)
                self._serial_thread = threading.Thread(
                    target=self._connect_serial_forever, daemon=True
                )
                self._serial_thread.start()

    def _write_pair_if_changed(self, speed: str, steer: str):
        """Push a (speed, steer) char pair only when it changed."""
        if (speed, steer) == self._last_chars:
            return
        self._write_chars(speed + steer)
        self._last_chars = (speed, steer)

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg: Twist):
        """Translate a Twist into the two firmware chars, honoring the
        remote e-stop latch and the soft front-sonar e-stop.
        """
        self.last_cmd_time = self.get_clock().now()
        if self.estop_active:
            # Latched remote estop trumps everything; watchdog re-pulses 'e'.
            return
        speed, steer = self._twist_to_chars(msg.linear.x, msg.angular.z)
        self._write_pair_if_changed(speed, steer)

    def _front_range_cb(self, msg: Range):
        self.front_range = float(msg.range)

    def _estop_cb(self, msg: String):
        """Latch/clear the remote estop based on an MQTT-bridged JSON
        payload of shape ``{"estop": bool, "source": str, "ts": number}``.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Bad estop JSON: {exc}; payload={msg.data!r}")
            return

        if bool(data.get('estop')):
            self.estop_active = True
            # Force an immediate 'e' regardless of the dedupe cache.
            self._write_chars(CMD_EMERGENCY_STOP)
            self._last_chars = (None, None)
            self.get_logger().warn(
                f"Remote ESTOP latched (source={data.get('source')})"
            )
        else:
            if self.estop_active:
                self.get_logger().info(
                    "Remote ESTOP cleared via estop=false payload"
                )
            self.estop_active = False

    def _go_cb(self, msg: String):
        """Release the e-stop latch on a ``{"command": "GO", ...}`` payload."""
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

    # ------------------------------------------------------------------
    # Core translation + watchdog
    # ------------------------------------------------------------------
    def _twist_to_chars(self, linear_x: float, angular_z: float):
        """Map a raw SAC action to the firmware's two-char command language.

        Applies the soft front-sonar e-stop inline: if the front sonar is
        closer than ``emergency_stop_distance`` and the command asked to go
        forward, we clamp the forward component to zero.
        """
        if (
            self.front_range < self.emergency_stop_distance
            and linear_x > 0.0
        ):
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
        """Periodic safety net.

        - If estop is latched, keep re-pulsing 'e' so a transient write
          failure can't leave the motors armed.
        - Else if no /cmd_vel has arrived within ``cmd_vel_timeout``,
          force neutral ('s' + 'f').
        """
        if self.estop_active:
            self._write_chars(CMD_EMERGENCY_STOP)
            self._last_chars = (None, None)
            return

        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_vel_timeout:
            self._write_pair_if_changed(CMD_SPEED_STOP, CMD_STEER_STRAIGHT)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def destroy_node(self):
        """Send an explicit emergency stop before closing the serial line.

        The firmware's 'e' handler zeroes the ESC and centers the steering
        servo in one shot, which is the correct shutdown behavior; 's' + 'f'
        (what the original Jetson script sent) only reaches the same state
        via two separate servo writes and does not invoke `emergencyStop()`.
        """
        try:
            self._write_chars(CMD_EMERGENCY_STOP)
        except Exception:
            pass
        with self._serial_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("arduino_motor_driver shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
