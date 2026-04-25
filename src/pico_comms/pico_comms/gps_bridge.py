#!/usr/bin/env python3
"""GPS bridge node.

TCP ROS2 adapter for the Raspberry Pi GPS process. The real robot no longer
has GPS directly attached to the Jetson as ``/dev/ttyUSB0``; instead, the Pi
publishes a simple newline-delimited TCP stream at ``10.0.0.2:5000`` by
default. This node keeps the ROS side stable by continuing to publish
``sensor_msgs/NavSatFix`` on ``/gps/fix`` for
``robot_localization/navsat_transform_node``.

Responsibilities, ordered by priority:

1. Maintain a TCP client connection to the Raspberry Pi GPS stream using the
   parameters ``gps_host``, ``gps_port``, ``connect_timeout``, and
   ``reconnect_period``. Connection refused, timeout, peer disconnect, and
   other socket errors are logged with throttling and retried forever so launch
   does not crash if the Pi starts late.

2. Parse expected Pi output lines:

       "lat,lon"          publish a valid NavSatFix
       "WAITING_FOR_FIX"  keep running, log throttled status, publish nothing
       ""                 ignore blank lines

   Malformed or non-numeric lines are skipped with throttled warnings.

3. Publish valid fixes on ``/gps/fix`` with frame ID ``gps`` by default. The
   Pi stream does not include altitude, so altitude is published as ``0.0`` and
   the covariance marks vertical position as much less trusted than latitude
   and longitude.

4. Preserve the existing launch contract: ``my_robot_bringup.launch.py`` can
   still start the executable as ``pico_comms/gps_bridge`` and downstream ROS
   nodes continue subscribing to ``/gps/fix`` without remaps.
"""

import time
import socket
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsBridge(Node):
    """TCP client for the Raspberry Pi GPS stream."""

    def __init__(self):
        super().__init__('gps_bridge_node')

        self.declare_parameter('gps_host', '10.0.0.2')
        self.declare_parameter('gps_port', 5000)
        self.declare_parameter('connect_timeout', 5.0)
        self.declare_parameter('reconnect_period', 2.0)
        self.declare_parameter('gps_frame_id', 'gps')

        self.gps_host = self.get_parameter('gps_host').value
        self.gps_port = int(self.get_parameter('gps_port').value)
        self.connect_timeout = float(self.get_parameter('connect_timeout').value)
        self.reconnect_period = float(self.get_parameter('reconnect_period').value)
        self.gps_frame_id = self.get_parameter('gps_frame_id').value

        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        # Network reads live off the ROS executor thread so reconnect delays do
        # not block parameter services, shutdown, or future timers.
        self._shutdown = threading.Event()
        self._last_log_times = {}
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"GPS bridge connecting to {self.gps_host}:{self.gps_port}"
        )

    def _log_throttled(self, level: str, key: str, message: str, period: float = 5.0):
        now = time.monotonic()
        last = self._last_log_times.get(key, 0.0)
        if now - last < period:
            return
        self._last_log_times[key] = now
        getattr(self.get_logger(), level)(message)

    def _run(self):
        while rclpy.ok() and not self._shutdown.is_set():
            try:
                # Use a fresh socket each attempt; after a Pi reboot or network
                # flap, the old connection cannot be trusted.
                with socket.create_connection(
                    (self.gps_host, self.gps_port),
                    timeout=self.connect_timeout,
                ) as sock:
                    self.get_logger().info(
                        f"Connected to GPS stream at {self.gps_host}:{self.gps_port}"
                    )
                    sock.settimeout(self.connect_timeout)
                    # makefile gives us line iteration while preserving socket
                    # timeouts, which matches the Pi's newline-delimited stream.
                    with sock.makefile('r', encoding='utf-8', errors='replace') as stream:
                        for raw_line in stream:
                            if self._shutdown.is_set() or not rclpy.ok():
                                return
                            self._handle_line(raw_line.strip())

                    self._log_throttled(
                        'warn',
                        'gps_socket_closed',
                        "GPS stream closed by peer; reconnecting.",
                    )
            except (ConnectionRefusedError, TimeoutError, OSError) as exc:
                self._log_throttled(
                    'warn',
                    'gps_connect',
                    f"GPS stream unavailable at {self.gps_host}:{self.gps_port}: {exc}",
                )
            except Exception as exc:
                self._log_throttled(
                    'error',
                    'gps_unexpected',
                    f"Unexpected GPS bridge error: {exc}",
                )

            # Event.wait doubles as an interruptible sleep during shutdown.
            self._shutdown.wait(self.reconnect_period)

    def _handle_line(self, line: str):
        if not line:
            return

        if line == 'WAITING_FOR_FIX':
            # No NavSatFix is better than publishing a bogus zero/last-known fix.
            self._log_throttled(
                'info',
                'waiting_for_fix',
                "GPS stream reports WAITING_FOR_FIX.",
                period=10.0,
            )
            return

        # The Pi intentionally sends a tiny protocol: just "lat,lon".
        # Anything else is treated as telemetry noise and skipped.
        parts = [part.strip() for part in line.split(',')]
        if len(parts) != 2:
            self._log_throttled(
                'warn',
                'malformed_gps',
                f"Skipping malformed GPS line: {line!r}",
            )
            return

        try:
            lat_val = float(parts[0])
            lon_val = float(parts[1])
        except ValueError:
            self._log_throttled(
                'warn',
                'bad_gps_float',
                f"Skipping GPS line with non-numeric lat/lon: {line!r}",
            )
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.gps_frame_id
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat_val
        msg.longitude = lon_val
        msg.altitude = 0.0
        # Altitude is absent from the Pi stream. Keep it numerically valid for
        # NavSatFix, but make the covariance tell localization not to trust it.
        msg.position_covariance = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1000000.0,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.publisher_.publish(msg)

    def destroy_node(self):
        self._shutdown.set()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GpsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()