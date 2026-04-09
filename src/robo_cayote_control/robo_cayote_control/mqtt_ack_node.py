import json
import ssl
from typing import Callable

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robo_cayote_control.protocol import (
    normalize_navigation,
    process_incoming_message,
    summarize_estop,
    summarize_navigation,
    validate_estop,
    validate_navigation,
)


DEFAULTS = {
    "broker.host": "6745aa058d8d4607a9ac29fb232b7e25.s1.eu.hivemq.cloud",
    "broker.port": 8883,
    "broker.user": "Jetson-Robo-Cayote",
    "broker.pass": "Idonotlikedeersforreal123!",
    "broker.tls.enabled": True,
    "mqtt.client_id": "robo-cayote-jetson",
    "mqtt.keepalive": 60,
    "bridge.mqtt2ros.estop.mqtt_topic": "robo-cayote/navigation/estop",
    "bridge.mqtt2ros.estop.ros_topic": "/robo_cayote/navigation/estop",
    "bridge.mqtt2ros.navigation.mqtt_topic": "robo-cayote/navigation",
    "bridge.mqtt2ros.navigation.ros_topic": "/robo_cayote/navigation",
    "bridge.ros2mqtt.estop_ack.mqtt_topic": "robo-cayote/navigation/estop_jetson_ack",
    "bridge.ros2mqtt.nav_ack.mqtt_topic": "robo-cayote/navigation_jetson_ack",
}


class MqttAckNode(Node):
    def __init__(self):
        super().__init__("mqtt_ack_node")

        self._declare_config()
        self.estop_ros_topic = self.get_parameter(
            "bridge.mqtt2ros.estop.ros_topic"
        ).value
        self.nav_ros_topic = self.get_parameter(
            "bridge.mqtt2ros.navigation.ros_topic"
        ).value
        self.estop_mqtt_topic = self.get_parameter(
            "bridge.mqtt2ros.estop.mqtt_topic"
        ).value
        self.nav_mqtt_topic = self.get_parameter(
            "bridge.mqtt2ros.navigation.mqtt_topic"
        ).value
        self.estop_ack_mqtt_topic = self.get_parameter(
            "bridge.ros2mqtt.estop_ack.mqtt_topic"
        ).value
        self.nav_ack_mqtt_topic = self.get_parameter(
            "bridge.ros2mqtt.nav_ack.mqtt_topic"
        ).value

        self.estop_cmd_pub = self.create_publisher(String, self.estop_ros_topic, 10)
        self.nav_cmd_pub = self.create_publisher(String, self.nav_ros_topic, 10)

        self.command_routes = {
            self.estop_mqtt_topic: {
                "name": "estop",
                "ros_topic": self.estop_ros_topic,
                "ros_publisher": self.estop_cmd_pub,
                "ack_mqtt_topic": self.estop_ack_mqtt_topic,
                "ros_normalizer": None,
                "validator": validate_estop,
                "summary_builder": summarize_estop,
            },
            self.nav_mqtt_topic: {
                "name": "navigation",
                "ros_topic": self.nav_ros_topic,
                "ros_publisher": self.nav_cmd_pub,
                "ack_mqtt_topic": self.nav_ack_mqtt_topic,
                "ros_normalizer": normalize_navigation,
                "validator": validate_navigation,
                "summary_builder": summarize_navigation,
            },
        }

        self.mqtt_client = self._create_mqtt_client()
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        self._connect_mqtt()

        self.get_logger().info(
            "MQTT handshake node started: broker -> validate -> ROS -> ACK/result."
        )

    def _declare_config(self):
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)

    def _create_mqtt_client(self):
        client_id = self.get_parameter("mqtt.client_id").value
        if hasattr(mqtt, "CallbackAPIVersion"):
            return mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id=client_id,
                protocol=mqtt.MQTTv311,
            )
        return mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)

    def _connect_mqtt(self):
        host = self.get_parameter("broker.host").value
        port = int(self.get_parameter("broker.port").value)
        username = self.get_parameter("broker.user").value
        password = self.get_parameter("broker.pass").value
        keepalive = int(self.get_parameter("mqtt.keepalive").value)
        tls_enabled = bool(self.get_parameter("broker.tls.enabled").value)

        if username:
            self.mqtt_client.username_pw_set(username, password=password)
        if tls_enabled:
            self.mqtt_client.tls_set(
                cert_reqs=ssl.CERT_REQUIRED,
                tls_version=ssl.PROTOCOL_TLS_CLIENT,
            )
            self.mqtt_client.tls_insecure_set(False)

        self.get_logger().info(f"Connecting to MQTT broker {host}:{port}")
        self.mqtt_client.connect(host, port, keepalive=keepalive)
        self.mqtt_client.loop_start()

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties =None):
        if reason_code != 0:
            self.get_logger().error(
                f"Failed to connect to MQTT broker: reason_code={reason_code}"
            )
            return

        for mqtt_topic in self.command_routes:
            client.subscribe(mqtt_topic)
            self.get_logger().info(f"Subscribed to MQTT topic {mqtt_topic}")

    def _on_mqtt_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        self.get_logger().warn(f"Disconnected from MQTT broker: reason_code={reason_code}")

    def _on_mqtt_message(self, client, userdata, message):
        route = self.command_routes.get(message.topic)
        if route is None:
            self.get_logger().warn(f"Ignoring unexpected MQTT topic {message.topic}")
            return

        raw_payload = message.payload.decode("utf-8", errors="replace")
        self.get_logger().info(
            f"[{route['name']}] MQTT message received on {message.topic}"
        )
        self._handle_command(
            raw_payload=raw_payload,
            topic_name=route["name"],
            ros_topic=route["ros_topic"],
            ros_publisher=route["ros_publisher"],
            ack_mqtt_topic=route["ack_mqtt_topic"],
            ros_normalizer=route["ros_normalizer"],
            validator=route["validator"],
            summary_builder=route["summary_builder"],
        )

    def _publish_ros_string(self, publisher, topic_name: str, payload: str):
        msg = String()
        msg.data = payload
        publisher.publish(msg)
        self.get_logger().info(f"Forwarded validated command to ROS topic {topic_name}")

    def _publish_mqtt_json(self, mqtt_topic: str, payload: dict):
        self.mqtt_client.publish(mqtt_topic, json.dumps(payload), qos=1)

    def _normalize_ros_payload(self, raw_payload: str, normalizer):
        if normalizer is None:
            return raw_payload
        payload = json.loads(raw_payload)
        normalizer(payload)
        return json.dumps(payload)

    def _handle_command(
        self,
        raw_payload: str,
        topic_name: str,
        ros_topic: str,
        ros_publisher,
        ack_mqtt_topic: str,
        ros_normalizer,
        validator: Callable[[dict], tuple[bool, str]],
        summary_builder: Callable[[dict], dict],
    ):
        ack_payload, result_payload = process_incoming_message(
            raw_message=raw_payload,
            topic=topic_name,
            validator=validator,
            summary_builder=summary_builder,
        )

        self._publish_mqtt_json(ack_mqtt_topic, ack_payload)
        self.get_logger().info(
            f"[{topic_name}] ACK published to MQTT topic {ack_mqtt_topic}"
        )

        if result_payload["status"] == "ok":
            ros_payload = self._normalize_ros_payload(raw_payload, ros_normalizer)
            self._publish_ros_string(ros_publisher, ros_topic, ros_payload)
            result_payload["ros_topic"] = ros_topic
            self._publish_mqtt_json(ack_mqtt_topic, result_payload)
            self.get_logger().info(
                f"[{topic_name}] OK published to MQTT topic {ack_mqtt_topic}"
            )
            return

        self._publish_mqtt_json(ack_mqtt_topic, result_payload)
        self.get_logger().warn(
            f"[{topic_name}] Error published to MQTT topic {ack_mqtt_topic}: "
            f"{result_payload.get('reason', 'unknown error')}"
        )

    def destroy_node(self):
        try:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttAckNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
