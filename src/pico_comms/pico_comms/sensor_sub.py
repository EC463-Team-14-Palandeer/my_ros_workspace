import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class PicoSubscriber(Node):
    def __init__(self):
        super().__init__('pico_subscriber')
        self.subscription = self.create_subscription(
            String, 'pico_data', self.listener_callback, 10)

    def listener_callback(self, msg):
        try:
            # Parse the JSON string
            data = json.loads(msg.data)
            
            # Update these keys to match your Pico's output
            s1 = data.get("front_left", 0.0)
            s2 = data.get("front_right", 0.0)

            self.get_logger().info(f'Received -> Front Left: {s1}, Front Right: {s2}')
            
        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to decode JSON: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PicoSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()