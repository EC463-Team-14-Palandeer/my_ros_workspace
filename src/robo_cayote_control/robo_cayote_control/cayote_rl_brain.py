#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Range
from stable_baselines3 import SAC
import sys
import numpy.core
from functools import partial

# The "Envelope" Hack for NumPy compatibility
sys.modules['numpy._core'] = sys.modules['numpy.core']

class CayoteInferenceNode(Node):
    def __init__(self):
        super().__init__('cayote_rl_brain')
        
        self.get_logger().info("Loading SAC Model into Jetson Memory...")
        try:
            self.model = SAC.load("/workspaces/isaac_ros-dev/src/robo_cayote_control/models/sac_cayote_v1_compat")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            sys.exit(4) 
            
        self.get_logger().info("Model Loaded Successfully!")
        
        # --- STATE VARIABLES ---
        self.found = 0.0
        self.offset = 0.0
        self.human_area_norm = 0.0
        
        # Define the exact 6 sensors used in training
        self.sonar_topics = [
            '/ultrasound/front', '/ultrasound/front_left', '/ultrasound/front_right',
            '/ultrasound/rear', '/ultrasound/rear_left', '/ultrasound/rear_right'
        ]
        # Initialize dictionary to hold readings
        self.sonar_ranges = {topic: 10.0 for topic in self.sonar_topics}
        
        # --- PUBLISHER ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- SUBSCRIBERS ---
        self.yolo_sub = self.create_subscription(
            Float32MultiArray, '/yolo/human_data', self.yolo_callback, 10)
            
        for topic in self.sonar_topics:
            self.create_subscription(
                Range, 
                topic, 
                partial(self.sonar_callback, topic_name=topic), 
                10)
        
        self.timer = self.create_timer(0.1, self.inference_loop)
        
    def yolo_callback(self, msg):
        self.found = msg.data[0]
        self.offset = msg.data[1]
        # Ensure normalization matches training (area / 10000.0)
        self.human_area_norm = msg.data[2] / 10000.0 
        
    def sonar_callback(self, msg, topic_name):
        self.sonar_ranges[topic_name] = msg.range
        
    def inference_loop(self):
        # 1. Build Observation Vector
        # Order MUST match the order used during training. 
        # Assuming: [found, offset, area, f, f_l, f_r, r, r_l, r_r]
        obs = np.array([
            self.found, 
            self.offset, 
            self.human_area_norm,
            self.sonar_ranges['/ultrasound/front'],
        ], dtype=np.float32)
        
        # 2. Predict
        action, _states = self.model.predict(obs, deterministic=True)
        
        # 3. Scale Outputs (Adjust multipliers to match your gym environment)
        speed = float(action[0]) * 10.0
        steer = float(action[1]) * 0.4
        
        # # 4. Critical Safety Override (Using front 3 sensors)
        # min_front_dist = min(
        #     self.sonar_ranges['/ultrasound/front'],
        #     self.sonar_ranges['/ultrasound/front_left'],
        #     self.sonar_ranges['/ultrasound/front_right']
        # )
        
        # if min_front_dist < 0.45 and speed > 0:
        #     speed = 0.0
        #     self.get_logger().warn(f"Safety Override: Obstacle at {min_front_dist:.2f}m. Braking!")
            
        # 5. Publish Twist
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steer
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CayoteInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cayote Brain Shutting Down safely.")
    finally:
        # Emergency stop on shutdown
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()