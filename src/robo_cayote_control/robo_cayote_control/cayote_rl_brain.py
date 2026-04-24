#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from stable_baselines3 import SAC
import sys
import numpy.core
from functools import partial
import math

# The "Envelope" Hack for NumPy compatibility
sys.modules['numpy._core'] = sys.modules['numpy.core']

class CayoteInferenceNode(Node):
    def __init__(self):
        super().__init__('cayote_rl_brain')
        
        self.get_logger().info("Loading SAC Model into Jetson Memory...")
        try:
            custom_objects = {
                "lr_schedule": lambda _: 0.0,
                "learning_rate": 0.0
            }
            
            self.model = SAC.load("/workspaces/isaac_ros-dev/src/robo_cayote_control/models/sac_cayote_v1_compat", custom_objects=custom_objects)
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            sys.exit(4) 
            
        self.get_logger().info("Model Loaded Successfully!")
        
        # --- STATE VARIABLES ---
        self.found = 0.0
        self.offset = 0.0
        self.human_area_norm = 0.0
        
        # New Odometry & Route Variables
        self.current_speed = 0.0
        self.current_steer = 0.0
        self.target_distance = 0.0
        self.target_angle = 0.0
        
        # New Costmap Variable (Starts empty to prevent premature inference)
        self.latest_costmap_array = np.zeros(0, dtype=np.float32)
        
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

        # New Subscribers for Nav2 and EKF data
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/local', self.odom_callback, 10)
            
        self.plan_sub = self.create_subscription(
            Path, '/local_plan', self.plan_callback, 10)
            
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10)
        
        # Main Loop
        self.timer = self.create_timer(0.1, self.inference_loop)
        
    # --- CALLBACKS ---
    def yolo_callback(self, msg):
        self.found = msg.data[0]
        self.offset = msg.data[1]
        self.human_area_norm = msg.data[2] / 10000.0 
        
    def sonar_callback(self, msg, topic_name):
        self.sonar_ranges[topic_name] = msg.range

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x
        self.current_steer = msg.twist.twist.angular.z

    def plan_callback(self, msg):
        # Look ahead for a "carrot" waypoint to follow
        if len(msg.poses) > 10:
            target_pose = msg.poses[10].pose.position
            self.target_distance = math.sqrt(target_pose.x**2 + target_pose.y**2)
            self.target_angle = math.atan2(target_pose.y, target_pose.x)
        elif len(msg.poses) > 0:
            # Fallback to the last available pose if the path is very short
            target_pose = msg.poses[-1].pose.position
            self.target_distance = math.sqrt(target_pose.x**2 + target_pose.y**2)
            self.target_angle = math.atan2(target_pose.y, target_pose.x)
        else:
            self.target_distance = 0.0
            self.target_angle = 0.0

    def costmap_callback(self, msg):
        # 1. Reshape the 1D tuple into a 2D NumPy array based on costmap dimensions
        grid_2d = np.array(msg.data, dtype=np.float32).reshape(msg.info.height, msg.info.width)
        
        # Treat unknown space (-1) as an obstacle (100) for safety
        grid_2d[grid_2d == -1] = 100.0
        
        # 2. Find the center coordinates (where the robot is)
        center_y = msg.info.height // 2
        center_x = msg.info.width // 2
        
        # 3. Define crop size (10 cells in each direction = 20x20 grid)
        crop = 10 
        
        # 4. Safely calculate array bounds to avoid indexing errors
        y1 = max(0, center_y - crop)
        y2 = min(msg.info.height, center_y + crop)
        x1 = max(0, center_x - crop)
        x2 = min(msg.info.width, center_x + crop)
        
        # Slice the array
        sliced = grid_2d[y1:y2, x1:x2]
        
        # Create a guaranteed 20x20 grid (handles edge case if costmap is smaller than 20x20)
        mini_grid = np.zeros((crop * 2, crop * 2), dtype=np.float32)
        shape_y, shape_x = sliced.shape
        mini_grid[0:shape_y, 0:shape_x] = sliced
        
        # 5. Flatten to 1D and normalize from [0, 100] to [0.0, 1.0]
        self.latest_costmap_array = mini_grid.flatten() / 100.0

    # --- MAIN LOOP ---
    def inference_loop(self):
        # Do not run inference if the costmap hasn't populated yet
        if len(self.latest_costmap_array) == 0:
            return
            
        # 1. Build Base Observation Vector (Size: 13)
        base_obs = np.array([
            self.found, 
            self.offset, 
            self.human_area_norm,  #These first three are wrong lmao :DDDD
            self.sonar_ranges['/ultrasound/front'],
            self.sonar_ranges['/ultrasound/front_left'],
            self.sonar_ranges['/ultrasound/front_right'],
            self.sonar_ranges['/ultrasound/rear'],
            self.sonar_ranges['/ultrasound/rear_left'],
            self.sonar_ranges['/ultrasound/rear_right'],
            self.current_speed,
            self.current_steer,
            self.target_distance,
            self.target_angle
        ], dtype=np.float32)
        
        # 2. Concatenate with Costmap Data (13 + 400 = 413 Total Elements)
        obs = np.concatenate((base_obs, self.latest_costmap_array))
        
        # 3. Predict
        action, _states = self.model.predict(obs, deterministic=True)
        
        # 4. Scale Outputs (Adjust multipliers to match your gym environment)
        speed = float(action[0]) * 10.0
        steer = float(action[1]) * 0.4
            
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