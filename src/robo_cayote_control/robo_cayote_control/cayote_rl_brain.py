#!/workspaces/isaac_ros-dev/sspbh_venv/bin/python3
import sys
# sys.path.insert(0, '/workspaces/isaac_ros-dev/sspbh_venv/lib/python3.10/site-packages')

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
import time
from collections import deque # Added for Frame Stacking to work. 

# The "Envelope" Hack for NumPy compatibility --> Supposedly do not do this; evil command
# sys.modules['numpy._core'] = sys.modules['numpy.core'] # Numpy backwards compatibility thing --> Ultralytics needs new numpy by default, so just have it use the old one.

class CayoteInferenceNode(Node):
    def __init__(self):
        super().__init__('cayote_rl_brain')
        self.declare_parameter('debug_motion', True)
        self.declare_parameter('debug_motion_period', 1.0)
        self.debug_motion = bool(self.get_parameter('debug_motion').value)
        self.debug_motion_period = float(self.get_parameter('debug_motion_period').value)
        self._last_debug_logs = {}
        
        self.get_logger().info("Loading SAC Model into Jetson Memory...")
        try:
            custom_objects = {
                "lr_schedule": lambda _: 0.0,
                "learning_rate": 0.0
            }
            
            self.model = SAC.load("/workspaces/isaac_ros-dev/src/robo_cayote_control/models/sac_cayote_curriculum_490000_steps.zip", custom_objects=custom_objects)
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
        self.latest_costmap_array = np.zeros(0, dtype=np.float32) # Just makes 0s of cost map (Low-key-kir-k-enuinely maybe make it 100s to symbolize full state just in case.)
        
        self.frame_stack = deque([np.zeros(26, dtype=np.float32) for _ in range(4)], maxlen=4) # Add deque that holds 4 frames worth of data. (Initially filled with 0s so no crash happens on accident.)

        # Define the exact 6 sensors used in training --> THESE ARE THE REAL ONES.
        self.sonar_topics = [
            '/ultrasound/front', '/ultrasound/front_left', '/ultrasound/front_right',
            '/ultrasound/rear', '/ultrasound/rear_left', '/ultrasound/rear_right',
        ]
        # Initialize dictionary to hold readings
        self.sonar_ranges = {topic: 10.0 for topic in self.sonar_topics} # Um... are these 10m range? I saw online 6.45... --> Don't know if this code is actually for that either to be honest...
        
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

    def _debug_throttled(self, key, message, period=None):
        if not self.debug_motion:
            return
        now = time.monotonic()
        last = self._last_debug_logs.get(key, 0.0)
        period = self.debug_motion_period if period is None else period
        if now - last < period:
            return
        self._last_debug_logs[key] = now
        self.get_logger().info(message)
        
    # --- CALLBACKS ---
    def yolo_callback(self, msg):
        self.found = msg.data[0]
        self.offset = msg.data[1]
        self.human_area_norm = msg.data[2] / 10000.0 
        
    def sonar_callback(self, msg, topic_name):
        # I changed this slightly. Now if there's a detected infinite or NAN value, then
        # the entire script won't fuck itself to death. 
        val = msg.range
        if math.isnan(val) or math.isinf(val):
            val = 10
        self.sonar_ranges[topic_name] = val

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
        grid_2d[grid_2d == -1] = 100.0 # So we only have a 100 and 0 array.
        
        # 2. Find the center coordinates (where the robot is)
        center_y = msg.info.height // 2
        center_x = msg.info.width // 2
        
        # 3. Define crop size (10 cells in each direction = 20x20 grid) --> Maybe change this lowkirkenuinely to 3x3
        crop = 1 # Changed from 10 to 1 --> 1 cell up, down, left, right.
        
        # 4. Safely calculate array bounds to avoid indexing errors
        y1 = max(0, center_y - crop)
        y2 = min(msg.info.height, center_y + crop)
        x1 = max(0, center_x - crop)
        x2 = min(msg.info.width, center_x + crop)
        
        # Slice the array
        sliced = grid_2d[y1:y2, x1:x2]
        
        # Create a guaranteed 20x20 grid (handles edge case if costmap is smaller than 20x20)
        mini_grid = np.zeros((3, 3), dtype=np.float32) # Changed from 20, 20
        shape_y, shape_x = sliced.shape
        mini_grid[0:shape_y, 0:shape_x] = sliced
        
        # 5. Flatten to 1D and normalize from [0, 100] to [0.0, 1.0]
        self.latest_costmap_array = mini_grid.flatten() / 100.0

    # --- MAIN LOOP ---
    def inference_loop(self):
        # Make sure that a complete 9-element costmap is had before running:
        if len(self.latest_costmap_array) != 9:
            self._debug_throttled(
                'waiting_costmap',
                f"Waiting for /local_costmap/costmap before publishing /cmd_vel "
                f"(costmap_len={len(self.latest_costmap_array)})",
            )
            return
            
        # 1. Build Base Observation Vector (Size: 13) --> NEED TO ADD THIS 4 TIMES EACH TO MAKE SURE WE CAN READ DATA CORRECTLY.
        current_obs = np.array([
            # YOLO:
            self.found, 
            self.offset, 
            self.human_area_norm,  

            # Odomoetry:
            self.current_speed,
            self.current_steer,

            # Route:
            self.target_distance,
            self.target_angle,

            # Sonars:
            # These are the real values of the ultrasonic sensors being read:
            self.sonar_ranges.get('/ultrasound/front', 10),
            self.sonar_ranges.get('/ultrasound/front_left', 10),
            self.sonar_ranges.get('/ultrasound/front_right', 10),
            self.sonar_ranges.get('/ultrasound/rear', 10),
            self.sonar_ranges.get('/ultrasound/rear_left', 10),
            self.sonar_ranges.get('/ultrasound/rear_right', 10),

            # These are fake --> The original RL was trained on side sensors, and is
            # expecing them, so we just make them read max values always:
            10.0, # Side left front
            10.0, # Side left back
            10.0, # Side right front
            10.0, # Side right back

        ], dtype=np.float32)
        

        
        current_frame = np.concatenate((current_obs, self.latest_costmap_array))

        self.frame_stack.append(current_frame) # Add the frame to the list.
        stacked_obs = np.concatenate(self.frame_stack) # Flatten 4 frames into a single array
        
        # 3. Predict
        try:
            action, _states = self.model.predict(stacked_obs, deterministic=True)
        except Exception as exc:
            self.get_logger().error(
                f"RL model prediction failed; no /cmd_vel published. "
                f"obs_shape={stacked_obs.shape} error={exc}"
            )
            return
        
        # 4. Scale Outputs (Adjust multipliers to match your gym environment)
        speed = float(action[0]) * 10.0
        steer = float(action[1]) * 0.4
            
        # 5. Publish Twist
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = steer
        self.cmd_vel_pub.publish(twist)
        self._debug_throttled(
            'published_cmd_vel',
            "Published /cmd_vel "
            f"linear.x={speed:.3f} angular.z={steer:.3f} "
            f"raw_action=({float(action[0]):.3f}, {float(action[1]):.3f}) "
            f"yolo=(found={self.found:.1f}, offset={self.offset:.3f}, area={self.human_area_norm:.3f}) "
            f"route=(distance={self.target_distance:.3f}, angle={self.target_angle:.3f}) "
            f"front_sonar={self.sonar_ranges.get('/ultrasound/front', 10):.3f} "
            f"costmap_minmax=({float(np.min(self.latest_costmap_array)):.2f}, "
            f"{float(np.max(self.latest_costmap_array)):.2f})",
        )

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