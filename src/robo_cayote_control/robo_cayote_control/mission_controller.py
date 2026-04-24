import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import json
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from robot_localization.srv import FromLL
from std_msgs.msg import Header
from std_msgs.msg import String # Make sure to import this at the top!

class CayoteMissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        self.callback_group = ReentrantCallbackGroup()
        self.navigator = BasicNavigator()
        self.gps_client = self.create_client(FromLL, '/from_ll', callback_group=self.callback_group)
        self.fence_pub = self.create_publisher(PointCloud2, '/geofence_points', 10)
        self.base_pose = None
        
        # ADD THIS SUBSCRIBER: Listen to the MQTT Bridge!
        self.nav_subscriber = self.create_subscription(
            String,
            '/robo_cayote/navigation',
            self.nav_callback,
            10,
            callback_group=self.callback_group
        )

    # ADD THIS CALLBACK FUNCTION
    def nav_callback(self, msg):
        self.get_logger().info("Received new mission data from MQTT bridge!")
        self.process_mqtt_message(msg.data)

    def process_mqtt_message(self, json_str):
        """Triggered when your MQTT bridge gets a message"""
        data = json.loads(json_str)
        self.get_logger().info(f"Processing {data['type']}...")

        # Update Geofence first (Safety first!)
        if data.get('geofence'):
            self.update_geofence(data['geofence'])

        # Handle Path
        if data.get('path'):
            # Capture base pose right before starting the mission
            self.base_pose = self.navigator.get_current_pose()
            self.run_mission(data['path'], data.get('repeat', False))

    def latlng_to_local(self, lat, lng):
        """Converts GPS to Map coordinates (Thread-safe)"""
        while not self.gps_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /from_ll...')
        
        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lng
        
        # Call the service and wait for result
        future = self.gps_client.call_async(req)
        # In a MultiThreadedExecutor, we can just wait for the future
        while not future.done():
            pass
        return future.result().map_point

    def update_geofence(self, coords):
        # ... [Your interpolation logic was perfect, keep it here] ...
        # (Just ensure it calls the updated latlng_to_local)
        pass

    def run_mission(self, path_data, repeat):
        """Executes the path and handles repeating logic"""
        waypoints = []
        for p in path_data:
            local_xy = self.latlng_to_local(p['lat'], p['lng'])
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = local_xy.x
            pose.pose.position.y = local_xy.y
            waypoints.append(pose)
            
            if p.get('returnToBase') and self.base_pose:
                waypoints.append(self.base_pose)

        # Start the loop for repeat logic
        while True:
            self.get_logger().info("Starting Waypoint Following...")
            self.navigator.followWaypoints(waypoints)

            # Wait for completion
            while not self.navigator.isTaskComplete():
                # Check for feedback (optional)
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(f'Executing waypoint: {feedback.current_waypoint}')
                
                # Sleep a bit to not thrash the CPU
                import time
                time.sleep(1)

            # Check result
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Mission completed successfully!')
            else:
                self.get_logger().warn('Mission failed or was canceled.')
                break

            if not repeat:
                break
            self.get_logger().info("Repeating mission in 5 seconds...")
            time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = CayoteMissionController()
    
    # CRITICAL: Must use MultiThreadedExecutor to prevent service deadlocks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()