#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloToRLNode(Node):
    def __init__(self):
        super().__init__('yolo_processor')
        
        self.get_logger().info("Loading TensorRT YOLO Engine...")
        # Point this to exactly where your .engine file is saved
        engine_path = '/workspaces/isaac_ros-dev/src/robo_cayote_control/models/best_cloth.onnx'
        self.model = YOLO(engine_path, task='detect') 
        self.bridge = CvBridge()
        
        # Publisher for RL Brain
        self.data_pub = self.create_publisher(Float32MultiArray, '/yolo/human_data', 10)
        
        # Subscriber for Camera
        self.img_sub = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw', # Update if your camera topic is different
            self.image_callback, 
            10)
            
        self.get_logger().info("YOLO to RL Bridge Started and Ready!")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run YOLO inference
        results = self.model(cv_image, imgsz=512, classes=0, conf=0.5, verbose=False)
        
        found = 0.0
        offset = 0.0
        area = 0.0
        
        if len(results[0].boxes) > 0:
            found = 1.0
            box = results[0].boxes[0] 
            
            img_w = cv_image.shape[1]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            center_x = (x1 + x2) / 2
            
            # Normalized offset [-1.0 to 1.0]
            offset = (center_x - (img_w / 2)) / (img_w / 2)
            
            # Bounding box area
            area = (x2 - x1) * (y2 - y1)
            
        # Publish to RL Node
        data_msg = Float32MultiArray()
        data_msg.data = [found, offset, area]
        self.data_pub.publish(data_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloToRLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()