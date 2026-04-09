import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import re
import time

class GpsUsbPublisher(Node):
    def __init__(self):
        super().__init__('gps_usb_publisher')
        
        # Publisher for the GPS fix
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        
        # Regex Pattern to find Lat, Lon, and Alt in your specific string format
        self.gps_pattern = re.compile(
            r"Lat:\s*(?P<lat>[-+]?\d*\.\d+|\d+).*?"
            r"Lon:\s*(?P<lon>[-+]?\d*\.\d+|\d+).*?"
            r"Alt:\s*(?P<alt>[-+]?\d*\.\d+|\d+)"
        )

        # Serial Setup
        # Note: GPS modules are often /dev/ttyUSB0 or /dev/ttyACM1 
        # Baud rate is usually 9600 or 115200 for these modules.
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
            self.get_logger().info("Successfully connected to GPS via Serial.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to GPS: {e}")

        # Timer at 1Hz (GPS usually updates once per second)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Check if serial is connected and has data waiting
        if hasattr(self, 'ser') and self.ser.in_waiting > 0:
            try:
                # Read line and decode
                line = self.ser.readline().decode('utf-8').strip()
                
                # Search for the GPS pattern in the line
                match = self.gps_pattern.search(line)
                
                if match:
                    # Convert text matches to floats
                    lat_val = float(match.group('lat'))
                    lon_val = float(match.group('lon'))
                    alt_val = float(match.group('alt'))

                    msg = NavSatFix()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'gps' # MUST match your URDF/TF link name
                    
                    msg.latitude = lat_val
                    msg.longitude = lon_val
                    msg.altitude = alt_val

                    # Set Covariance (Standard 1m accuracy for EKF)
                    msg.position_covariance = [1.0, 0.0, 0.0, 
                                               0.0, 1.0, 0.0, 
                                               0.0, 0.0, 1.0]
                    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published GPS: {lat_val}, {lon_val}")
                
            except Exception as e:
                self.get_logger().error(f"Error processing GPS data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GpsUsbPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser'):
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()