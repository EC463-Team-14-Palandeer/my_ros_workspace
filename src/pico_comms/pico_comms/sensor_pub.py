import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import json

class PicoPublisher(Node):
    def __init__(self):
        super().__init__('pico_publisher')
        
        # --- NEW: Define the specifications for the two sensor types ---
        self.sensor_specs = {
            'mb1010': {
                'fov': 0.785,       # Approx 45 degrees in radians
                'min_range': 0.15,  # 15cm (MB1010 reports 6 inches minimum)
                'max_range': 6.45   # 6.45m max range
            },
            'hc_sr04': {
                'fov': 0.261,       # Approx 15 degrees in radians
                'min_range': 0.02,  # 2cm
                'max_range': 4.0    # 4m max range
            }
        }
        
        # Updated config to include the sensor type as the 3rd item in the tuple
        self.sensor_config = {
            'front': ('ultrasound/front', 'ultrasonic_front_link', 'mb1010'),
            'front_left':  ('ultrasound/front_left',  'ultrasonic_front_left_link', 'mb1010'),
            'front_right':  ('ultrasound/front_right',  'ultrasonic_front_right_link', 'mb1010'),
            'side_right_front': ('ultrasound/side_right_front', 'ultrasonic_side_right_front_link', 'hc_sr04'),
            'side_right_back':  ('ultrasound/side_right_back',  'ultrasonic_side_right_back_link', 'hc_sr04'),
            'side_left_front':  ('ultrasound/side_left_front',  'ultrasonic_side_left_front_link', 'hc_sr04'),
            'side_left_back': ('ultrasound/side_left_back', 'ultrasonic_side_left_back_link', 'hc_sr04'),
            'rear':  ('ultrasound/rear',  'ultrasonic_rear_link', 'mb1010'),
            'rear_left':  ('ultrasound/rear_left',  'ultrasonic_rear_left_link', 'mb1010'),
            'rear_right':  ('ultrasound/rear_right',  'ultrasonic_rear_right_link', 'mb1010')
        }
        
        # Create a dictionary of publishers dynamically
        self.publishers_ = {}
        for key, (topic, frame, sensor_type) in self.sensor_config.items():
            self.publishers_[key] = self.create_publisher(Range, topic, 10)
        
        # Serial Setup
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
            self.get_logger().info("Successfully connected to Pico via Serial.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Pico: {e}")

        # Timer at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if hasattr(self, 'ser') and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                data = json.loads(line) 
                
                # We assume the Pico sends a JSON dictionary like:
                # {"front": 1.2, "side_right_front": 0.5, "rear": 2.1, ...}
                
                now = self.get_clock().now().to_msg()
                
                # Loop through every sensor reported in the JSON
                for key, distance_value in data.items():
                    # Check if the sensor key exists in our config
                    if key in self.sensor_config:
                        topic, frame_id, sensor_type = self.sensor_config[key]
                        specs = self.sensor_specs[sensor_type]
                        
                        msg = Range()
                        msg.header.stamp = now
                        msg.header.frame_id = frame_id
                        
                        # Apply specific sensor specs
                        msg.radiation_type = Range.ULTRASOUND
                        msg.field_of_view = specs['fov']
                        msg.min_range = specs['min_range']
                        msg.max_range = specs['max_range']
                        msg.range = float(distance_value)

                        # Publish to the specific publisher for this sensor
                        self.publishers_[key].publish(msg)
                
            except json.JSONDecodeError:
                self.get_logger().warn("Received non-JSON data from Pico")
            except Exception as e:
                self.get_logger().error(f"Error processing Pico data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PicoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()