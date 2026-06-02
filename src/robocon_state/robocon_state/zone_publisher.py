#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import yaml
from nav_msgs.msg import Odometry
from robocon_interfaces.msg import ZoneState

class ZonePublisher(Node):
    def __init__(self):
        super().__init__('zone_publisher')
        
        self.declare_parameter('yaml_path', '/home/robot/robocon_ws/src/robocon_bringup/config/game_field_map.yaml')
        self.declare_parameter('team', 'red_team')
        
        yaml_path = self.get_parameter('yaml_path').value
        self.team = self.get_parameter('team').value
        
        try:
            with open(yaml_path, 'r') as f:
                self.zones = yaml.safe_load(f)[self.team]
            self.get_logger().info(f"Loaded game field map for {self.team}")
        except Exception as e:
            self.get_logger().error(f"Failed to load yaml: {e}")
            self.zones = {}

        self.publisher_ = self.create_publisher(ZoneState, '/current_zone', 10)
        self.subscription = self.create_subscription(Odometry, '/ground_truth_odom', self.odom_callback, 10)

    def is_inside(self, x, y, bounds):
        return bounds['x_min'] <= x <= bounds['x_max'] and bounds['y_min'] <= y <= bounds['y_max']

    def calculate_slope_height(self, y, slope_data):
        sy, ey = slope_data['start_y'], slope_data['end_y']
        sh, eh = slope_data['start_height'], slope_data['end_height']
        if ey == sy: return sh
        # Clamp y to slope bounds
        y_clamped = max(min(y, max(sy, ey)), min(sy, ey))
        return sh + (y_clamped - sy) / (ey - sy) * (eh - sh)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        zone_msg = ZoneState()
        zone_msg.global_zone = "unknown"
        zone_msg.local_zone = "none"
        zone_msg.expected_height = 0.0
        zone_msg.on_slope = False
        
        if not self.zones:
            self.get_logger().error(f"No zones loaded")
            return

        # Check global zones first
        for global_name, global_data in self.zones.items():
            if self.is_inside(x, y, global_data['full_zone']):
                zone_msg.global_zone = global_name
                zone_msg.expected_height = global_data['full_zone'].get('height', 0.0)
                
                # Check specific sub-zones
                for local_name, local_data in global_data.items():
                    if local_name == 'full_zone' or local_name == 'blocks': continue
                    if self.is_inside(x, y, local_data):
                        zone_msg.local_zone = local_name
                        if 'slope' in local_data:
                            zone_msg.on_slope = True
                            zone_msg.expected_height = self.calculate_slope_height(y, local_data['slope'])
                        elif 'height' in local_data:
                            zone_msg.expected_height = local_data['height']
                        break
                
                # Check meihua blocks if inside forest
                if global_name == 'meihua_forest' and 'blocks' in global_data:
                    for block_name, block_data in global_data['blocks'].items():
                        if self.is_inside(x, y, block_data):
                            zone_msg.local_zone = block_name
                            zone_msg.expected_height = block_data.get('height', 0.0)
                            break
                break

        self.publisher_.publish(zone_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZonePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
