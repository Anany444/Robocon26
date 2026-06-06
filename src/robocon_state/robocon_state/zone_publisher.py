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
                            
                            # Calculate facing block based on yaw
                            import math
                            try:
                                block_id = int(block_name.replace('block_', ''))
                                COORDS_TO_BLOCK = {
                                    (0, 0): 12, (0, 1): 11, (0, 2): 10,
                                    (1, 0): 9,  (1, 1): 8,  (1, 2): 7,
                                    (2, 0): 6,  (2, 1): 5,  (2, 2): 4,
                                    (3, 0): 3,  (3, 1): 2,  (3, 2): 1
                                }
                                BLOCK_TO_COORDS = {v: k for k, v in COORDS_TO_BLOCK.items()}
                                
                                if block_id in BLOCK_TO_COORDS:
                                    r, c = BLOCK_TO_COORDS[block_id]
                                    q = msg.pose.pose.orientation
                                    t3 = +2.0 * (q.w * q.z + q.x * q.y)
                                    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                                    yaw = math.atan2(t3, t4)
                                    
                                    yaw = (yaw + 2 * math.pi) % (2 * math.pi)
                                    if yaw >= 7 * math.pi / 4 or yaw < math.pi / 4:
                                        dr, dc = 0, 1 # East
                                    elif math.pi / 4 <= yaw < 3 * math.pi / 4:
                                        dr, dc = -1, 0 # North
                                    elif 3 * math.pi / 4 <= yaw < 5 * math.pi / 4:
                                        dr, dc = 0, -1 # West
                                    else:
                                        dr, dc = 1, 0 # South
                                        
                                    nr, nc = r + dr, c + dc
                                    if (nr, nc) in COORDS_TO_BLOCK:
                                        zone_msg.facing_block = f"block_{COORDS_TO_BLOCK[(nr, nc)]}"
                                    else:
                                        zone_msg.facing_block = "none"
                            except Exception:
                                pass
                            
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
