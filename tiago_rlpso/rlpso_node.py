import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from .rlpso_navigator import RLPSONavigator
import numpy as np

class RLPSONode(Node):
    def __init__(self):
        super().__init__('rlpso_node')
        self.subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.publisher = self.create_publisher(Twist, '/mobile_base_controller/cmd_vel', 10)
        
        self.target_waypoint = None
        self.grid_data = None

    def map_callback(self, msg):
        # Simplify 1D array to 100x100 if necessary
        self.grid_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
        # Example: Start RLPSO to find the next best point towards a fixed goal (10,10)
        navigator = RLPSONavigator(start=[0,0], goal=[10,10], occupancy_grid=self.grid_data)
        self.target_waypoint = navigator.run()
        self.navigate_to_point(self.target_waypoint)

    def navigate_to_point(self, point):
        msg = Twist()
        # Simple proportional controller to move TIAGo
        # In a real scenario, you'd use a local planner or transform this to robot frame
        msg.linear.x = 0.2 
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RLPSONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
