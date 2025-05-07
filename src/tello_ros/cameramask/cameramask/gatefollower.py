#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy

class TelloGateFollower(Node):

    def __init__(self):
        super().__init__('gateFollower_node')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.get_logger().info("Initializing TelloGateFollower...")

        self.acceptable_error = 0.05
        self.image_width = 960
        self.image_height = 720 - 150
        self.twist_obj = Twist()
        self.pub_mira_move = self.create_publisher(Twist, '/cmd_vel', 1)

        self.create_subscription(Point, '/goal_position', self.point_blob_callback, 10)

    def point_blob_callback(self, msg):
        if msg.x < 0 or msg.y < 0:
            self.get_logger().info("Negative coordinates received, skipping movement.")
            return

        k = 5
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0

        norm_x = (msg.x - cx) / cx
        norm_y = (cy - msg.y) / cy

        self.twist_obj = Twist()  # Reset

        self.twist_obj.angular.z = -k * norm_x * 0.1
        self.twist_obj.linear.z = k * norm_y * 0.1

        self.pub_mira_move.publish(self.twist_obj)
        self.get_logger().info("Published twist values.")

def main(args=None):
    rclpy.init(args=args)
    node = TelloGateFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down TelloGateFollower.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
