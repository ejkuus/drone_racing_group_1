#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

class FlyToGateNode(Node):

    def __init__(self):
        super().__init__('fly_to_gate_node')
        self.get_logger().info("FlyToGateNode started")

        self.image_width = 960
        self.image_height = 720
        self.center_x = self.image_width / 2
        self.center_y = self.image_height / 2
        self.tolerance = 10  # px

        self.forward_speed = 0.1

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            Point,
            '/goal_position',
            self.goal_position_callback,
            qos_profile
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)

    def goal_position_callback(self, msg: Point):
        if msg.x == -1 and msg.y == -1:
            self.get_logger().info("Gate lost, flying forward to clear the gate...")
            twist = Twist()
            twist.linear.x = self.forward_speed
            self.cmd_vel_pub.publish(twist)
            time.sleep(2)
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            self.get_logger().info("Stopped after forward movement.")
            return

        error_x = abs(msg.x - self.center_x)
        error_y = abs(msg.y - self.center_y)

        if error_x <= self.tolerance and error_y <= self.tolerance:
            twist = Twist()
            twist.linear.x = self.forward_speed
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Center aligned! Sent forward command.")
        else:
            self.get_logger().info("Not centered yet.")

def main(args=None):
    rclpy.init(args=args)
    node = FlyToGateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
