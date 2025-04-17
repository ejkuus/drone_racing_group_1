#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class CenterAlignmentNode(Node):

    def __init__(self):
        super().__init__('center_alignment_node')
        self.get_logger().info("CenterAlignmentNode started")

        # Kameraresoluutio ja toleranssi
        self.image_width = 960
        self.image_height = 720
        self.center_x = self.image_width / 2
        self.center_y = self.image_height / 2
        self.tolerance = 20  # pikseliä

        # Lentoparametrit
        self.forward_speed = 0.1  # m/s
        self.k_t = 0.6            # eteenpäin lennettävä matka (metreinä)

        self.forward_timer = None

        # QoS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Tilaukset ja julkaisijat
        self.subscription = self.create_subscription(
            Point,
            '/goal_position',
            self.goal_position_callback,
            qos_profile
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)

    def goal_position_callback(self, msg: Point):
        if msg.x == -1 and msg.y == -1:
            self.get_logger().info("Green gate lost — flying forward based on speed/distance.")
            self.start_forward_timer()
            return

        error_x = abs(msg.x - self.center_x)
        error_y = abs(msg.y - self.center_y)

        if error_x <= self.tolerance and error_y <= self.tolerance:
            # Portti on keskellä — liikutaan eteenpäin
            twist = Twist()
            twist.linear.x = self.forward_speed
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Center aligned! Sent forward command.")
        else:
            self.get_logger().info("Not centered yet.")

    def start_forward_timer(self):
        if self.forward_timer is not None:
            return  # Älä aloita uutta ajoa jos vanha on käynnissä

        forward_duration = self.k_t / self.forward_speed

        twist = Twist()
        twist.linear.x = self.forward_speed
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info(
            f"Flying forward at {self.forward_speed:.2f} m/s for {forward_duration:.2f} seconds"
        )

        self.forward_timer = self.create_timer(
            forward_duration,
            self.stop_forward_motion
        )

    def stop_forward_motion(self):
        twist = Twist()  # pysäytys
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Forward duration ended, drone stopped.")

        self.forward_timer.cancel()
        self.forward_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = CenterAlignmentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
