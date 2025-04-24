#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class CenterAlignmentNode(Node):

    def __init__(self):
        super().__init__('center_alignment_node')
        self.get_logger().info("CenterAlignmentNode started")

        # Kamera ja keskitys
        self.image_width = 960
        self.image_height = 720
        self.center_x = self.image_width / 2
        self.center_y = self.image_height / 2
        self.tolerance = 20

        # Liikeparametrit
        self.forward_speed = 0.1
        self.k_t = 0.6
        self.fov_horizontal = math.radians(69.4)  # radiaaneina

        # ROS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(Point, '/goal_position', self.goal_position_callback, qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)

        # Tilat
        self.forward_timer = None
        self.recovery_timer = None
        self.recovery_step = 0
        self.seeing_gate = False

    def stop_drone(self):
        twist = Twist()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def goal_position_callback(self, msg: Point):
        self.seeing_gate = not (msg.x == -1 and msg.y == -1)
        temp_error = 2
        if self.seeing_gate:
            self.get_logger().info("Gate visible")

            error_x = abs(msg.x - self.center_x)
            error_y = abs(msg.y - self.center_y)
            temp_error = 0

            if error_x <= self.tolerance and error_y <= self.tolerance:
                twist = Twist()
                twist.linear.x = self.forward_speed
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info("Gate centered — moving forward")
        else:
            if temp_error == 0:
                self.get_logger().info("Gate not visible — starting forward")
                self.start_forward_timer()
                temp_error = 1
            else:
                self.get_logger().info("Gate not visible — starting search")
                self.recovery_behavior()

    def start_forward_timer(self):
        if self.forward_timer is not None:
            return
        self.stop_drone()
        forward_duration = self.k_t / self.forward_speed
        twist = Twist()
        twist.linear.x = self.forward_speed
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info(f"Flying forward at {self.forward_speed:.2f} m/s for {forward_duration:.2f} sec")

        self.forward_timer = self.create_timer(forward_duration, self.forward_timer_callback)

    def forward_timer_callback(self):
        self.forward_timer.cancel()
        self.forward_timer = None
        self.recovery_behavior()

    def recovery_behavior(self):
        if self.recovery_timer is not None:
            self.recovery_timer.cancel()
            self.recovery_timer = None

        if self.seeing_gate:
            self.get_logger().info("Gate found, no recovery needed.")
            return

        self.recovery_step += 1
        twist = Twist()

        if self.recovery_step == 1:
            # Kurkkaa nopeasti vasemmalle (0.9 × FOV)
            angle = 0.9 * self.fov_horizontal  # rad
            angular_speed = 0.1  # rad/s
            duration = angle / angular_speed

            self.stop_drone()
            twist.angular.z = angular_speed
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Looking left...")
            self.recovery_timer = self.create_timer(duration, self.recovery_behavior)

        elif self.recovery_step == 2:
            # Käänny oikealle 1.9 × FOV (paljon)
            angle = 1.9 * self.fov_horizontal
            angular_speed = 0.1
            duration = angle / angular_speed

            self.stop_drone()
            twist.angular.z = -angular_speed
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Looking far right...")
            self.recovery_timer = self.create_timer(duration, self.recovery_behavior)

        else:
            # Pyöri hitaasti oikealle kunnes portti löytyy
            self.stop_drone()
            twist.angular.z = -0.3
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Spinning slowly to find next gate")
            # Ei ajastinta, jää pyörimään kunnes portti löytyy

    def stop_motion(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CenterAlignmentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
