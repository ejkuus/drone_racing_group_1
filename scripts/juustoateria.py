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
        self.center_y = self.image_height / 2 - 180
        self.tolerance = 50

        # Liikeparametrit
        self.forward_speed = 0.1
        self.k_t = 0.6
        self.fov_horizontal = math.radians(69.4)

        # ROS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(Point, '/goal_position', self.goal_position_callback, qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # Tilat
        self.seeing_gate = False
        self.forward_timer = None
        self.recovery_timer = None
        self.recovery_step = 0
        self.temp_error = 0  
        self.current_twist = Twist()

        # Timer
        self.cmd_timer = self.create_timer(0.1, self.cmd_vel_timer_callback)

    def cmd_vel_timer_callback(self):
        self.cmd_vel_pub.publish(self.current_twist)

    def stop_drone(self):
        self.current_twist = Twist()

    def goal_position_callback(self, msg: Point):
        self.seeing_gate = not (msg.x == -1 and msg.y == -1)

        if self.seeing_gate:
            self.get_logger().info("Gate visible")
            error_x = abs(msg.x - self.center_x)
            error_y = abs(msg.y - self.center_y)
            self.temp_error = 0

            if error_x <= self.tolerance and error_y <= self.tolerance:
                twist = Twist()
                twist.linear.x = self.forward_speed
                self.current_twist = twist
                self.get_logger().info("Gate centered — moving forward")
                self.cmd_vel_pub.publish(self.current_twist)
            else:
                self.stop_drone()

        else:
            if self.temp_error == 0:
                self.get_logger().info("Gate not visible — moving forward blindly")
                self.start_forward_timer()
                self.temp_error = 1
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
        self.current_twist = twist

        self.get_logger().info(f"Flying forward blindly for {forward_duration:.2f} sec")

        self.forward_timer = self.create_timer(forward_duration, self.forward_timer_callback)

    def forward_timer_callback(self):
        if self.forward_timer is not None:
            self.forward_timer.cancel()
            self.forward_timer = None

        self.stop_drone()
        self.recovery_behavior()

    def recovery_behavior(self):
        if self.recovery_timer is not None:
            self.recovery_timer.cancel()
            self.recovery_timer = None

        if self.seeing_gate:
            self.get_logger().info("Gate found, no recovery needed.")
            return

        self.recovery_step += 1
        if self.recovery_step == 1:
            # Look left
            self.rotate(angle=0.9 * self.fov_horizontal, speed=0.2, direction='left')
        elif self.recovery_step == 2:
            # Look far right
            self.rotate(angle=1.9 * self.fov_horizontal, speed=0.2, direction='right')
        else:
            # Spin slowly
            twist = Twist()
            twist.angular.z = 0.2 
            self.current_twist = twist
            self.get_logger().info("Spinning slowly to find gate")

    def rotate(self, angle, speed, direction):
        twist = Twist()
        if direction == 'left':
            twist.angular.z = speed
        else:
            twist.angular.z = -speed

        self.current_twist = twist
        duration = angle / speed

        self.get_logger().info(f"Rotating {direction} for {duration:.2f} sec")
        self.recovery_timer = self.create_timer(duration, self.stop_after_rotation)

    def stop_after_rotation(self):
        if self.recovery_timer is not None:
            self.recovery_timer.cancel()
            self.recovery_timer = None
        self.stop_drone()

    def stop_motion(self):
        self.stop_drone()


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
