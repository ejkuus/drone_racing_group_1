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
        self.center_y = self.image_height / 2 + 180  # HUOM: keskipiste ylempänä
        self.tolerance = 50  # sallittu virhe pikseleinä

        # Liikeparametrit
        self.forward_speed = 0.1  # m/s
        self.k_t = 0.6            # kuinka pitkään lentää eteenpäin kun portti katoaa
        self.fov_horizontal = math.radians(69.4)

        # ROS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(Point, '/goal_position', self.goal_position_callback, qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Tilat
        self.seeing_gate = False
        self.lost_gate = False
        self.forward_timer = None

    def goal_position_callback(self, msg: Point):
        # Tarkista onko portti näkyvissä
        gate_visible = not (msg.x == -1 and msg.y == -1)

        if gate_visible:
            self.seeing_gate = True
            self.lost_gate = False

            error_x = abs(msg.x - self.center_x)
            error_y = abs(msg.y - self.center_y)

            if error_x <= self.tolerance and error_y <= self.tolerance:
                self.move_forward()
                self.get_logger().info("Gate centered — moving forward")
            else:
                self.stop_motion()
                self.get_logger().info("Gate visible but not centered — waiting")
        else:
            if self.seeing_gate:
                self.seeing_gate = False
                self.lost_gate = True
                self.get_logger().info("Gate lost — flying forward briefly")
                self.start_forward_timer()
            elif self.lost_gate:
                # Jos jo menetetty, pyöri hitaasti
                self.spin_right()

    def move_forward(self):
        twist = Twist()
        twist.linear.x = self.forward_speed
        self.cmd_vel_pub.publish(twist)

    def stop_motion(self):
        twist = Twist()
        twist.linear.x = 0.0         # Ainoastaan eteenpäin nopeus nollataan
        twist.angular.z = 0.0         # Ainoastaan pyörimisnopeus nollataan
        self.cmd_vel_pub.publish(twist)


    def spin_right(self):
        twist = Twist()
        twist.angular.z = -0.  # rad/s oikealle
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Spinning right to search for gate")

    def start_forward_timer(self):
        if self.forward_timer is None:
            duration = 1.5  # Kuinka kauan lentää eteenpäin
            self.move_forward()
            self.forward_timer = self.create_timer(duration, self.forward_timer_callback)
            self.move_forward()

    def forward_timer_callback(self):
        if self.forward_timer < 1.5:
            self.forward_timer.cancel()
            self.forward_timer = None
        else:    
            self.spin_right()

    def destroy_node(self):
        self.stop_motion()
        super().destroy_node()

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
