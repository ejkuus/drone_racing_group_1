#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy

"""
Topics To Write on:
type: std_msgs/Float64
/mira/pitch_joint_position_controller/command
/mira/roll_joint_position_controller/command
/mira/yaw_joint_position_controller/command
"""

class TelloGateFollower(Node):

    def __init__(self, is_2D = True):
        super().__init__('gateFollower_node')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.get_logger().info("Mira Initialising Blob Follower...")

        self.move_rate = self.create_rate(10)

        self._is_2D = is_2D
        self.acceptable_error = 5
        self.image_width = 960
        self.image_height = 720
        self.current_yaw = 0.0
        self.twist_obj = Twist()
        self.pub_mira_move = self.create_publisher(Twist,'/drone1/cmd_vel', 1)
       
        self.point_blob_topic = '/goal_position'
#        self._check_cv_image_ready()
        self.create_subscription(Point, self.point_blob_topic, self.point_blob_callback, qos_profile)

        self.get_logger().info("Mira Initialising Goal Follower...")
    

#    def _check_cv_image_ready(self):
#        self.point_blob = None
#        while self.point_blob is None: #and not rclpy.shutdown():
#            try:
#                self.point_blob = rclpy.wait_for_message(Point, self.point_blob_topic,  timeout=1.0)
#                self.get_logger().info("Current "+self.point_blob_topic+" READY=>")
#
#            except:
#                self.get_logger().info("Current "+self.point_blob_topic+" not ready yet, retrying for getting "+self.point_blob_topic+"")
#        return self.point_blob
    
    def point_blob_callback(self, msg):
        # Proportional control factor
        k = 0.5
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0

        norm_x = (msg.x - cx) / cx      # left: -1, right: +1
        norm_y = (cy - msg.y) / cy      # down: -1, up: +1


        # Centering along X axis (left-right in image): rotate (yaw)
        if norm_x < self.acceptable_error:
            self.twist_obj.linear.y = -k * norm_x * 0.1

        elif norm_x > self.acceptable_error:
            self.twist_obj.angular.y = k * norm_x * 0.1
#        else:
#            self.twist_obj.angular.z = 0.0

        # Centering along Y axis (up-down in image): vertical movement
        if norm_y < self.acceptable_error:
            self.twist_obj.linear.z = k * norm_y *0.1
        elif norm_y > self.acceptable_error:
            self.twist_obj.linear.z = -k * norm_y *0.1
#        else:
#            self.twist_obj.linear.z = 0.0


        # Publish and log
        self.pub_mira_move.publish(self.twist_obj)
        self.get_logger().info("Published twist values...")
    





def main(args=None):
    #rclpy.init('mira_follow_blob_node', anonymous=True, log_level=rclpy.DEBUG)
    rclpy.init(args = args)
    node = TelloGateFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down LineObjective node.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()