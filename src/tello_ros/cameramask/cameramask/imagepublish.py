#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point

class imagePublisher(Node):
    def __init__(self):
        super().__init__('imagePublisher_node')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.img_path = "/home/elias/drone_racing_ros2_ws/cornerGreenCircle_0.jpg"
        self.pub = self.create_publisher(Image, 'img_rviz', qos_profile)
        if os.path.isfile(self.img_path):
            img = cv2.imread(self.img_path)
            bridge = CvBridge()
            img_msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
            self.pub.publish(img_msg)
            self.get_logger().info("Publishing gate image.")
        #{}


def main(args=None):
    rclpy.init(args=args)
    node = imagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()