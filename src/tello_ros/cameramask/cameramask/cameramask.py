#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point


class LineObjective(Node):

    def __init__(self):
        super().__init__('line_objective_node')
        self.bridge_object = CvBridge()
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            Image,'/drone1/image_raw', self.camera_callback, qos_profile
#             Image,'img_rviz', self.camera_callback, qos_profile
        )

        self.publisher = self.create_publisher(Point, '/goal_position', 10)
        self.get_logger().info("GreenShapeDetector node started and listening to /drone1/image_raw")


    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data)#, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Green mask range
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # Apply mask to original image
        green_only = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)


        # Find contours
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_center = None

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 500:  # Ignore small noise
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    largest_center = (cx, cy)

                    # Publish position
                    point_msg = Point()
                    point_msg.x = float(cx)
                    point_msg.y = float(cy)
                    point_msg.z = 0.0
                    self.publisher.publish(point_msg)
                    self.get_logger().info(f"Published green shape center: ({cx}, {cy})")

                    # Visual debug
                    cv2.circle(cv_image, (cx, cy), 8, (0, 0, 255), -1)
                    cv2.putText(cv_image, f"({cx},{cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show both original and masked image
        cv2.imshow("Original", cv_image)
        cv2.imshow("Green Only", green_only)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = LineObjective()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down LineObjective node.")
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
