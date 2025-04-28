#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
import cv2.aruco as aruco
from tello_msgs.srv import TelloAction


class LineObjective(Node):

    def __init__(self):
        super().__init__('line_objective_node')
        self.bridge_object = CvBridge()
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)  
        self.parameters = aruco.DetectorParameters_create()

        self.subscription = self.create_subscription(
            Image, '/image_raw', self.camera_callback, 10
        )

        self.publisher = self.create_publisher(Point, '/goal_position', 10)
        self.get_logger().info("GreenShapeDetector node started and listening to /image_raw")
        self.no_detection_counter = 0
        self.stop_detected = False

        self.land_client = self.create_client(TelloAction, '/tello_action')

        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /tello_action service...')

    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Green mask range
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        green_only = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)

        # Find green contours
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detection_made = False

        if contours:
            total_cx = 0
            total_cy = 0
            valid_contours = 0

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        total_cx += cx
                        total_cy += cy
                        valid_contours += 1

                        cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)

            if valid_contours > 0:
                avg_cx = total_cx // valid_contours
                avg_cy = total_cy // valid_contours

                # Publish the averaged center
                point_msg = Point()
                point_msg.x = float(avg_cx)
                point_msg.y = float(avg_cy)
                point_msg.z = 0.0
                self.publisher.publish(point_msg)
                self.get_logger().info(f"Published average green center: ({avg_cx}, {avg_cy})")
                detection_made = True

                cv2.circle(cv_image, (avg_cx, avg_cy), 8, (255, 0, 0), -1)
                cv2.putText(cv_image, f"AVG ({avg_cx},{avg_cy})", (avg_cx + 10, avg_cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        else:
            detection_made = False

        # ArUco marker detection
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

        if len(corners) > 0:
            total_cx, total_cy = 0, 0
            detection_count = 0
            for i, corner in enumerate(corners):
                c = corner[0]
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))
                total_cx += cx
                total_cy += cy
                detection_count += 1

                cv2.circle(cv_image, (cx, cy), 8, (0, 0, 255), -1)
                cv2.putText(cv_image, f"Marker ({cx},{cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if detection_count == 3:
                    center_x = total_cx // 3
                    center_y = total_cy // 3

                    point_msg = Point()
                    point_msg.x = float(center_x)
                    point_msg.y = float(center_y)
                    point_msg.z = 0.0
                    self.publisher.publish(point_msg)
                    self.get_logger().info(f"Published ArUco marker center: ({center_x}, {center_y})")

                    cv2.circle(cv_image, (center_x, center_y), 8, (0, 0, 255), -1)
                    cv2.putText(cv_image, f"Marker ({center_x},{center_y})", (center_x + 10, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if not detection_made:
            self.no_detection_counter += 1
            if self.no_detection_counter >= 15:
                point_msg = Point()
                point_msg.x = -1.0
                point_msg.y = -1.0
                point_msg.z = 0.0
                self.publisher.publish(point_msg)
                self.get_logger().info("No green shape or fiducial markers detected for 5 frames. Published: (-1, -1)")
        else:
            self.no_detection_counter = 0

        # Red stop sign detection
        lower_red1 = np.array([0,120,70])
        upper_red1 = np.array([10,255,255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = red_mask1 | red_mask2

        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if red_contours and not self.stop_detected:
            largest_red = max(red_contours, key=cv2.contourArea)
            area_red = cv2.contourArea(largest_red)

            if area_red > 800:
                self.stop_detected = True

                land_req = TelloAction.Request()
                land_req.cmd = 'land'

                # Call service
                # future = self.land_client.call_async(land_req)
                # self.get_logger().info("RED STOP SIGN detected! Sending LAND command via service call!")

                return  # Exit callback immediately after stop detected

        # Visualization
        cv2.imshow("Original", cv_image)
        cv2.imshow("Red Mask", red_mask)
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
