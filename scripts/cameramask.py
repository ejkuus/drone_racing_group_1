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
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Updated to the new policy
            durability=DurabilityPolicy.VOLATILE,  # Updated to the new policy
            history=HistoryPolicy.KEEP_LAST,  # Updated to the new policy
            depth=10  # Keep last 10 messages
        )

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)  
        self.parameters = aruco.DetectorParameters_create()

        self.subscription = self.create_subscription(
		Image,'/image_raw', self.camera_callback, 10
#            Image,'/drone1/image_raw', self.camera_callback, qos_profile
#             Image,'img_rviz', self.camera_callback, qos_profile
        )

        self.publisher = self.create_publisher(Point, '/goal_position', 10)
        self.get_logger().info("GreenShapeDetector node started and listening to /image_raw")
        self.no_detection_counter = 0
        self.stop_detected = False 

        # landing publisher
        self.land_client = self.create_client(TelloAction, '/drone1/tello_action')

        while not self.land_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /drone1/tello_action service...')

    def camera_callback(self, data):
        self.get_logger().info("Testing callback")
        
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
        largest_center = None
        detection_made = False
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 500:  
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    largest_center = (cx, cy)

                    
                    point_msg = Point()
                    point_msg.x = float(cx)
                    point_msg.y = float(cy)
                    point_msg.z = 0.0
                    self.publisher.publish(point_msg)
                    self.get_logger().info(f"Published green shape center: ({cx}, {cy})")
                    detection_made = True
                    
                    cv2.circle(cv_image, (cx, cy), 8, (0, 0, 255), -1)
                    cv2.putText(cv_image, f"({cx},{cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

		
        else:
            largest_center = None


        # ArUco marker detection
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

        if len(corners) > 0:
            for i, corner in enumerate(corners):
        
                c = corner[0]
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))

            
                point_msg = Point()
                point_msg.x = float(cx)
                point_msg.y = float(cy)
                point_msg.z = 0.0
                self.publisher.publish(point_msg)
                self.get_logger().info(f"Published ArUco marker center: ({cx}, {cy})")

            
                cv2.circle(cv_image, (cx, cy), 8, (0, 0, 255), -1)
                cv2.putText(cv_image, f"Marker ({cx},{cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


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


        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        red_mask1 = cv2.inRange(cv_image, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(cv_image, lower_red2, upper_red2)
        red_mask = red_mask1 | red_mask2

        # red contours
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if red_contours and not self.stop_detected:
            largest_red = max(red_contours, key=cv2.contourArea)
            area_red = cv2.contourArea(largest_red)

            if area_red > 300:  # red area treshhold
                self.stop_detected = True  # Mark that we detected the stop

                
                # Prepare the service request
                land_req = TelloAction.Request()
                land_req.cmd = 'land'

                # Call the service
                future = self.land_client.call_async(land_req)
                self.get_logger().info("RED STOP SIGN detected! Sending LAND command via service call!")

                return  # Exit the callback immediately 
            
            cv2.imshow("Original", cv_image)
            #cv2.imshow("Green Only", green_only)
            cv2.waitKey(1)
            # Show both original and masked image
            cv2.imshow("Original", cv_image)
            #cv2.imshow("Green Only", green_only)
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

