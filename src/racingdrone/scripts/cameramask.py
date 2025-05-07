#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
# from move_robot import MoveKobuki


class LineObjective(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        
    def get_hsv_ranges(self):
        """Define HSV ranges for colors."""
        return {
            "yellow": (np.array([20, 100, 100]), np.array([30, 255, 255])),
            "red": (np.array([0, 120, 70]), np.array([10, 255, 255])),
            "green": (np.array([40, 50, 50]), np.array([80, 255, 255])),
            "blue": (np.array([100, 150, 0]), np.array([140, 255, 255]))
        } 
               
    def camera_callback(self, data):
        """Handle camera feed and path following."""
        if self.reached_target:
            rospy.loginfo(f"Reached target color: {self.target_color}. Stopping.")
            self.movekobuki_object.stop_robot()
            return

        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Create masks for the target and yellow colors
        lower_green , upper_green = self.hsv_ranges["green"]
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        if self.target_color in self.hsv_ranges:
            lower_target, upper_target = self.hsv_ranges[self.target_color]
            target_mask = cv2.inRange(hsv, lower_target, upper_target)

            # Check if target color is detected
            if cv2.countNonZero(target_mask) > 0:
                rospy.loginfo(f"Detected target color: {self.target_color}.")
                self.reached_target = True
                cv2.imshow("Target Color", cv2.bitwise_and(cv_image, cv_image, mask=target_mask))
                cv2.imshow("Original", cv_image)
                cv2.waitKey(1)
                # self.movekobuki_object.stop_robot()
                return

        # # Follow yellow line if target not yet reached
        # contours, _ = cv2.findContours(green_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        # rospy.loginfo(f"Number of centroids: {len(contours)}")

        # centres = []
        # for contour in contours:
        #     moments = cv2.moments(contour)
        #     try:
        #         centres.append((int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00'])))
        #     except ZeroDivisionError:
        #         pass

        # # Select the rightmost centroid
        # if centres:
        #     cx, cy = max(centres, key=lambda c: c[0])
        # else:
        #     cx, cy = width / 2, height / 2

        # Visualize the results
        res = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)
        cv2.circle(res, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("Res", res)
        cv2.waitKey(1)

        # # Control logic for the robot
        # error_x = cx - width / 2
        # twist_object = Twist()
        # twist_object.linear.x = 0.2
        # twist_object.angular.z = -error_x / 100
        # rospy.loginfo(f"ANGULAR VALUE SENT: {twist_object.angular.z}")
        # self.movekobuki_object.move_robot(twist_object)
