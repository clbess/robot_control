#!/usr/bin/env python

import math
import sys
import rospy
import cv2
import time
import numpy as np

from geometry_msgs.msg      import Twist
from control_msgs.msg       import *
from std_msgs.msg           import *
from sensor_msgs.msg        import JointState
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError

def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

class Blob:
    def __init__(self, mode_settings, precision, x_tar, y_tar, w_tar, h_tar, hsv_min, hsv_max):
        self.bridge = CvBridge()
        self.joint_goal = Float64()
        self.joint_goal.data = 0.0
        self.image_sub = rospy.Subscriber("/raspicam_node/image", Image, self.callback)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        #Publishers to move each joints
        self.joint1_pub = rospy.Publisher('/joint_1/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/joint_2/command', Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher('/joint_3/command', Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher('/joint_4/command', Float64, queue_size=10)
        self.joint5_pub = rospy.Publisher('/joint_5/command', Float64, queue_size=10)
        self.revolute_joint_pub = rospy.Publisher('/gripper_revolute_joint/command', Float64, queue_size=10)
        self.prismatic_joint_pub = rospy.Publisher('/gripper_prismatic_joint/command', Float64, queue_size=10)

        self.x_tar = x_tar
        self.y_tar = y_tar
        self.w_tar = w_tar
        self.h_tar = h_tar
        self.hsv_min = hsv_min
        self.hsv_max = hsv_max
        self.mode_settings = mode_settings
        self.precision = precision

        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.found = 0

        #To set Hue, Saturation and Value values
        if (self.mode_settings == 1):
            cv2.namedWindow("Trackbars")
            cv2.resizeWindow("Tracbars", 640, 240)
            cv2.createTrackbar("H min", "Trackbars", self.hsv_min[0], 179, self.empty)
            cv2.createTrackbar("S min", "Trackbars", self.hsv_min[1], 255, self.empty)
            cv2.createTrackbar("V min", "Trackbars", self.hsv_min[2], 255, self.empty)
            cv2.createTrackbar("H max", "Trackbars", self.hsv_max[0], 179, self.empty)
            cv2.createTrackbar("S max", "Trackbars", self.hsv_max[1], 255, self.empty)
            cv2.createTrackbar("V max", "Trackbars", self.hsv_max[2], 255, self.empty)

    def __del__(self):
        print("I'm being automatically destroyed. Goodbye!")

    def empty(self, a):
        pass

    def callback(self, data):
        try:
            #Get Camera Image
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            img_Copy = img.copy()

            if (self.mode_settings == 1):
                h_min = cv2.getTrackbarPos("H min", "Trackbars")
                s_min = cv2.getTrackbarPos("S min", "Trackbars")
                v_min = cv2.getTrackbarPos("V min", "Trackbars")
                h_max = cv2.getTrackbarPos("H max", "Trackbars")
                s_max = cv2.getTrackbarPos("S max", "Trackbars")
                v_max = cv2.getTrackbarPos("V max", "Trackbars")
                self.hsv_min = (h_min, s_min, v_min)
                self.hsv_max = (h_max, s_max, v_max)

            #Find Oject
            img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            img_Mask = cv2.inRange(img_HSV, self.hsv_min, self.hsv_max)
            img_Result = cv2.bitwise_and(img, img, mask=img_Mask)

            #Find Object Edges
            img_Gray = cv2.cvtColor(img_Result, cv2.COLOR_BGR2GRAY)
            img_Blur = cv2.GaussianBlur(img_Gray, (7, 7), 1)
            img_Edges = cv2.Canny(img_Blur, 50, 50)
            img_Contour = self.GetContours(img_Edges, img_Copy)

            #Control Robot
            self.ControlRobot()

            #Display Images
            if(self.mode_settings == 0):
                imgStack = stackImages(3, ([img_Contour]))
            else:
                imgStack = stackImages(1.5, ([img, img_Contour], [img_Result, img_Edges]))
            cv2.imshow("Detection", imgStack)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def GetContours(self, img, img_Contour):
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:
                cv2.drawContours(img_Contour, cnt, -1, (0, 0, 255), 3)
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                self.x, self.y, self.w, self.h = cv2.boundingRect(approx)
                if (self.mode_settings == 2 or self.mode_settings == 3):
                    print(self.x, self.y, self.w, self.h)
                img_Rect = cv2.rectangle(img_Contour, (self.x, self.y), (self.x + self.w, self.y + self.h), (0, 255, 0), 2)
                if ((self.x >= self.x_tar - self.precision) and (self.x <= self.x_tar + self.precision) and (self.w >= self.w_tar - 4 * self.precision) and (self.w <= self.w_tar + 4 * self.precision) and (self.h >= self.h_tar - self.precision) and (self.h <= self.h_tar + self.precision) and ((self.mode_settings == 0) or (self.mode_settings == 3))):
                    if (self.found == 0):
                        print("The object can be gripped")
                        self.linear_vel = 0
                        self.angular_vel = 0
                        self.found = 1
                        time.sleep(1)

                        self.joint_goal.data = 0.0
                        self.joint1_pub.publish(self.joint_goal)
                        self.joint5_pub.publish(self.joint_goal)
                        self.revolute_joint_pub.publish(self.joint_goal)
                        self.joint_goal.data = 0.7
                        self.joint2_pub.publish(self.joint_goal)
                        self.joint_goal.data = -0.5
                        self.joint3_pub.publish(self.joint_goal)
                        self.joint_goal.data = 0.9
                        self.joint4_pub.publish(self.joint_goal)
                        time.sleep(0.5)

                        self.joint_goal.data = 0.8
                        self.joint2_pub.publish(self.joint_goal)
                        time.sleep(0.5)
                        self.joint_goal.data = 0.9
                        self.joint2_pub.publish(self.joint_goal)
                        time.sleep(0.5)
                        self.joint_goal.data = 1.0
                        self.joint4_pub.publish(self.joint_goal)
                        time.sleep(0.5)
                        self.joint_goal.data = 0.95
                        self.joint2_pub.publish(self.joint_goal)
                        time.sleep(0.5)

                        self.joint_goal.data = 0.0
                        self.prismatic_joint_pub.publish(self.joint_goal)
                        time.sleep(2.5)
                        self.joint_goal.data = 0.7
                        self.joint2_pub.publish(self.joint_goal)
                        self.joint_goal.data = 1.0
                        self.joint4_pub.publish(self.joint_goal)
                        print("The object has been gripped")
                        rospy.signal_shutdown("STOP")
        return img_Contour

    def ControlRobot(self):
        if(self.found == 1 or self.mode_settings == 1 or self.mode_settings == 2):
            self.linear_vel = 0
            self.angular_vel = 0
        elif(self.x == 0 and self.y == 0 and self.w == 0 and self.h == 0):
            self.linear_vel = 0.1
        else:
            if(self.x < self.x_tar - 8 * self.precision or self.x > self.x_tar + 8 * self.precision):
                self.angular_vel = math.copysign(0.13, self.x_tar - self.x)
            elif(self.x < self.x_tar - 5 * self.precision or self.x > self.x_tar + 5 * self.precision):
                self.angular_vel = math.copysign(0.08, self.x_tar - self.x)
            elif(self.x < self.x_tar - self.precision or self.x > self.x_tar + self.precision):
                self.angular_vel = math.copysign(0.05, self.x_tar - self.x)
            else:
                self.angular_vel = 0.0

            if(self.h < self.h_tar - 8 * self.precision or self.h > self.h_tar + 8 * self.precision):
                self.linear_vel = math.copysign(0.06, self.h_tar - self.h)
            elif(self.h < self.h_tar - 5 * self.precision or self.h > self.h_tar + 5 * self.precision):
                self.linear_vel = math.copysign(0.03, self.h_tar - self.h)
            elif(self.h < self.h_tar - self.precision or self.h > self.h_tar + self.precision):
                self.linear_vel = math.copysign(0.01, self.h_tar - self.h)
            else:
                self.linear_vel = 0.0

        twist = Twist()
        twist.linear.x = self.linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = self.angular_vel
        self.cmd_pub.publish(twist)
        if (self.mode_settings == 3):
            print(str(self.angular_vel) + "  " + str(self.linear_vel))

def main(args):
    rospy.init_node('detection', anonymous=True)

    #Chose mode : 0 -> Normal mode / 1 -> Set new color / 2 -> Set new object / 3 -> Debug
    mode_settings = 0

    precision = 1

    x_target = 151
    y_target = 107
    w_target = 36
    h_target = 114

    hsv_min = (121, 116, 67)
    hsv_max = (179, 255, 160)

    blob = Blob(mode_settings, precision, x_target, y_target, w_target, h_target, hsv_min, hsv_max)

    print("Node launched")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
