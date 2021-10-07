#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import numpy as np
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def nothing(x):
    pass


# 设置滑动条组件
cv2.namedWindow('HSV_Window')
cv2.createTrackbar('H_L', 'HSV_Window', 0, 255, nothing)
cv2.createTrackbar('H_H', 'HSV_Window', 0, 255, nothing)
cv2.createTrackbar('S_L', 'HSV_Window', 0, 255, nothing)
cv2.createTrackbar('S_H', 'HSV_Window', 0, 255, nothing)
cv2.createTrackbar('V_L', 'HSV_Window', 0, 255, nothing)
cv2.createTrackbar('V_H', 'HSV_Window', 0, 255, nothing)


class cvBridgeDemo():
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)  # 当ros关闭时对opencv进行清理
        self.bridge = CvBridge()
        # 订阅usb_cam发布的图像topic, 当得到数据类型为Image的图像时调用对于回调函数image_callback
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        # 每30ms执行一次show_img_cb，用于刷新显示窗口中的图像
        rospy.Timer(rospy.Duration(0.03), self.show_img_cb)

    def show_img_cb(self, evnet):
        try:
            cv2.namedWindow("HSV_Window", cv2.WINDOW_NORMAL);
            cv2.imshow("HSV_Window", self.frame)
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)  # 将读取的BGR转换为HSV

            H_L = cv2.getTrackbarPos('H_L', 'HSV_Window')
            H_H = cv2.getTrackbarPos('H_H', 'HSV_Window')
            S_L = cv2.getTrackbarPos('S_L', 'HSV_Window')
            S_H = cv2.getTrackbarPos('S_H', 'HSV_Window')
            V_L = cv2.getTrackbarPos('V_L', 'HSV_Window')
            V_H = cv2.getTrackbarPos('V_H', 'HSV_Window')

            lower = np.array([H_L, S_L, V_L])  # 所要检测的像素范围
            upper = np.array([H_H, S_H, V_H])  # 此处检测绿色区域

            mask = cv2.inRange(hsv, lowerb=lower, upperb=upper)
            cv2.imshow("mask", mask)

            cv2.waitKey(3)
        except:
            pass

    def image_callback(self, ros_image):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print
            e

    def cleanup(self):
        cv2.destroyAllWindows()


def main(args):
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

