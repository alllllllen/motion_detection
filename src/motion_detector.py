#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
class Motion_detector:
    def __init__(self):
        self.pub = rospy.Publisher("table/info", String, queue_size=2)
    def detect(self,image):
        blur = cv2.blur(image, (4, 4))
        try:
            diff = cv2.absdiff(self.avg, blur)
        except:
            self.avg = cv2.blur(image, (4, 4))
            self.avg_float = np.float32(self.avg)
            diff = cv2.absdiff(self.avg, blur)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        cntImg, cnts, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            if cv2.contourArea(c) < 2500:
                continue
            self.pub.publish("someone there")
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.drawContours(image, cnts, -1, (0, 255, 255), 2)
        cv2.accumulateWeighted(blur, self.avg_float, 0.01)
        self.avg = cv2.convertScaleAbs(self.avg_float)
        return image
class Motion:
    def __init__(self):
        rospy.init_node("motion_detector_node")
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("camera/visible/image", Image, queue_size=2)
        rospy.Subscriber("usb_cam/image_raw", Image, self.imageCallback)
        self.motion_detector = Motion_detector()
        rospy.spin()
    def imageCallback(self, image):
        if self.motion_detector:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            result_img = self.motion_detector.detect(cv_image)
            image = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
        self.pub.publish(image)
if __name__ == '__main__':
    detector = Motion()