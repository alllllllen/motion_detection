#!/usr/bin/env python
import os
import json
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

info_of_table = {
}

class Motion_detector:
    def __init__(self, table):
        self.pub = rospy.Publisher("table/info_"+table, String, queue_size=2)
    def detect(self,image):
        blur = cv2.blur(image, (4, 4))
        try:
            diff = cv2.absdiff(self.avg, blur)
        except:
            self.avg = cv2.blur(image, (4, 4))
            self.avg_float = np.float32(self.avg)
            diff = cv2.absdiff(self.avg, blur)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        _ret, thresh = cv2.threshold(gray, 25, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        _cntImg, cnts, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
    def __init__(self, table):
        self.table = table
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("camera/visible/image_"+self.table, Image, queue_size=2)  
        rospy.Subscriber("usb_cam/image_raw", Image, self.imageCallback)
        self.motion_detector = Motion_detector(self.table)
        # rospy.spin()
    def imageCallback(self, image):
        if self.motion_detector:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            result_img = self.motion_detector.detect(cv_image[info_of_table[self.table]["row0"]:info_of_table[self.table]["row1"], info_of_table[self.table]["col0"]:info_of_table[self.table]["col1"]])
            image = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
        self.pub.publish(image)
if __name__ == '__main__':
    path = os.path.dirname(__file__)
    with open(path+"/info_table.json", "r") as reader:
        info_of_table = json.loads(reader.read())
    rospy.init_node("motion_detector_node")
    list_of_table = info_of_table.keys()
    for i in range(len(list_of_table)):
        Motion(list_of_table[i])
    rospy.spin()