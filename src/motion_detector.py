#!/usr/bin/env python
import os
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

path = os.path.dirname(__file__)
cred = credentials.Certificate(path+'/serviceAccount.json')
firebase_admin.initialize_app(cred)
db = firestore.client()

class Motion_detector:
    def __init__(self, table):
        # self.pub = rospy.Publisher("table/info_"+table['tablename'], TableStatus, queue_size=2)
        self.table = table
        self.path = 'Tables/'+self.table['tablename']
        self.update_info_of_table = db.document(self.path)
        self.count_false = 0
        self.count_true = 0
    def detect(self, image):
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
            if cv2.contourArea(c) < 500:
                # self.count_false = self.count_false+1
                # if self.count_false > 5:
                #     tmp = {'ins': False}
                #     self.update_info_of_table.update(tmp)
                #     self.count_false = 0
                remaining_number = self.table['maxnum']
                tmp = {'ins': False, 'remaining_number': remaining_number}
                self.update_info_of_table.update(tmp)
                continue
            tmp = {'ins': True}
            self.update_info_of_table.update(tmp)
            (x, y, w, h) = cv2.boundingRect(c)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # cv2.drawContours(image, cnts, -1, (0, 255, 255), 2)
        cv2.accumulateWeighted(blur, self.avg_float, 0.01)
        self.avg = cv2.convertScaleAbs(self.avg_float)
        return image

class Motion:
    def __init__(self, table):
        self.table = table
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("camera/visible/image_"+self.table['tablename'], Image, queue_size=2)  
        rospy.Subscriber("usb_cam/image_raw", Image, self.imageCallback)
        self.motion_detector = Motion_detector(self.table)
    def imageCallback(self, image):
        if self.motion_detector:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            # result_img = self.motion_detector.detect(cv_image[info_of_table[self.table]["row0"]:info_of_table[self.table]["row1"], info_of_table[self.table]["col0"]:info_of_table[self.table]["col1"]])
            result_img = self.motion_detector.detect(cv_image[self.table['col0']:self.table['col1'], self.table['row0']:self.table['row1']])
            image = self.bridge.cv2_to_imgmsg(result_img, "bgr8")
        self.pub.publish(image)

def shutdown():
    info_of_tables = db.collection('Tables')
    info_of_tables = info_of_tables.get()
    for table in info_of_tables:
        table = table.to_dict()
        path = 'Tables/'+table['tablename']
        recovery_remaining_number = table['maxnum']
        db.document(path).update({'ins': False, 'preorder': False, 'remaining_number': recovery_remaining_number})
  
if __name__ == '__main__':
    rospy.init_node("MotionDetection")
    info_of_tables = db.collection('Tables')
    info_of_tables = info_of_tables.get()
    for table in info_of_tables:
        table = table.to_dict()
        Motion(table)
    rospy.spin()
    rospy.on_shutdown(shutdown)