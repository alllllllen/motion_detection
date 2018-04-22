#!/usr/bin/env python
import os
import json
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String ,Bool

status_of_table = {

}

class SubscriberTableStatus:
    def __init__(self, table):
        self.table = table
        rospy.Subscriber("table/info_"+table, Bool, self.statusCallback)
    def statusCallback(self, boolen):
        if boolen :
            status_of_table[self.table]["in"] = True
        else:
            status_of_table[self.table]["in"] = False

class PublisherTableStatus:
    def __init__(self):
        self.pub = rospy.Publisher("table/freetable", Bool, queue_size=2)
        


def main():
    path = os.path.dirname(__file__)
    with open(path+"/info_table.json", "r") as reader:
    # with open("/home/allen/dashgo_ws/src/motion_detection/src/info_table.json", "r") as reader:
        status_of_table = json.loads(reader.read())
    rospy.init_node("collect_table_status_node")    
    list_of_table = status_of_table.keys()
    for i in range(len(status_of_table)):
        SubscriberTableStatus(list_of_table[i])

    rospy.spin()
    # print(status_of_table)

if __name__ == '__main__':
    main()