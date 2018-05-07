#!/usr/bin/env python
import os
import json
import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int8
from motion_detection.msg import TableStatus
from motion_detection.msg import Tables
from motion_detection.msg import FreeList

status_of_table = {}
subs = []
x = []
def callback(*kwargs):
    pub = rospy.Publisher('table/info', Tables, queue_size=2)
    freelist = Tables()
    for tablestatus in kwargs:
        if (tablestatus.ins == False) and (tablestatus.preorder == False):
            freelist.tables.append(tablestatus)
    pub.publish(freelist)

def main():
    path = os.path.dirname(__file__)
    with open(path+"/info_table.json", "r") as reader:
        status_of_table = json.loads(reader.read())
    rospy.init_node("Collect_Table_Status_node")
    list_of_table = status_of_table.keys()
    for table in list_of_table:
        subs.append(message_filters.Subscriber("/table/info_"+table, TableStatus))
    # ts = message_filters.TimeSynchronizer(subs, 3)
    ts = message_filters.ApproximateTimeSynchronizer(subs, 10, 0.1)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    main()