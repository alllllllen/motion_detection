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
import threading

status_of_table = {}

class Collect_Table_Status:
    def __init__(self, list_of_table, initial_state_of_table):
        self.subs = []
        self.freelist = Tables()
        self.lock = threading.Lock()
        self.pub = rospy.Publisher('table/info', Tables, queue_size=2)
        self.pub.publish(initial_state_of_table)
        for table in list_of_table:
            x = message_filters.Subscriber("/table/info_"+table, TableStatus)
            x.registerCallback(self.tablecallback)
            self.subs.append(x)
        
    def tablecallback(self, tablestatus):
        self.lock.acquire()
        if (tablestatus.ins == False) and (tablestatus.preorder == False) and (self.freelist.tables.count(tablestatus) == 0 ):
            self.freelist.tables.append(tablestatus)
        else:
            try:
                tmp = tablestatus
                tmp.ins = False
                tmp.preorder = False
                self.freelist.tables.remove(tmp)
            except:
                pass
        self.pub.publish(self.freelist)
        self.lock.release()

def main():
    path = os.path.dirname(__file__)
    with open(path+"/info_table.json", "r") as reader:
        status_of_table = json.loads(reader.read())
    rospy.init_node("Collect_Table_Status_node")
    list_of_table = status_of_table.keys()
    initial_state_of_table = Tables()
    for table in list_of_table:
        tableStatus = TableStatus()
        tableStatus.tablename = table
        tableStatus.id = status_of_table[table]["id"]
        tableStatus.row0 = status_of_table[table]["row0"]
        tableStatus.row1 = status_of_table[table]["row1"]
        tableStatus.col0 = status_of_table[table]["col0"]
        tableStatus.col1 = status_of_table[table]["col1"]
        tableStatus.maxnum = status_of_table[table]["maxnum"]
        tableStatus.preorder = status_of_table[table]["preorder"]
        tableStatus.ins = status_of_table[table]["ins"]
        tableStatus.distance = status_of_table[table]["distance"]
        tableStatus.priority = status_of_table[table]["priority"]
        initial_state_of_table.tables.append(tableStatus)
    Collect_Table_Status(list_of_table, initial_state_of_table)
    rospy.spin()

if __name__ == '__main__':
    main()