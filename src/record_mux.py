#! /usr/bin/python

import os
import rospy
from mavros_msgs.msg import RCIn
from std_msgs.msg import Bool

class RecordMux:
    def __init__(self):
        self.rec_pub = rospy.Publisher("/record", Bool, queue_size=10)
        self.diagnostic = rospy.Subscriber("/mavros/rc/in", RCIn, self.rcin_cb)
        self.crnt_switch_val = 0
        self.last_switch_val = 0
        

    def rcin_cb(self, msg):
        self.crnt_switch_val = int(msg.channels[8])

if __name__ == '__main__':
    rospy.init_node('record_mux')
    rm = RecordMux()

    r = rospy.Rate(10) # 5hz
    while not rospy.is_shutdown():
        if rm.crnt_switch_val != rm.last_switch_val:
            rospy.loginfo(rm.crnt_switch_val)
            rm.last_switch_val = rm.crnt_switch_val
            if rm.crnt_switch_val > 1800:
                rm.rec_pub.publish(False)
            elif rm.crnt_switch_val < 1800:
                rm.rec_pub.publish(True)

        r.sleep()

