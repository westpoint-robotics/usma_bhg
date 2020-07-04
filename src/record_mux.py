#! /usr/bin/python

'''
This program does;
1. Triggers recording based on a channel from the flight cotnroller.
2. Triggers recording based on flight modes. It automaticlly starts recording
when the system is in auto mode and automatically stops when it leaves auto mode.
'''

import os
import rospy
from mavros_msgs.msg import RCIn
from std_msgs.msg import Bool
from mavros_msgs.msg import State

class RecordMux:
    def __init__(self):
        self.rec_pub = rospy.Publisher("/record", Bool, queue_size=10)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rcin_cb)
        rospy.Subscriber("/mavros/state", State, self.mav_mode_cb)
        self.crnt_switch_val = 0
        self.last_switch_val = 0
        self.last_mode=""
        self.crnt_mode=""
        
    # Listen for Remote Controller switch to flip and update the state variable
    def rcin_cb(self, msg):
        try:
            self.crnt_switch_val = int(msg.channels[8])
        except:
            #rospy.loginfo("***** RECORD MUX MAV MSG not a list of 8 or more *****")
            pass
            
    def mav_mode_cb(self, msg):
        #rospy.loginfo("Mavros mode is: %s" % msg.mode)
        if self.last_mode != "AUTO" and msg.mode == "AUTO":
            self.rec_pub.publish(True)
        elif self.last_mode == "AUTO" and msg.mode != "AUTO":
            self.rec_pub.publish(False)
        self.last_mode = msg.mode

if __name__ == '__main__':
    rospy.init_node('record_mux')
    rm = RecordMux()

    r = rospy.Rate(10) 
    while not rospy.is_shutdown():
        if rm.crnt_switch_val != rm.last_switch_val:
            rospy.loginfo(rm.crnt_switch_val)
            rm.last_switch_val = rm.crnt_switch_val
            if rm.crnt_switch_val > 1800:
                rm.rec_pub.publish(False)
            elif rm.crnt_switch_val < 1800:
                rm.rec_pub.publish(True)

        r.sleep()

