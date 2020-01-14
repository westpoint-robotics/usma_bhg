#!/usr/bin/env python
# license removed for brevity
import rospy
import keyboard
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('record', Bool, queue_size=10)
    rospy.init_node('pub_record', anonymous=True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if keyboard.is_pressed(' '):
            is_recording = True
            rospy.loginfo("Starting the recording")
        elif keyboard.is_pressed('q'):
            is_recording = False
            rospy.loginfo("Stopping the recording")
        pub.publish(is_recording)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
