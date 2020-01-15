#! /usr/bin/python

import csv
# os for changing directories for saving images, data, CSVs
import os
import subprocess
# rospy for the subscriber
import rospy
from datetime import datetime
from std_msgs.msg import String
directory_topic = "directory"
#timestamp_topic = "timestamp"
dataDirectory = "/home/user1/Data/"
# Camera directories, csv file, and bag file all go here
now = datetime.now() # current date and time
missionName = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
#timestamp   = missionName 
missionDirectory = dataDirectory + missionName + "/"
flirSN = "FLIR_18284612"
flirDirectory = missionDirectory + "/" + flirSN


def directory_setup():
    if not os.path.exists(missionDirectory):
        os.makedirs(missionDirectory)
    if not os.path.exists(flirDirectory):
        os.makedirs(flirDirectory)


def main():
    global missionName
    #global timestamp
    pub = rospy.Publisher(directory_topic, String, queue_size=10)
    #pub = rospy.Publisher(timestamp_topic, String, queue_size=10)
    rospy.init_node('directory_setup')
    directory_setup()
    r = rospy.Rate(3) # 3hz
    while not rospy.is_shutdown():
        now = datetime.now() # current date and time
        #timestamp = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        #print('publishing, missionName: {}'.format(missionName))
        #print('publishing, timestamp: {}'.format(timestamp))
        pub.publish(missionName)
        #pub.publish(timestamp)
         
        r.sleep()

if __name__ == '__main__':
    main()


