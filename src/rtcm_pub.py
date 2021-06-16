#! /usr/bin/python
"""
This module manages the directories for BHG. It does the following:
    - Creates the data directory and mission directory for saving the images
    - Publishes the mission directory for the cameras to use for saving images
    - Publishes the number of files in the subdirectories
    - Cleans up (deletes) unused directories
    
It also does 

"""
import os
import rospy
from mavros_msgs.msg import RTCM


rtcm_pub = rospy.Publisher('directory', RTCM, queue_size=10)

rospy.init_node('rtcm_pub')  
r = rospy.Rate(5) # 5hz
msg=RTCM()
while not rospy.is_shutdown():
    rtcm_pub.pub_mission_dir()
    ds.count_files()
    r.sleep()
    
    
print("===== All done exiting cleanly")
   
    
'''    
rosmsg info mavros_msgs/RTCM 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint8[] data
'''

