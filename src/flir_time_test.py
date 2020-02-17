#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html
import message_filters
import csv
# os for changing directories for savidatetimeng images, data, CSVs
import os
# rospy for the subscriber
import rospy
# time for date and time
#import time
import datetime
# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
#from datetime import datetime
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float64


# Instantiate CvBridge
bridge = CvBridge()
dataDirectory = "/home/user1/Data/"; 
# Camera directories, csv file, and bag file all go here
#now = datetime.now() # current date and time
flirSN = "FLIR18284612"
flirDirectory = ""
csvFilename = ""
timestamp_data = ""
is_recording = False
imageCount = 0

def directory_callback(msg):
    global flirDirectory
    global csvFilename
    missionName = msg.data
    missionDirectory = dataDirectory + missionName 
    flirDirectory    = missionDirectory + "/" + flirSN
    csvFilename      = missionDirectory + "/" + missionName + "_flir.csv"
    
    if(not (os.path.isdir(flirDirectory))):
        os.mkdir(flirDirectory)
        rospy.loginfo("FLIR Directory Created: " + flirDirectory)

    if(not (os.path.exists(csvFilename))):
        rospy.loginfo("FLIR CSV File Created: " + csvFilename)
          
def record_callback(msg):
    global is_recording
    is_recording = msg.data

def main():
    
    rospy.init_node('image_listener')
       
    #timestamp_topic = 'timestamp'
    rospy.Subscriber('/directory', String, directory_callback)
    rospy.Subscriber("/record", Bool, record_callback)

    pub = rospy.Publisher('flir_timer', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
    
        if (is_recording and os.path.exists(csvFilename) ):
            #Before taking a picture, grab timestamp to record to filename, and CSV
            tNow = rospy.get_time() # current date and time
            #rospy.loginfo("tNow = %.3f\n", tNow)
            datetimeData = datetime.datetime.fromtimestamp(tNow).strftime('%Y%m%d_%H%M%S_%f')      
            flirFilename = flirDirectory + "/" + flirSN + "_" + datetimeData[:-3] + ".ppm" #date_time + ".ppm"    
            rospy.loginfo("%s", flirFilename)    
            pub.publish(flirFilename)
        rate.sleep()    
    
    
    
    
    
    
    

if __name__ == '__main__':
    main()
    
    
'''
rostopic pub /record std_msgs/Bool True
'''    
