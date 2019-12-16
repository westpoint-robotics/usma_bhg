#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# os for changing directories for saving images, data, CSVs
import os
# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
from datetime import datetime


# Instantiate CvBridge
bridge = CvBridge()
dataDirectory = "/home/user1/Data/"; 
# Camera directories, csv file, and bag file all go here
now = datetime.now() # current date and time
missionName = now.strftime("%Y%m%d_%H%M%S_%f")[:-3] 
missionDirectory = dataDirectory + missionName
flirDirectory = missionDirectory + "/FLIR sn"
gobiDirectory = missionDirectory + "/Gobi sn"
csvFilename  = missionDirectory + "/" + missionName + ".csv"
is_recording = False

def directory_setup():
    if not os.path.exists(missionDirectory):
        os.makedirs(missionDirectory)
    if not os.path.exists(flirDirectory):
        os.makedirs(flirDirectory)
    if not os.path.exists(gobiDirectory):
        os.makedirs(gobiDirectory)    
   

def record_callback(msg):
    global is_recording
    is_recording = msg.data

def image_callback(msg):    
    if (is_recording):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except CvBridgeError, e:
            print(e)
        else:
            now = datetime.now() # current date and time
            date_time = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
            flirFilename = flirDirectory + "_" + date_time + ".ppm"
            rospy.loginfo("Saving image as: " + flirFilename)      
            gobiFilename = gobiDirectory + "_" + date_time + ".png"  
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite(flirFilename, cv2_img)
            cv2.imshow("FLIR", cv2_img)
            cv2.waitKey(3)
            # Update CSV file with the names of the images recorded at this date and time
            with open(csvFilename, 'a+') as csvFile:
                csvFileWriter = csv.writer(csvFile)
                csvFileWriter.writerow([date_time, flirFilename, gobiFilename])

def main():
    directory_setup()
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/camera_array/cam0/image_raw"
    record_topic = "record"
    # Set up your subscriber and define its callback
    rospy.Subscriber(record_topic, Bool, record_callback)
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
    
    
'''
rostopic pub /record std_msgs/Bool True
'''    
