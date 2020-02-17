#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

import csv
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
from std_msgs.msg import String



# Instantiate CvBridge
bridge = CvBridge()
dataDirectory = "/home/user1/Data/"; 
# Camera directories, csv file, and bag file all go here
#now = datetime.now() # current date and time
#missionName = now.strftime("%Y%m%d_%H%M%S_%f")[:-3] 
#missionDirectory = dataDirectory + missionName
flirSN = "FLIR_18284612"
flirDirectory = ""
csvFilename = ""
#gobiSN = "XEN_000088"
is_recording = False
directorySet = False


def directory_setup():
    if not os.path.exists(flirDirectory):
        print('Create flirDirectory: %s', flirDirectory)
        os.makedirs(flirDirectory)  

def directory_callback(msg):
    global missionDirectory
    missionDirectory = msg.data
    print('directory_callback, missionDirectory: %s', missionDirectory)
    if(missionDirectory.data != ""):
        directorySet = True
        flirDirectory = missionDirectory + "/" + flirSN
        csvFilename  = missionDirectory + "/" + missionName + ".csv"
    

def record_callback(msg):
    global is_recording
    is_recording = msg.data

def image_callback(msg):  
    #pub.publish(missionName)  
    if (is_recording):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except CvBridgeError, e:
            print(e)
        else:
            now = datetime.now() # current date and time
            date_time = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
            flirFilename = flirDirectory + "/" + flirSN + "_" + date_time + ".ppm"
            rospy.loginfo("Saving image as: " + flirFilename)      
            #gobiFilename = gobiDirectory + "_" + date_time + ".png"  
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite(flirFilename, cv2_img)
            #cv2.imshow("FLIR", cv2_img)
            #cv2.waitKey(3)
            # Update CSV file with the names of the images recorded at this date and time
            with open(csvFilename, 'a+') as csvFile:
                csvFileWriter = csv.writer(csvFile)
                #csvFileWriter.writerow([date_time, flirFilename, gobiFilename])
                csvFileWriter.writerow([date_time, flirFilename])

def main():
    
    rospy.init_node('image_listener')
    
    # Define your image topic
    image_topic = "/camera_array/cam0/image_raw"
    record_topic = "record"
    directory_topic = "directory"
    
    print('directorySet: {}'.format(directorySet))
    rospy.Subscriber(directory_topic, String, directory_callback)
    if(directorySet == False):
       print('directorySet == False')
        # Set up your subscriber and define its callback
        rospy.Subscriber(directory_topic, String, directory_callback)
    else:
        print('directory_setup()')
        directory_setup()
        rospy.Subscriber(record_topic, Bool, record_callback)
        rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.sleep(3)
    rospy.spin()

if __name__ == '__main__':
    main()
    
    
'''
rostopic pub /record std_msgs/Bool True
'''    
