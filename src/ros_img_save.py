#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html
import message_filters
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
missionName = ""#now.strftime("%Y%m%d_%H%M%S_%f")[:-3] 
missionDirectory = ""
flirSN = "FLIR_18284612"
flirDirectory = ""
csvFilename = ""
#gobiSN = "XEN_000088"
timestamp_data = ""
is_recording = False
directorySet = False


def directory_setup(directory):
    if not os.path.exists(flirDirectory):
        print('Create flirDirectory: {}'.format(directory))
        os.makedirs(directory)  

def directory_callback(msg):#directory, timestamp):
    global directorySet
    global flirDirectory
    global csvFilename
    global missionName
    global timestamp_data
    
    missionName = msg.data#directory_topic#.data
    #timestamp_data = timestamp#.data

    print('receiving, missionName: {}'.format(missionName))
    #print('timestamp_callback: {}'.format(timestamp_data))
    
    if(missionName <> "" and directorySet == False):
        directorySet     = True
        #print('directorySet (should be True) = {}'.format(directorySet))
        missionDirectory = dataDirectory + missionName 
        flirDirectory    = missionDirectory + "/" + flirSN
        csvFilename      = missionDirectory + "/" + missionName + ".csv"
        #directory_setup(csvFilename)
        #directory_setup(flirDirectory)
        
    else:
        print('directorySet (should be False) = {}'.format(directorySet))
'''
def timestamp_callback(msg):
    global timestamp_data
    timestamp_data = msg.data
    print('timestamp_callback: {}'.format(timestamp_data))
    print('sleep(3)\nspin()')
'''    
def record_callback(msg):
    global is_recording
    is_recording = msg.data

def image_callback(msg):  
    global flirDirectory
    global csvFileName

    if (is_recording):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except CvBridgeError, e:
            print(e)
        else:
            now = datetime.now() # current date and time
            timestamp_data = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
            flirFilename = flirDirectory + "/" + flirSN + "_" + timestamp_data + ".ppm" #date_time + ".ppm"
            rospy.loginfo("Mission directory: " + missionDirectory)
            rospy.loginfo("Saving image as: " + flirFilename)      
            #gobiFilename = gobiDirectory + "_" + timestamp_data + ".png"  
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite(flirFilename, cv2_img)
            #cv2.imshow("FLIR", cv2_img)
            #cv2.waitKey(3)
            
            # Update CSV file with the names of the images recorded at this date and time
            with open(csvFilename, 'a+') as csvFile:
                csvFileWriter = csv.writer(csvFile)
                #csvFileWriter.writerow([timestamp_data, flirFilename, gobiFilename])
                csvFileWriter.writerow([timestamp_data, flirFilename])
             

def main():
    
    rospy.init_node('image_listener')
    
    # Define your image topic
    image_topic = "/camera_array/cam0/image_raw"
    record_topic = "record"
    directory_topic = 'directory'
    #timestamp_topic = 'timestamp'
    rospy.Subscriber(record_topic, Bool, record_callback)
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber(directory_topic, String, directory_callback)
    '''
    #Getting errors: Examine this (http://wiki.ros.org/message_filters#Example_.28Python.29-1)
    directory_sub = message_filters.Subscriber(directory_topic, String)#, directory_callback)
    timestamp_sub = message_filters.Subscriber(timestamp_topic, String)#, timestamp_callback)
    
    ts = message_filters.TimeSynchronizer([directory_sub, timestamp_sub], 10)
    ts.registerCallback(directory_callback)
    '''
    # Spin until ctrl + c
    rospy.sleep(3)
    rospy.spin()

if __name__ == '__main__':
    main()
    
    
'''
rostopic pub /record std_msgs/Bool True
'''    
