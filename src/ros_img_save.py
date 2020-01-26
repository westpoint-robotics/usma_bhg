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
from mavros_msgs.msg import Altitude
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
missionName = ""#now.strftime("%Y%m%d_%H%M%S_%f")[:-3] 
missionDirectory = ""
flirSN = "FLIR_18284612"
flirDirectory = ""
csvFilename = ""
#gobiSN = "XEN_000088"
timestamp_data = ""
is_recording = False
directorySet = False
rel_alt = Altitude()
gps_fix = NavSatFix()
imu_mag = MagneticField()
imu_fcu = Imu()
vel_fcu = TwistStamped()
temp_fcu = Temperature()


def make_header():
    header = "rostime,rel_alt.monotonic,rel_alt.amsl,rel_alt.local,rel_alt.relative,"
    header += "gps_fix.status.status,gps_fix.status.service,gps_fix.latitude,gps_fix.longitude,gps_fix.altitude,"
    header += "imu_fcu.magnetic_field.x,imu_fcu.magnetic_field.y,imu_fcu.magnetic_field.z,"
    header += "imu_mag.orientation.x,imu_mag.orientation.y,imu_mag.orientation.z,imu_mag.orientation.w, imu_mag.angular_velocity.x,imu_mag.angular_velocity.y,imu_mag.angular_velocity.z,"
    header += "imu_mag.linear_acceleration:.x,imu_mag.linear_acceleration:.y,imu_mag.linear_acceleration:.z,"
    header += "vel_fcu.twist.linear.x,vel_fcu.twist.linear.y,vel_fcu.twist.linear.z,"
    header += "vel_fcu.twist.angular.x,vel_fcu.twist.angular.y,vel_fcu.twist.angular.z,"
    header += "temp_fcu.temperature"
    return header

def make_logentry():
    alt_str = str(rel_alt.monotonic) + "," + str(rel_alt.amsl) + "," + str(rel_alt.local) + "," + str(rel_alt.relative) 
    gps_str = str(gps_fix.status.status) + "," + str(gps_fix.status.service) + "," + str(gps_fix.latitude) + "," + str(gps_fix.longitude) + "," + str(gps_fix.altitude) 
    mag_str = str(imu_mag.magnetic_field.x) + "," + str(imu_mag.magnetic_field.y) + "," + str(imu_mag.magnetic_field.z)
    imu_str = str(imu_fcu.orientation.x) + "," + str(imu_fcu.orientation.y) + "," + str(imu_fcu.orientation.z) + "," + str(imu_fcu.orientation.w) + ","
    imu_str += str(imu_fcu.angular_velocity.x) + "," + str(imu_fcu.angular_velocity.y) + "," + str(imu_fcu.angular_velocity.z) + ","
    imu_str += str(imu_fcu.linear_acceleration.x) + "," + str(imu_fcu.linear_acceleration.y) + "," + str(imu_fcu.linear_acceleration.z)    
    vel_str = str(vel_fcu.twist.linear.x) + "," + str(vel_fcu.twist.linear.y) + "," + str(vel_fcu.twist.linear.z) + ","
    vel_str += str(vel_fcu.twist.angular.x) + "," + str(vel_fcu.twist.angular.y) + "," + str(vel_fcu.twist.angular.z)  
    temp_str = str(temp_fcu.temperature)    
    output = str(rospy.Time.now()) + "," + alt_str + "," + gps_str + "," + mag_str + "," + imu_str + "," + vel_str + "," + temp_str
    return output

def alt_cb(msg):
    global rel_alt
    rel_alt = msg
        
def gps_cb(msg):
    global gps_fix
    gps_fix = msg
    
def mag_cb(msg):
    global imu_mag
    imu_mag = msg
    
def imu_fcu(msg):
    global imu_fcu
    imu_fcu = msg
    
def vel_cb(msg):
    global vel_fcu
    vel_fcu = msg
    
def temp_cb(msg):
    global temp_fcu
    temp_fcu = msg

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

    #print('receiving, missionName: {}'.format(missionName))
    #print('timestamp_callback: {}'.format(timestamp_data))
    
    if(missionName <> "" and directorySet == False):
        directorySet     = True
        #print('directorySet (should be True) = {}'.format(directorySet))
        missionDirectory = dataDirectory + missionName 
        flirDirectory    = missionDirectory + "/" + flirSN
        csvFilename      = missionDirectory + "/" + missionName + ".csv"
        #directory_setup(csvFilename)
        #directory_setup(flirDirectory)
        print("====== Filename for csv is: ", csvFilename )        
        with open(csvFilename, 'a+') as csvFile:
            #csvFileWriter = csv.writer(csvFile)
            #csvFileWriter.writerow([timestamp_data, flirFilename, gobiFilename])
            #csvFileWriter.writerow(make_header() + "\n")      
            csvFile.write(make_header() + "\n")
    else:
        pass
        #print('directorySet (should be False) = {}'.format(directorySet))     
          
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
                #csvFileWriter = csv.writer(csvFile)
                #csvFileWriter.writerow([timestamp_data, flirFilename, gobiFilename])
                output_str = flirFilename + "," + make_logentry()
                #csvFileWriter.writerow(output_str + "\n")
                csvFile.write(output_str + "\n")
                        

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
