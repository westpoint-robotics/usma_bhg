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
from mavros_msgs.msg import Altitude
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float64


# Install PySpin for Python 2.7
import PySpin
import sys


# Instantiate CvBridge
bridge = CvBridge()
dataDirectory = "/home/user1/Data/" # default value changes on subscribing
# Camera directories, csv file, and bag file all go here
#now = datetime.now() # current date and time
flirSN = "FLIR18284612"
flirDirectory = ""
csvFilename = ""
datetimeData = ""
is_recording = False
rel_alt = Altitude()
gps_fix = NavSatFix()
imu_mag = MagneticField()
imu_data = Imu()
vel_gps = TwistStamped()
temp_imu = Temperature()
imageCount = 0


def make_header():
    header = "filename,rostime,rel_alt.monotonic,rel_alt.amsl,rel_alt.local,rel_alt.relative,"
    header += "gps_fix.status.status,gps_fix.status.service,gps_fix.latitude,gps_fix.longitude,gps_fix.altitude,"
    header += "imu_data.magnetic_field.x,imu_data.magnetic_field.y,imu_data.magnetic_field.z,"
    header += "imu_mag.orientation.x,imu_mag.orientation.y,imu_mag.orientation.z,imu_mag.orientation.w, imu_mag.angular_velocity.x,imu_mag.angular_velocity.y,imu_mag.angular_velocity.z,"
    header += "imu_mag.linear_acceleration:.x,imu_mag.linear_acceleration:.y,imu_mag.linear_acceleration:.z,"
    header += "vel_gps.twist.linear.x,vel_gps.twist.linear.y,vel_gps.twist.linear.z,"
    header += "vel_gps.twist.angular.x,vel_gps.twist.angular.y,vel_gps.twist.angular.z,"
    header += "temp_imu.temperature"
    return header

def make_logentry():
    global datetimeData
    alt_str = str(rel_alt.monotonic) + "," + str(rel_alt.amsl) + "," + str(rel_alt.local) + "," + str(rel_alt.relative) 
    gps_str = str(gps_fix.status.status) + "," + str(gps_fix.status.service) + "," + str(gps_fix.latitude) + "," + str(gps_fix.longitude) + "," + str(gps_fix.altitude) 
    mag_str = str(imu_mag.magnetic_field.x) + "," + str(imu_mag.magnetic_field.y) + "," + str(imu_mag.magnetic_field.z)
    imu_str = str(imu_data.orientation.x) + "," + str(imu_data.orientation.y) + "," + str(imu_data.orientation.z) + "," + str(imu_data.orientation.w) + ","
    imu_str += str(imu_data.angular_velocity.x) + "," + str(imu_data.angular_velocity.y) + "," + str(imu_data.angular_velocity.z) + ","
    imu_str += str(imu_data.linear_acceleration.x) + "," + str(imu_data.linear_acceleration.y) + "," + str(imu_data.linear_acceleration.z)    
    vel_str = str(vel_gps.twist.linear.x) + "," + str(vel_gps.twist.linear.y) + "," + str(vel_gps.twist.linear.z) + ","
    vel_str += str(vel_gps.twist.angular.x) + "," + str(vel_gps.twist.angular.y) + "," + str(vel_gps.twist.angular.z)  
    temp_str = str(temp_imu.temperature)    
    output = str(datetimeData + "," + alt_str + "," + gps_str + "," + mag_str + "," + imu_str + "," + vel_str + "," + temp_str)
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
    
def imu_cb(msg):
    global imu_data
    imu_data = msg
    
def vel_cb(msg):
    global vel_gps
    vel_gps = msg
    
def temp_cb(msg):
    global temp_imu
    temp_imu = msg  

def directory_callback(msg):
    global flirDirectory
    global csvFilename
    missionDirectory = msg.data
    missionName = missionDirectory.split("/")[4]     
    flirDirectory    = missionDirectory + flirSN
    csvFilename      = missionDirectory + missionName + "_flir.csv"
    
    if(not (os.path.isdir(flirDirectory))):
        os.mkdir(flirDirectory)
        rospy.loginfo("***** FLIR *****: Directory Created: " + flirDirectory)

    if(not (os.path.exists(csvFilename))):
        rospy.loginfo("***** FLIR *****: CSV File Created: " + csvFilename)
        with open(csvFilename, 'a+') as csvFile:     
            csvFile.write(make_header() + "\n")
          
def record_callback(msg):
    global is_recording
    is_recording = msg.data

def image_callback(msg):  
    global imageCount
    global datetimeData
    if (is_recording and os.path.exists(csvFilename) ):
        #Before taking a picture, grab timestamp to record to filename, and CSV
        tNow = rospy.get_time() # current date and time
        #rospy.loginfo("tNow = %.3f\n", tNow)
        datetimeData = datetime.datetime.fromtimestamp(tNow).strftime('%Y%m%d_%H%M%S_%f')
              
        #Grab the image and convert it to OpenCV format        
        try:

            # Enable STROBE
            system = PySpin.System.GetInstance()
            cam_list = system.GetCameras()
            cam_strobe = cam_list[0]            
            enable_strobe_bfs(cam_strobe)

            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        flirFilename = flirDirectory + "/" + flirSN + "_" + datetimeData[:-3] + ".ppm" #date_time + ".ppm"            
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite(flirFilename, cv2_img)
        imageCount = imageCount + 1

        # Disable STROBE
        disable_strobe_bfs(cam_strobe)

        if (imageCount % 10 == 0):
            rospy.loginfo("***** FLIR *****: Image Count: %d\n" %(imageCount) )
        # Update CSV file with the names of the images recorded at this date and time
        with open(csvFilename, 'a+') as csvFile:
            output_str = flirFilename + "," + make_logentry()
            csvFile.write(output_str + "\n")

# Strobe
'''
Measure the voltage across Line2 (as this line is used for strobe)
and see whether it spikes periodically.
Plugging Line2(pin3) and ground(pin6) into a voltage meter or oscilloscope.
'''

def enable_strobe_bfs(cam):
    """
    Enables stobe for the given bfs `cam` by turning
    on 3.3V Enable for Line2 and Line1 output on ExposureActive
    *Maybe need the "nodemap"
    # Initialize camera
    cam.Init()
    # Retrieve GenICam nodemap
    nodemap = cam.GetNodeMap()
    """
    print('+++++ STROBE: Enable +++++')

    # Initialize camera
    cam.Init()
    # Retrieve GenICam nodemap
    nodemap = cam.GetNodeMap()

    # Turn on 3.3V Enable for Line 2
    cam.LineSelector.SetValue(PySpin.LineSelector_Line2)
    cam.V3_3Enable.SetValue(True)
    # Turn on strobe for Line 1
    cam.LineSelector.SetValue(PySpin.LineSelector_Line1)
    cam.LineSource.SetValue(PySpin.LineSource_ExposureActive)

def disable_strobe_bfs(cam):
    """
    Enables stobe for the given bfs `cam` by turning
    off 3.3V Enable for Line2 and Line1 output on ExposureActive
    *Maybe need the "nodemap"
    # Initialize camera
    cam.Init()
    # Retrieve GenICam nodemap
    nodemap = cam.GetNodeMap()
    """
    print('----- STROBE: Disable -----')

    # Initialize camera
    cam.Init()
    # Retrieve GenICam nodemap
    nodemap = cam.GetNodeMap()

    # Turn on 3.3V disable for Line 2
    cam.LineSelector.SetValue(PySpin.LineSelector_Line2)
    cam.V3_3Enable.SetValue(False)
    # Turn on strobe for Line 1
    cam.LineSelector.SetValue(PySpin.LineSelector_Line1)
    cam.LineSource.SetValue(PySpin.LineSource_ExposureActive)


def main():
    
    rospy.init_node('image_listener')
    
    # Define your image topic
    #image_topic = "/camera_array/cam0/image_raw"
    image_topic = "/camera/image_raw"
       
    #timestamp_topic = 'timestamp'
    rospy.Subscriber('/directory', String, directory_callback)
    rospy.Subscriber("/record", Bool, record_callback)
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber("/mavros/altitude", Altitude, alt_cb)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, gps_cb)
    rospy.Subscriber("/mavros/imu/mag", MagneticField, mag_cb)
    rospy.Subscriber("/mavros/imu/data", Imu, imu_cb)
    rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, vel_cb)
    rospy.Subscriber("/mavros/imu/temperature_imu", Temperature, temp_cb)

    # Spin until ctrl + c
    #r = rospy.Rate(20) # 5hz
    #while not rospy.is_shutdown():
    #    r.sleep()    
    rospy.sleep(3)
    rospy.spin()


    return result

if __name__ == '__main__':
    main()
    
    
'''
rostopic pub /record std_msgs/Bool True
'''    
