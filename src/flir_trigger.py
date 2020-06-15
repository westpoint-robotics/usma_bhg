#! /usr/bin/python
# -*- coding: utf-8 -*-
#  Copyright © 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.

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
import time
import datetime
# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
# ROS Image message -> OpenCV2 image converter
#from cv_bridge import CvBridge, CvBridgeError
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

# Parameters
dataDirectory = "/home/user1/Data/" # default value changes on subscribing
# Camera directories, csv file, and bag file all go here
#now = datetime.now() # current date and time
flirSN = "FLIR18284612"
flirDirectory = ""
csvFilename = ""
datetimeData = ""
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


class TriggerType:
    SOFTWARE = 1
    HARDWARE = 2

CHOSEN_TRIGGER = TriggerType.HARDWARE

def configure_trigger(cam):
    """
    This function configures the camera to use a trigger. First, trigger mode is
    ensured to be off in order to select the trigger source. Trigger mode is
    then enabled, which has the camera capture only a single image upon the
    execution of the chosen trigger.
    """

    if CHOSEN_TRIGGER == TriggerType.SOFTWARE:
        print("SOFTWARE trigger chosen...")
    elif CHOSEN_TRIGGER == TriggerType.HARDWARE:
        print("HARDWARE trigger chosen...")

    try:
        result = True

        # Ensure trigger mode off
        # The trigger must be disabled in order to configure whether the source
        # is software or hardware.
        if cam.TriggerMode.GetAccessMode() != PySpin.RW:
            print("Unable to disable trigger mode (node retrieval). Aborting...")
            return False

        cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)

        print("Trigger mode disabled...")

        # Select trigger source
        # The trigger source must be set to hardware or software while trigger
		# mode is off.
        if cam.TriggerSource.GetAccessMode() != PySpin.RW:
            print("Unable to get trigger source (node retrieval). Aborting...")
            return False

        if CHOSEN_TRIGGER == TriggerType.SOFTWARE:
            cam.TriggerSource.SetValue(PySpin.TriggerSource_Software)
        elif CHOSEN_TRIGGER == TriggerType.HARDWARE:
            cam.TriggerSource.SetValue(PySpin.TriggerSource_Line3)

        # Turn trigger mode on
        # Once the appropriate trigger source has been set, turn trigger mode
        # on in order to retrieve images using the trigger.
        cam.TriggerMode.SetValue(PySpin.TriggerMode_On)
        print("Trigger mode turned back on...")

    except PySpin.SpinnakerException as ex:
        print("Error: %s" % ex)
        return False

    return result


def grab_next_image_by_trigger(cam):
    """
    This function acquires an image by executing the trigger node.
    """
    try:
        result = True
        # Use trigger to capture image
        # The software trigger only feigns being executed by the Enter key;
        # what might not be immediately apparent is that there is not a
        # continuous stream of images being captured; in other examples that
        # acquire images, the camera captures a continuous stream of images.
        # When an image is retrieved, it is plucked from the stream.

        if CHOSEN_TRIGGER == TriggerType.SOFTWARE:
            # Get user input
            raw_input("Press the Enter key to initiate software trigger.")

            # Execute software trigger
            if cam.TriggerSoftware.GetAccessMode() != PySpin.WO:
                print("Unable to execute trigger. Aborting...")
                return False
            cam.TriggerSoftware.Execute()
            # TODO: Blackfly and Flea3 GEV cameras need 2 second delay after software trigger

        elif CHOSEN_TRIGGER == TriggerType.HARDWARE:
            print("Use the hardware to trigger image acquisition.")

    except PySpin.SpinnakerException as ex:
        print("Error: %s" % ex)
        return False

    return result


def acquire_images(cam):
#  This function acquires and saves images from a device.

    global imageCount
    global datetimeData

    try:
        result = True
        # Set acquisition mode to continuous
        if cam.AcquisitionMode.GetAccessMode() != PySpin.RW:
            print("Unable to set acquisition mode to continuous. Aborting...")
            return False

        cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
        print("Acquisition mode set to continuous...")

        #  Begin acquiring images
        cam.BeginAcquisition()

        # Retrieve, convert, and save images
        r = rospy.Rate(20) # Hz
        while not rospy.is_shutdown():
            try:
                #  Retrieve the next image from the trigger
                result &= grab_next_image_by_trigger(cam)
                #  Retrieve next received image
                image_result = cam.GetNextImage()

                #  Before taking a picture, grab timestamp to record to filename, and CSV
                tNow = rospy.get_time() # current date and time
                #rospy.loginfo("tNow = %.3f\n", tNow)
                datetimeData = datetime.datetime.fromtimestamp(tNow).strftime('%Y%m%d_%H%M%S_%f')

                #  Ensure image completion
                if image_result.IsIncomplete():
                    print("Image incomplete with image status %d ..." % image_result.GetImageStatus())

                else:
                    if os.path.exists(csvFilename):
                        #Before taking a picture, grab timestamp to record to filename, and CSV
                        tNow = rospy.get_time() # current date and time
                        #rospy.loginfo("tNow = %.3f\n", tNow)
                        datetimeData = datetime.datetime.fromtimestamp(tNow).strftime('%Y%m%d_%H%M%S_%f')

                    #  Convert image to bgr8/mono8
                    image_converted = image_result.Convert(PySpin.PixelFormat_BGR8, PySpin.HQ_LINEAR)
                    #image_converted = image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)

                    image_data = image_converted.GetNDArray()
                    cv2.waitKey(1)

                    flirFilename = flirDirectory + "/" + flirSN + "_" + datetimeData[:-3] + ".ppm" #date_time + ".ppm"

# Saving time takes long.
                    #print(time.time())
                    # time.time(): 0.05
                    #  Save your OpenCV2 image as a jpeg
                    image_converted.Save(flirFilename)
                    # time.time(): 0.51

                    imageCount = imageCount + 1
                    if (imageCount % 10 == 0):
                        rospy.loginfo("***** FLIR *****: Image Count: %d\n" %(imageCount) )
                        #  Update CSV file with the names of the images recorded at this date and time
                        with open(csvFilename, 'a+') as csvFile:
                            output_str = flirFilename + "," + make_logentry()
                            csvFile.write(output_str + "\n")

                    #  Release image
                    image_result.Release()

            except PySpin.SpinnakerException as ex:
                print("Error: %s" % ex)

                return False
            r.sleep()

        # End acquisition
        cam.EndAcquisition()

    except PySpin.SpinnakerException as ex:
        print("Error: %s" % ex)
        return False

    return result


def reset_trigger(cam):
    """
    This function returns the camera to a normal state by turning off trigger mode.
    """
    try:
        result = True
        # Ensure trigger mode off
        # The trigger must be disabled in order to configure whether the source
        # is software or hardware.
        if cam.TriggerMode.GetAccessMode() != PySpin.RW:
            print("Unable to disable trigger mode (node retrieval). Aborting...")
            return False

        cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)

        print("Trigger mode disabled...")

    except PySpin.SpinnakerException as ex:
        print("Error: %s" % ex)
        result = False

    return result


def print_device_info(nodemap):
#  This function prints the device information of the camera from the transport layer.

    print("*** DEVICE INFORMATION ***\n")

    try:
        result = True
        node_device_information = PySpin.CCategoryPtr(nodemap.GetNode("DeviceInformation"))

        if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
            features = node_device_information.GetFeatures()
            for feature in features:
                node_feature = PySpin.CValuePtr(feature)
                print("%s: %s" % (node_feature.GetName(),
                                  node_feature.ToString() if PySpin.IsReadable(node_feature) else "Node not readable"))

        else:
            print("Device control information not available.")

    except PySpin.SpinnakerException as ex:
        print("Error: %s" % ex)
        return False

    return result


def run_single_camera(cam):
#  The body of the example.

    try:
        result = True
        err = False

        #  Retrieve TL device nodemap and print device information
        nodemap_tldevice = cam.GetTLDeviceNodeMap()
        #result &= print_device_info(nodemap_tldevice)

        #  Initialize camera
        cam.Init()
        #  Retrieve GenICam nodemap
        nodemap = cam.GetNodeMap()

        #  Configure trigger
        if configure_trigger(cam) is False:
            return False

        #  Acquire images
        result &= acquire_images(cam)

        #  Reset trigger
        result &= reset_trigger(cam)
        #  Deinitialize camera
        cam.DeInit()

    except PySpin.SpinnakerException as ex:
        print("Error: %s" % ex)
        result = False

    return result


def main():
    rospy.init_node('FLIR_trigger')

    # Define your image topic
    image_topic = "/camera/image_raw"
    rospy.Subscriber('/directory', String, directory_callback)

    rospy.Subscriber("/record", Bool, record_callback)

    rospy.Subscriber("/mavros/altitude", Altitude, alt_cb)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, gps_cb)
    rospy.Subscriber("/mavros/imu/mag", MagneticField, mag_cb)
    rospy.Subscriber("/mavros/imu/data", Imu, imu_cb)
    rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, vel_cb)
    rospy.Subscriber("/mavros/imu/temperature_imu", Temperature, temp_cb)

    #  Retrieve singleton reference to system object
    system = PySpin.System.GetInstance()

    #  Retrieve list of cameras from the system
    cam_list = system.GetCameras()
    num_cameras = cam_list.GetSize()

    #  Finish if there are no cameras
    if num_cameras == 0:
        #  Clear camera list before releasing system
        cam_list.Clear()

        #  Release system
        system.ReleaseInstance()

        print("FLIR Camera is not detected! Reconnect the USB cable")
        return False

    #  Run example on each camera
    cam = cam_list[0]
    result = run_single_camera(cam)

    # Release reference to camera
    # NOTE: Unlike the C++ examples, we cannot rely on pointer objects being automatically
    # cleaned up when going out of scope.
    # The usage of del is preferred to assigning the variable to None.
    del cam

    #  Clear camera list before releasing system
    cam_list.Clear()

    #  Release instance
    system.ReleaseInstance()


    return result


if __name__ == "__main__":
    main()



