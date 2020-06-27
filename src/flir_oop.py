#! /usr/bin/python
from __future__ import print_function

# Based heavily on the FLIR Trigger_QuickSpin.py example code

import PySpin
import rospy
import cv2
import os
import datetime

from multiprocessing.pool import ThreadPool
from collections import deque

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image  
from std_msgs.msg import Bool
from std_msgs.msg import String  
from mavros_msgs.msg import Altitude
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float64    
        
class TriggerType:
    SOFTWARE = 1
    HARDWARE = 2
    
class Bhg_flir:

    def __init__(self, _threaded_mode = False):
        self.timestamp = 0
        self.chosen_trigger = TriggerType.HARDWARE
        self.image_pub = rospy.Publisher("image_color",Image, queue_size=10)
        self.cam_system = PySpin.System.GetInstance()
        self.cam_list = self.cam_system.GetCameras()
        self.num_cameras = self.cam_list.GetSize()
        self.ser_num = '0'
        self.data_dir = "/tmp/BHG_DATA"
        self.image_folder = self.data_dir + '/FLIR/'
        self.image_filename = "default_flir_imgname.png"
        self.csv_filename = "default_flir.csv"
        self.datetimeData = ""
        self.is_recording = False
        self.rel_alt = Altitude()
        self.gps_fix = NavSatFix()
        self.imu_mag = MagneticField()
        self.imu_data = Imu()
        self.vel_gps = TwistStamped()
        self.temp_imu = Temperature()
        self.threadn = cv2.getNumberOfCPUs()
        self.pool = ThreadPool(processes = self.threadn)
        self.pending = deque()
        self.threaded_mode = _threaded_mode        
        
        rospy.Subscriber('/directory', String, self.directory_callback)
        rospy.Subscriber("/record", Bool, self.record_callback)
        rospy.Subscriber("/mavros/altitude", Altitude, self.alt_cb)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.gps_cb)
        rospy.Subscriber("/mavros/imu/mag", MagneticField, self.mag_cb)
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, self.vel_cb)
        rospy.Subscriber("/mavros/imu/temperature_imu", Temperature, self.temp_cb)

        # Finish if there are no cameras
        if self.num_cameras == 0:
            # Clear camera list before releasing system
            self.cam_list.Clear()

            # Release system
            self.cam_system.ReleaseInstance()

            rospy.loginfo("***** FLIR:  Not enough cameras!")
            exit()

        self.cam = self.cam_list.GetByIndex(0)    

    # Helper function for multi threaded 
    class DummyTask:
        def __init__(self, data):
            self.data = data
        def ready(self):
            return True
        def get(self):
            return self.data


    def configure_trigger(self):
        """
        This function configures the camera to use a trigger. First, trigger mode is
        ensured to be off in order to select the trigger source. Trigger mode is
        then enabled, which has the camera capture only a single image upon the
        execution of the chosen trigger.

         :param cam: Camera to configure trigger for.
         :type cam: CameraPtr
         :return: True if successful, False otherwise.
         :rtype: bool
        """

        # rospy.loginfo("***** FLIR:  CONFIGURING TRIGGER ***\n")

        if self.chosen_trigger == TriggerType.SOFTWARE:
            rospy.loginfo("***** FLIR:  Software trigger chosen...")
        elif self.chosen_trigger == TriggerType.HARDWARE:
            rospy.loginfo("***** FLIR:  Hardware trigger chosen...")

        try:
            result = True

            # Ensure trigger mode off
            # The trigger must be disabled in order to configure whether the source
            # is software or hardware.
            if self.cam.TriggerMode.GetAccessMode() != PySpin.RW:
                rospy.loginfo("***** FLIR:  Unable to disable trigger mode (node retrieval). Aborting...")
                return False

            self.cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)
            # To add a delay from trigger to capture. Tested and works.
            self.cam.TriggerDelay.SetValue(25.0) # in microseconds must be (24.0 < delay < 65520)

            #rospy.loginfo("***** FLIR:  Trigger mode disabled...")

            # Select trigger source
            # The trigger source must be set to hardware or software while trigger
		    # mode is off.
            if self.cam.TriggerSource.GetAccessMode() != PySpin.RW:
                rospy.loginfo("***** FLIR:  Unable to get trigger source (node retrieval). Aborting...")
                return False

            if self.chosen_trigger == TriggerType.SOFTWARE:
                self.cam.TriggerSource.SetValue(PySpin.TriggerSource_Software)
            elif self.chosen_trigger == TriggerType.HARDWARE:
                self.cam.TriggerSource.SetValue(PySpin.TriggerSource_Line3)

            # Turn trigger mode on
            # Once the appropriate trigger source has been set, turn trigger mode
            # on in order to retrieve images using the trigger.
            self.cam.TriggerMode.SetValue(PySpin.TriggerMode_On)
            #rospy.loginfo("***** FLIR:  Trigger mode turned back on...")

        except PySpin.SpinnakerException as ex:
            rospy.loginfo("***** FLIR:  Error: %s" % ex)
            return False

        return result
        
    def grab_next_image_by_trigger(self):
        """
        This function acquires an image by executing the trigger node.

        :param cam: Camera to acquire images from.
        :type cam: CameraPtr
        :return: True if successful, False otherwise.
        :rtype: bool
        """
        try:
            result = True
            # Use trigger to capture image
            # The software trigger only feigns being executed by the Enter key;
            # what might not be immediately apparent is that there is not a
            # continuous stream of images being captured; in other examples that
            # acquire images, the camera captures a continuous stream of images.
            # When an image is retrieved, it is plucked from the stream.

            if self.chosen_trigger == TriggerType.SOFTWARE:
                # Get user input
                raw_input("Press the Enter key to initiate software trigger.")

                # Execute software trigger
                if self.cam.TriggerSoftware.GetAccessMode() != PySpin.WO:
                    rospy.loginfo("***** FLIR:  Unable to execute trigger. Aborting...")
                    return False

                self.cam.TriggerSoftware.Execute()

                # TODO: Blackfly and Flea3 GEV cameras need 2 second delay after software trigger

            elif self.chosen_trigger == TriggerType.HARDWARE:
                # rospy.loginfo("Use the hardware to trigger image acquisition.")
                pass
                
        except PySpin.SpinnakerException as ex:
            rospy.loginfo("***** FLIR:  Error: %s" % ex)
            return False

        return result     
        
    def acquire_images(self):
        """
        This function acquires and saves 10 images from a device.
        Please see Acquisition example for more in-depth comments on acquiring images.

        :param cam: Camera to acquire images from.
        :type cam: CameraPtr
        :return: True if successful, False otherwise.
        :rtype: bool
        """

        #rospy.loginfo("***** FLIR:  IMAGE ACQUISITION ***\n")
        try:
            result = True

            # Set acquisition mode to continuous
            if self.cam.AcquisitionMode.GetAccessMode() != PySpin.RW:
                rospy.loginfo("***** FLIR:  Unable to set acquisition mode to continuous. Aborting...")
                return False

            self.cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
            #rospy.loginfo("***** FLIR:  Acquisition mode set to continuous...")

            #  Begin acquiring images
            self.cam.BeginAcquisition()

            rospy.loginfo("***** FLIR:  Acquiring images...")

            # Get device serial number for filename
            if self.cam.TLDevice.DeviceSerialNumber.GetAccessMode() == PySpin.RO:
                self.ser_num = self.cam.TLDevice.DeviceSerialNumber.GetValue()

                #rospy.loginfo("***** FLIR:  Device serial number retrieved as %s..." % self.ser_num)

            # Retrieve, convert, and save images
            bridge = CvBridge()
            last_time = rospy.get_time()
            n = 0
            saved_count = 0
            r = rospy.Rate(40) # 40hz
            while not rospy.is_shutdown():        
            
                try:
                    #print("Time now1: %f" % (rospy.get_time() - last_time)) # 0.000086
                    #  Retrieve the next image from the trigger
                    #  Does nothing if Hardware trigger except print line.
                    result &= self.grab_next_image_by_trigger()
                    #print("Time now2: %f" % (rospy.get_time() - last_time)) # 0.000107
                                    
                    #  Retrieve next received image
                    image_result = self.cam.GetNextImage(500) # timeout in milliseconds
                    #print("Time now3: %f" % (rospy.get_time() - last_time)) # 0.029838

                    #  Ensure image completion
                    if image_result.IsIncomplete():
                        rospy.loginfo_throttle(30, "***** FLIR:  Image incomplete with image status %d ..." % image_result.GetImageStatus())

                    else:
                        n += 1
                        
                        #  Convert image to mono 8
                        image_converted = image_result.Convert(PySpin.PixelFormat_BGR8, PySpin.HQ_LINEAR)
                        image_data = image_converted.GetNDArray()
                        #print("Time now4: %f" % (rospy.get_time() - last_time)) # 0.045697
                        
                        #Before taking a picture, grab timestamp to record to filename, and CSV
                        tNow = rospy.get_time() # current date and time
                        self.datetimeData = datetime.datetime.fromtimestamp(tNow).strftime('%Y%m%d_%H%M%S_%f')
                        if (self.is_recording and os.path.exists(self.csv_filename) ):  
                            saved_count += 1                      
                            self.threaded_save_img(image_data, self.datetimeData) 
                            with open(self.csv_filename,"a") as f:
                                f.write(self.make_logentry()+'\n')  
                        #cv2.imshow("frame",image_data)
                        #cv2.waitKey(1)
                        try:
                            cv2.putText(image_data,self.datetimeData[:-3], 
                                (30,1280), 
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                2,
                                (251,251,251),
                                3)
                            
                            self.image_pub.publish(bridge.cv2_to_imgmsg(image_data, "bgr8"))
                        except CvBridgeError as e:
                          rospy.loginfo("***** FLIR: %s" %e)
                        #print("Time now5: %f\n--------------------\n" % (rospy.get_time() - last_time)) # 0.049854

                        #  Release image
                        image_result.Release()
                        rospy.loginfo_throttle(10,"    ***** FLIR:  Grabbed Image %d, and saved %d" % (n, saved_count))

                except PySpin.SpinnakerException as ex:
                    rospy.loginfo_throttle(10, "***** FLIR:  Error: %s" % ex)

                last_time = rospy.get_time()
                r.sleep()
            # End acquisition
            self.cam.EndAcquisition()

        except PySpin.SpinnakerException as ex:
            rospy.loginfo("***** FLIR:  Error: %s" % ex)
            return False

        return result           

    def reset_trigger(self):
        """
        This function returns the camera to a normal state by turning off trigger mode.

        :param cam: Camera to acquire images from.
        :type cam: CameraPtr
        :returns: True if successful, False otherwise.
        :rtype: bool
        """
        try:
            result = True
            # Ensure trigger mode off
            # The trigger must be disabled in order to configure whether the source
            # is software or hardware.
            if self.cam.TriggerMode.GetAccessMode() != PySpin.RW:
                rospy.loginfo("***** FLIR:  Unable to disable trigger mode (node retrieval). Aborting...")
                return False

            self.cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)

            rospy.loginfo("***** FLIR:  Trigger mode disabled...")

        except PySpin.SpinnakerException as ex:
            rospy.loginfo("***** FLIR:  Error: %s" % ex)
            result = False

        return result  

    def print_device_info(self,nodemap):
        """
        This function prints the device information of the camera from the transport
        layer; please see NodeMapInfo example for more in-depth comments on printing
        device information from the nodemap.

        :param nodemap: Transport layer device nodemap.
        :type nodemap: INodeMap
        :returns: True if successful, False otherwise.
        :rtype: bool
        """

        rospy.loginfo("***** FLIR:   DEVICE INFORMATION ***\n")

        try:
            result = True
            node_device_information = PySpin.CCategoryPtr(nodemap.GetNode("DeviceInformation"))

            if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
                features = node_device_information.GetFeatures()
                for feature in features:
                    node_feature = PySpin.CValuePtr(feature)
                    rospy.loginfo("***** FLIR:  %s: %s" % (node_feature.GetName(),
                                      node_feature.ToString() if PySpin.IsReadable(node_feature) else "Node not readable"))
            else:
                rospy.loginfo("***** FLIR:  Device control information not available.")

        except PySpin.SpinnakerException as ex:
            rospy.loginfo("***** FLIR:  Error: %s" % ex)
            return False

        return result        

    def run_camera(self):
        """
        This function acts as the body of the example; please see NodeMapInfo example
        for more in-depth comments on setting up cameras.

        :param cam: Camera to run on.
        :type cam: CameraPtr
        :return: True if successful, False otherwise.
        :rtype: bool
        """
        try:
            result = True
            err = False

            # Retrieve TL device nodemap and print device information
            nodemap_tldevice = self.cam.GetTLDeviceNodeMap()

            #result &= self.print_device_info(nodemap_tldevice)

            # Initialize camera
            self.cam.Init()

            # Retrieve GenICam nodemap
            nodemap = self.cam.GetNodeMap()

            # Configure trigger
            if self.configure_trigger() is False:
                return False

            # Acquire images
            result &= self.acquire_images()

            # Reset trigger
            result &= self.reset_trigger()

            # Deinitialize camera
            self.cam.DeInit()

        except PySpin.SpinnakerException as ex:
            rospy.loginfo_throttle(60, "***** FLIR:  Error: %s" % ex)
            result = False

        return result  

    def close_camera(self):
        # Release reference to camera
        # NOTE: Unlike the C++ examples, we cannot rely on pointer objects being automatically
        # cleaned up when going out of scope.
        # The usage of del is preferred to assigning the variable to None.
        rospy.loginfo("***** FLIR:  Closing camera now.\n")
        del self.cam

        # Clear camera list before releasing system
        self.cam_list.Clear()

        # Release instance
        self.cam_system.ReleaseInstance()

    def make_header(self):
        header = "filename,rostime,rel_alt.monotonic,rel_alt.amsl,rel_alt.local,rel_alt.relative,"
        header += "gps_fix.status.status,gps_fix.status.service,gps_fix.latitude,gps_fix.longitude,gps_fix.altitude,"
        header += "imu_data.magnetic_field.x,imu_data.magnetic_field.y,imu_data.magnetic_field.z,"
        header += "imu_mag.orientation.x,imu_mag.orientation.y,imu_mag.orientation.z,imu_mag.orientation.w,"
        header += "imu_mag.angular_velocity.x,imu_mag.angular_velocity.y,imu_mag.angular_velocity.z,"
        header += "imu_mag.linear_acceleration:.x,imu_mag.linear_acceleration:.y,imu_mag.linear_acceleration:.z,"
        header += "vel_gps.twist.linear.x,vel_gps.twist.linear.y,vel_gps.twist.linear.z,"
        header += "vel_gps.twist.angular.x,vel_gps.twist.angular.y,vel_gps.twist.angular.z,"
        header += "temp_imu.temperature"
        return header

    def make_logentry(self):
        alt_str = str(self.rel_alt.monotonic) + "," + str(self.rel_alt.amsl) + "," + str(self.rel_alt.local) + "," + str(self.rel_alt.relative) 
        gps_str = str(self.gps_fix.status.status) + "," + str(self.gps_fix.status.service) + "," + str(self.gps_fix.latitude) + "," + str(self.gps_fix.longitude) + "," + str(self.gps_fix.altitude) 
        mag_str = str(self.imu_mag.magnetic_field.x) + "," + str(self.imu_mag.magnetic_field.y) + "," + str(self.imu_mag.magnetic_field.z)
        imu_str = str(self.imu_data.orientation.x) + "," + str(self.imu_data.orientation.y) + "," + str(self.imu_data.orientation.z) + "," + str(self.imu_data.orientation.w) + ","
        imu_str += str(self.imu_data.angular_velocity.x) + "," + str(self.imu_data.angular_velocity.y) + "," + str(self.imu_data.angular_velocity.z) + ","
        imu_str += str(self.imu_data.linear_acceleration.x) + "," + str(self.imu_data.linear_acceleration.y) + "," + str(self.imu_data.linear_acceleration.z)    
        vel_str = str(self.vel_gps.twist.linear.x) + "," + str(self.vel_gps.twist.linear.y) + "," + str(self.vel_gps.twist.linear.z) + ","
        vel_str += str(self.vel_gps.twist.angular.x) + "," + str(self.vel_gps.twist.angular.y) + "," + str(self.vel_gps.twist.angular.z)  
        temp_str = str(self.temp_imu.temperature)    
        output = str(self.image_folder + "," + self.datetimeData + "," + alt_str + "," + gps_str + "," + mag_str + "," + imu_str + "," + vel_str + "," + temp_str)
        return output

    def alt_cb(self, msg):
        self.rel_alt = msg

    def gps_cb(self, msg):
        self.gps_fix = msg

    def mag_cb(self, msg):
        self.imu_mag = msg

    def imu_cb(self, msg):
        self.imu_data = msg

    def vel_cb(self, msg):
        self.vel_gps = msg

    def temp_cb(self, msg):
        self.temp_imu = msg  

    def directory_callback(self, msg):
        self.data_dir = msg.data
        self.create_directories();
        
    def create_directories(self):
        # missionDirectory = msg.data   # data: "/home/user1/Data/20200615_145002_422/"
        dir_time = self.data_dir.split("/")[4] #   missionName  "20200615_145002_422"
        self.image_folder = self.data_dir + 'FLIR_SN_' + self.ser_num + '/'
        self.csv_filename = self.data_dir + dir_time + "_flir.csv"

        if(not (os.path.isdir(self.image_folder))):
            os.mkdir(self.image_folder)
            rospy.loginfo("***** FLIR:  Directory Created: " + self.image_folder)

        if(not (os.path.exists(self.csv_filename))):
            rospy.loginfo("***** FLIR:  CSV File Created: " + self.csv_filename)
            with open(self.csv_filename, 'w') as csvFile:     
                csvFile.write(self.make_header() + "\n")

    def record_callback(self, msg):
        self.is_recording = msg.data
        
    def save_img(self, image_data, dtime_data):   
        self.image_filename = self.image_folder + "/FLIR" + self.ser_num + "_" + dtime_data + ".ppm"  
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite(self.image_filename, image_data, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        
    # Based on https://github.com/opencv/opencv/blob/master/samples/python/video_threaded.py
    def threaded_save_img(self, image_data, dtime_data):

        while len(self.pending) > 0 and self.pending[0].ready():
            #res, t0 = self.pending.popleft().get()  
            t0 = self.pending.popleft()        
        if len(self.pending) < self.threadn:
            if self.threaded_mode:
                rospy.loginfo("threaded save")
                task = self.pool.apply_async(self.save_img, (image_data.copy(), dtime_data))
            else:
                task = self.DummyTask(self.save_img(image_data, dtime_data))
            self.pending.append(task)

if __name__ == "__main__":
        """
        Example entry point; please see Enumeration example for more in-depth
        comments on preparing and cleaning up the system.

        :return: True if successful, False otherwise.
        :rtype: bool
        """
        rospy.init_node('gobi_trigger')
        is_threaded = rospy.get_param('/camera/flir/multi_threaded', 'false')        
        bhg_flir = Bhg_flir(is_threaded)
        bhg_flir.run_camera()
        bhg_flir.close_camera()
        



              
