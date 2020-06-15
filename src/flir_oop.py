#! /usr/bin/python
from __future__ import print_function

# Based heavily on the FLIR Trigger_QuickSpin.py example code

import PySpin
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image        
        
class TriggerType:
    SOFTWARE = 1
    HARDWARE = 2

class Bhg_flir:

    def __init__(self):
        self.timestamp = 0
        self.chosen_trigger = TriggerType.HARDWARE
        self.image_pub = rospy.Publisher("image_color",Image, queue_size=10)
        self.cam_system = PySpin.System.GetInstance()
        self.cam_list = self.cam_system.GetCameras()
        num_cameras = self.cam_list.GetSize()

        # Finish if there are no cameras
        if num_cameras == 0:
            # Clear camera list before releasing system
            self.cam_list.Clear()

            # Release system
            self.cam_system.ReleaseInstance()

            print("Not enough cameras!")
            exit()

        self.cam = self.cam_list.GetByIndex(0)    

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

        print("*** CONFIGURING TRIGGER ***\n")

        if self.chosen_trigger == TriggerType.SOFTWARE:
            print("Software trigger chosen...")
        elif self.chosen_trigger == TriggerType.HARDWARE:
            print("Hardware trigger chosen...")

        try:
            result = True

            # Ensure trigger mode off
            # The trigger must be disabled in order to configure whether the source
            # is software or hardware.
            if self.cam.TriggerMode.GetAccessMode() != PySpin.RW:
                print("Unable to disable trigger mode (node retrieval). Aborting...")
                return False

            self.cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)

            print("Trigger mode disabled...")

            # Select trigger source
            # The trigger source must be set to hardware or software while trigger
		    # mode is off.
            if self.cam.TriggerSource.GetAccessMode() != PySpin.RW:
                print("Unable to get trigger source (node retrieval). Aborting...")
                return False

            if self.chosen_trigger == TriggerType.SOFTWARE:
                self.cam.TriggerSource.SetValue(PySpin.TriggerSource_Software)
            elif self.chosen_trigger == TriggerType.HARDWARE:
                self.cam.TriggerSource.SetValue(PySpin.TriggerSource_Line3)

            # Turn trigger mode on
            # Once the appropriate trigger source has been set, turn trigger mode
            # on in order to retrieve images using the trigger.
            self.cam.TriggerMode.SetValue(PySpin.TriggerMode_On)
            print("Trigger mode turned back on...")

        except PySpin.SpinnakerException as ex:
            print("Error: %s" % ex)
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
                    print("Unable to execute trigger. Aborting...")
                    return False

                self.cam.TriggerSoftware.Execute()

                # TODO: Blackfly and Flea3 GEV cameras need 2 second delay after software trigger

            elif self.chosen_trigger == TriggerType.HARDWARE:
                # print("Use the hardware to trigger image acquisition.")
                pass
                
        except PySpin.SpinnakerException as ex:
            print("Error: %s" % ex)
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

        print("*** IMAGE ACQUISITION ***\n")
        try:
            result = True

            # Set acquisition mode to continuous
            if self.cam.AcquisitionMode.GetAccessMode() != PySpin.RW:
                print("Unable to set acquisition mode to continuous. Aborting...")
                return False

            self.cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
            print("Acquisition mode set to continuous...")

            #  Begin acquiring images
            self.cam.BeginAcquisition()

            print("Acquiring images...")

            # Get device serial number for filename
            device_serial_number = ""
            if self.cam.TLDevice.DeviceSerialNumber.GetAccessMode() == PySpin.RO:
                device_serial_number = self.cam.TLDevice.DeviceSerialNumber.GetValue()

                print("Device serial number retrieved as %s..." % device_serial_number)

            # Retrieve, convert, and save images
            bridge = CvBridge()
            r = rospy.Rate(30) # 5hz
            last_time = rospy.get_time()
            n = 0
            while not rospy.is_shutdown():        
            
                try:
                    #  Retrieve the next image from the trigger
                    #  Does nothing if Hardware trigger except print line.
                    result &= self.grab_next_image_by_trigger()
                                    
                    #  Retrieve next received image
                    image_result = self.cam.GetNextImage(500) # timeout in milliseconds

                    #  Ensure image completion
                    if image_result.IsIncomplete():
                        print("Image incomplete with image status %d ..." % image_result.GetImageStatus())

                    else:
                        n += 1
                        #  Print image information
                        width = image_result.GetWidth()
                        height = image_result.GetHeight()
                        print("Grabbed Image %d, width = %d, height = %d" % (n, width, height))
                        
                        #  Convert image to mono 8
                        image_converted = image_result.Convert(PySpin.PixelFormat_BGR8, PySpin.HQ_LINEAR)
                        image_data = image_converted.GetNDArray()
                        
                        #cv2.imshow("frame",image_data)
                        #cv2.waitKey(1)
                        try:
                          self.image_pub.publish(bridge.cv2_to_imgmsg(image_data, "bgr8"))
                        except CvBridgeError as e:
                          print(e)

                        #  Release image
                        image_result.Release()

                except PySpin.SpinnakerException as ex:
                    print("Error: %s" % ex)

                last_time = rospy.get_time()
                r.sleep()
            # End acquisition
            self.cam.EndAcquisition()

        except PySpin.SpinnakerException as ex:
            print("Error: %s" % ex)
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
                print("Unable to disable trigger mode (node retrieval). Aborting...")
                return False

            self.cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)

            print("Trigger mode disabled...")

        except PySpin.SpinnakerException as ex:
            print("Error: %s" % ex)
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

            result &= self.print_device_info(nodemap_tldevice)

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
            print("Error: %s" % ex)
            result = False

        return result  
        
    def close_camera(self):
        # Release reference to camera
        # NOTE: Unlike the C++ examples, we cannot rely on pointer objects being automatically
        # cleaned up when going out of scope.
        # The usage of del is preferred to assigning the variable to None.
        print("Closing camera now.\n")
        del self.cam

        # Clear camera list before releasing system
        self.cam_list.Clear()

        # Release instance
        self.cam_system.ReleaseInstance()
    
if __name__ == "__main__":
        """
        Example entry point; please see Enumeration example for more in-depth
        comments on preparing and cleaning up the system.

        :return: True if successful, False otherwise.
        :rtype: bool
        """
        rospy.init_node('gobi_trigger')
        bhg_flir = Bhg_flir()
        bhg_flir.run_camera()
        bhg_flir.close_camera()
        



              
