#include "stdio.h" // C Standard Input/Output library.
#include "XCamera.h" // Xeneth SDK main header.
#include "XFilters.h" // Xeneth SDK main header.
#include <string>     // std::string, std::to_string
#include <sys/stat.h> // mkdir command

#include "ros/ros.h"
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "std_msgs/String.h" // Callback for directory needs this
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"

using namespace std;

class GobiBHG {
    private:
        XCHANDLE handle = 0; // Handle to the camera        
        dword *frameBuffer; // 16-bit buffer to store the capture frame.
        dword frameSize; // The size in bytes of the raw image.
        bool is_initialized;
        int serial_num;
        image_transport::Publisher image_pub_;
        bool record;
        int img_hz;

        ros::Subscriber record_sub;
        ros::Subscriber dir_sub;
        ros::Subscriber alt_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber vel_sub;
        ros::Subscriber mag_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber temp_sub;
        
        std::ofstream csvOutfile;
        std::string image_folder;
        std::string image_filename;
        std::string img_time;
        std::string crnt_time;
        
        // State updated by callbacks
        sensor_msgs::MagneticField mag_data;
        sensor_msgs::Imu imu_data;
        mavros_msgs::Altitude rel_alt;
        sensor_msgs::NavSatFix gps_fix;
        geometry_msgs::TwistStamped vel_gps;
        sensor_msgs::Temperature temp_imu;              
        std::string data_dir; 
        int saved_count;
        bool verbose;
        int img_delay;
  
    public:
        GobiBHG(ros::NodeHandle *nh){
            frameBuffer = 0; 
            frameSize = 0;  
            is_initialized = false;  
            serial_num = 0;
            data_dir = "/tmp/BHG_DATA";
            image_filename = "default_imgname.ppm";
            csvOutfile;
            image_transport::ImageTransport it_(*nh);
            image_pub_ = it_.advertise("gobi_image", 1); // TODO Namespace this to /camera/gobi/ , use the same pattern as FLIR  
            record = false; 
            img_hz = 5;
            saved_count = 0;
            verbose = false;
            
            
            record_sub  = nh->subscribe("/record", 10, &GobiBHG::recordCallback, this);
            dir_sub     = nh->subscribe("/directory", 1000, &GobiBHG::dirCallback, this);
            alt_sub     = nh->subscribe("/mavros/altitude", 1000, &GobiBHG::alt_cb, this);
            gps_sub     = nh->subscribe("/mavros/global_position/raw/fix", 1000, &GobiBHG::gps_cb, this);
            vel_sub     = nh->subscribe("/mavros/global_position/raw/gps_vel", 1000, &GobiBHG::vel_cb, this); 
            mag_sub     = nh->subscribe("/mavros/imu/mag", 1000, &GobiBHG::mag_cb, this);
            imu_sub     = nh->subscribe("/mavros/imu/data", 1000, &GobiBHG::imu_cb, this);
            temp_sub    = nh->subscribe("/mavros/imu/temperature_imu", 1000, &GobiBHG::temp_cb, this);            
            create_directories();
        }

        ~GobiBHG(){
            printf("***** GOBI:  Starting GOBI clean up as part of shutdown process\n");
            this->clean_up();
        }
        
        int get_savedcount(){
            return this->saved_count;
        }
         
        
        int retrieve_info(bool verbose){
            ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
            unsigned int deviceCount = 0;
            if ((errorCode = XCD_EnumerateDevices(NULL, &deviceCount, XEF_EnableAll)) != I_OK) {
                ROS_INFO("***** GOBI:  An error occurred while enumerating the devices. errorCode: %i", int(errorCode));
                return -1;
            }
            if (deviceCount == 0) {
                ROS_INFO("***** GOBI:  Enumeration was a success but no devices were found!");
                return 0;
            }         
            XDeviceInformation *devices = new XDeviceInformation[deviceCount];
            if ((errorCode = XCD_EnumerateDevices(devices, &deviceCount, XEF_UseCached)) != I_OK) {
                ROS_INFO("***** GOBI:  Error while retrieving the cached device information structures. errorCode: %i", int(errorCode));
                delete [] devices;
                return -1;
            }

            /*  All discovered devices are now available in our local array and we are now able 
             *  to iterate the list and output each item in the array */

            for(unsigned int i = 0; i < deviceCount; i++) {
                XDeviceInformation * dev = &devices[i];
                if (verbose){
                    ROS_INFO("***** GOBI:  device[%i] %s @ %s (%s) ", i, dev->name, dev->address, dev->transport);
                    ROS_INFO("***** GOBI:  PID: %4X", dev->pid); 
                    ROS_INFO("***** GOBI:  Serial: %i", dev->serial);
                    ROS_INFO("***** GOBI:  URL: %s", dev->url);
                    ROS_INFO("***** GOBI:  State: %s", dev->state == XDS_Available ? "Available" : dev->state == XDS_Busy ? "Busy" : "Unreachable");
                }
                if (std::string(dev->name).rfind("Gobi") == 0){
                    this->serial_num = dev->serial;                    
                }
            }
            //ROS_INFO("***** GOBI:  Serial number is: %i", this->serial_num);
            delete [] devices;   
            return 0;    
        }
        
        void initialize_cam(int is_trigMode = 0, int _img_hz = 5, int _img_delay = 20000){ // 0: no triger, 1: input trigger, 2: output trigger
            this->img_hz = img_hz;
            this->img_delay = _img_delay;
            ROS_INFO("***** GOBI:  Img Delay is: %d and img_hz is: %d",this->img_delay,this->img_hz );
            ErrCode errCode = I_OK;
            // Open a connection to the first detected camera by using connection string cam://0
            //ROS_INFO("***** GOBI:  Opening connection to cam://0");
            this->handle = XC_OpenCamera("cam://0"); 
            if(XC_IsInitialised(handle)){             
                //ROS_INFO("***** GOBI:  Is initialized.");
                this->is_initialized = true;    
                
                /* retrieve camera product id (PID)  and serial number*/
                long pid = 0;
                long ser = 0;
                long timeout = 0;
                errCode = XC_GetPropertyValueL(this->handle, "_CAM_PID", &pid);
                if (!HandleError(errCode, "Retrieving the camera PID")) AbortSession(); 
                errCode = XC_GetPropertyValueL(this->handle, "_CAM_SER", &ser);
                if (!HandleError(errCode, "Retrieving the camera serial number")) AbortSession();        
                ROS_INFO("***** GOBI:  Connected to camera with product id (PID) 0x%ld and serial number %lu", pid, ser);
                errCode = XC_GetPropertyValueL(this->handle, "_API_GETFRAME_TIMEOUT", &timeout);
                if (!HandleError(errCode, "Retrieving the camera frame timeout")) AbortSession();        
                //ROS_INFO("***** GOBI:  Frame timeout is: %lu", timeout);
       
                Setup_F027(this->handle, is_trigMode);
            }
            else{
                ROS_ERROR("***** GOBI:  Initialization failed");
                this->is_initialized = false;    
            }
        }
        
        /*
         *  In SetupShutterControl_F027 we disable the automatic shutter correction.
         *  When this is set to enabled it is possible that triggers being received 
         *  during a calibrate cycle are not processed by the camera.
         *  To make sure the image does not drift and stays corrected the camera has
         *  to be occasionally calibrated by stopping / starting the acquisition or
         *  executing the "Calibrate"-property by setting its value to 1.
         */

        bool SetupShutterControl(XCHANDLE handle) {

            ErrCode errCode = I_OK;

            //printf("Configuring camera to disable the automatic shutter control: \n");

            /* 
             *  AutoCorrectionEnabled = Disabled (0), Enabled (1)
             * -----------------------------------------------------------------
             *  Disable the automatic internal calibration. 
             */

            errCode = XC_SetPropertyValueL(handle, "AutoCorrectionEnabled", 0, "");
            if (!HandleError(errCode, " * Disable auto correction"))
                return false;
            
            printf("\n");
            return true;
        }


        /*
         *  To make sure the image does not drift and stays corrected between bursts 
         *  the camera has to be occasionally calibrated by stopping / starting the
         *  acquisition or by executing the "Calibrate"-property between bursts by setting 
         *  its value to 1. In the case that automatic correction was enabled the calibration
         *  operation occurs every 150 seconds or when the devices temperature deviates 0.5
         *  degrees from the last measured point.
         *
         *  Automatic correction while we are expecting triggers can cause events to 
         *  be missed. Because of this the calibration progress has to be controlled.
         */

        bool ExecuteCalibration(XCHANDLE handle) {

            ErrCode errCode = I_OK;
            
            errCode = XC_SetPropertyValueL(handle, "Calibrate", 1, "");
            if (!HandleError(errCode, "Perform calibration"))
                return false;

            usleep(3000000);

            return true;
        }        
        
        

        /*
         *  In Setup_F027 we configure the camera in one of three modes: 
         *  Mode 0: free run no trigger
         *  Mode 1: external trigger in mode with rising edge activation 
         *  Mode 2: in external trigger out mode
         */
        bool Setup_F027(XCHANDLE handle, int trig_mode = 0) {
            //ROS_INFO("***** GOBI:  BHG trigger mode is: %d. (0 = No trigger | 1 = External In | 2 = External Out)",trig_mode);
            //ROS_INFO("***** GOBI:  Image HZ is %i", this->img_hz );
            ErrCode errorCode = I_OK;      
            long wavelength = long(1.0/img_hz * 1000000); 
            long duty_cycle = wavelength / 2;  
           
            if (trig_mode == 1){
                ROS_INFO("***** GOBI:  Configuring camera in external 'TRIGGER IN' mode with rising edge activation");
                // TriggerInputDelay Defines the time delay (in microseconds) between the trigger and the actual start of integration. 
                errorCode = XC_SetPropertyValueL(handle, "TriggerInDelay", this->img_delay, "");
                if (!HandleError(errorCode, " * Set TriggerInDelay"))
                    return false; 
                errorCode = XC_SetPropertyValueL(handle, "TriggerInTiming", 1, "");
                if (!HandleError(errorCode, " * Set trigger input timing"))
                    return false;                    
                errorCode = XC_SetPropertyValueL(handle, "AutoModeUpdate", 0, "");
                if (!HandleError(errorCode, " * Set auto mode"))
                    return false;
                errorCode = XC_SetPropertyValueL(handle, "TriggerDirection", 1, "");
                if (!HandleError(errorCode, " * Set trigger direction"))
                    return false;
                errorCode = XC_SetPropertyValueL(handle, "TriggerInMode", 0, "");
                if (!HandleError(errorCode, " * Set trigger input mode")) 
                    return false;            
                errorCode = XC_SetPropertyValueL(handle, "TriggerInEnable", 1, "");
                if (!HandleError(errorCode, " * Enable trigger input"))
                    return false;            
                errorCode = XC_SetPropertyValueL(handle, "TriggerInSensitivity", 0, "");
                if (!HandleError(errorCode, " * Set trigger input sensitivity"))
                    return false;            
                errorCode = XC_SetPropertyValueL(handle, "TriggerInPolarity", 1, "");
                if (!HandleError(errorCode, " * Set trigger input polarity"))
                    return false;                    
                errorCode = XC_SetPropertyValueL(handle, "TriggerOutEnable", 0, "");
                if (!HandleError(errorCode, " * Disable trigger output"))
                    return false;            
                errorCode = XC_SetPropertyValueL(handle, "TriggerOutPolarity", 0, "");
                if (!HandleError(errorCode, " * Set TriggerOutPolarity")) 
                    return false;
            }
            else if (trig_mode == 2){             
                errorCode = XC_SetPropertyValueL(handle, "AutoModeUpdate", 0, "");
                if (!HandleError(errorCode, " * Set auto mode"))
                    return false;            
                ROS_INFO("***** GOBI:  Configuring camera in external 'TRIGGER OUT' mode");        
                errorCode = XC_SetPropertyValueL(handle, "TriggerDirection", 0, "");
                if (!HandleError(errorCode, " * Set trigger direction"))
                    return false; 
                errorCode = XC_SetPropertyValueL(handle, "TriggerInMode", 1, "");
                if (!HandleError(errorCode, " * Set trigger input mode"))
                    return false;   
                errorCode = XC_SetPropertyValueL(handle, "TriggerInEnable", 0, "");
                if (!HandleError(errorCode, " * Enable trigger input"))
                    return false;                          
                errorCode = XC_SetPropertyValueL(handle, "TriggerOutEnable", 1, "");
                if (!HandleError(errorCode, " * Disable trigger output"))
                    return false;           
                errorCode = XC_SetPropertyValueL(handle, "TriggerOutPolarity", 1, "");
                if (!HandleError(errorCode, " * Set TriggerOutPolarity")) 
                    return false;
                // MinimumFrameTime Values: Time in microseconds. This limits the camera capture rate
                // and consequentially the trigger rate.   
                errorCode = XC_SetPropertyValueL(handle, "MinimumFrameTime", wavelength, "");
                if (!HandleError(errorCode, " * Set MinimumFrameTime"))
                    return false;                   
                errorCode = XC_SetPropertyValueL(handle, "TriggerOutWidth", duty_cycle, "");
                if (!HandleError(errorCode, " * Set TriggerOutWidth  ")) 
                    return false;             
            }
            else // no trigger mode
            {
                ROS_INFO("***** GOBI:  Configuring camera in free run mode with no trigger.");
                errorCode = XC_SetPropertyValueL(handle, "AutoModeUpdate", 1, "");
                if (!HandleError(errorCode, " * Set auto mode"))
                    return false;            
                errorCode = XC_SetPropertyValueL(handle, "TriggerInEnable", 0, "");
                if (!HandleError(errorCode, " * Enable trigger input"))
                    return false;                             
                errorCode = XC_SetPropertyValueL(handle, "TriggerOutEnable", 0, "");
                if (!HandleError(errorCode, " * Disable trigger output"))
                    return false;
                errorCode = XC_SetPropertyValueL(handle, "TriggerInMode", 1, "");
                if (!HandleError(errorCode, " * Set trigger input mode")) 
                    return false;                            
            }
            /* configure camera to disable the automatic shutter calibration process */
            if (!SetupShutterControl(handle)) AbortSession();
            
            // GainControl Values: Automatic (=0), Manual (=1)
            errorCode = XC_SetPropertyValueL(handle, "GainControl", 0, "");
            if (!HandleError(errorCode, " * Set GainControl"))
                return false; 
            // OffsetControl Values: Automatic (=0), Manual (=1) 
            errorCode = XC_SetPropertyValueL(handle, "OffsetControl", 0, "");
            if (!HandleError(errorCode, " * Set OffsetControl"))
                return false;    
            // AutoCorrectionEnabled Values: Off(0), On(1)                         
            errorCode = XC_SetPropertyValueL(handle, "AutoCorrectionEnabled", 1, "");
            if (!HandleError(errorCode, " * Set AutoCorrectionEnabled"))
                return false; 
            if (verbose){
                long triggerInDelay = 0;
                errorCode = XC_GetPropertyValueL(handle, "TriggerInDelay", &triggerInDelay);
                ROS_INFO("***** GOBI:  TriggerInDelay '%lu' . "
                                "Values: delay in microseconds between the trigger and start of integration", triggerInDelay);
                long gaincontrol = 0;
                errorCode = XC_GetPropertyValueL(handle, "GainControl", &gaincontrol);
                ROS_INFO("***** GOBI:  GainControl '%lu' Global gain factor applied to the image is computed automatically. "
                                "Values: Automatic (=0), Manual (=1)", gaincontrol);                            
                long offsetControl = 0;
                errorCode = XC_GetPropertyValueL(handle, "OffsetControl", &offsetControl);
                ROS_INFO("***** GOBI:  OffsetControl '%lu' The offset applied to the image is computed automatically. "
                                "Values: Automatic (=0), Manual (=1)", offsetControl);     
                long autoCorrectionEnabled = 0;
                errorCode = XC_GetPropertyValueL(handle, "AutoCorrectionEnabled", &autoCorrectionEnabled);
                ROS_INFO("***** GOBI:  AutoCorrectionEnabled '%lu' Enables the automatic internal calbration. "
                                "Values: Automatic (=0), Manual (=1)", autoCorrectionEnabled);
                long automode = 1;
                errorCode = XC_GetPropertyValueL(handle, "AutoModeUpdate", &automode);
                ROS_INFO("***** GOBI:  AutoModeUpdate '%lu' Freeze the current values for the offset and the gain. Values: Running (=0), Stopped (=1)", automode);             
                long trigimode = 0; 
                errorCode = XC_GetPropertyValueL(handle, "TriggerInMode", &trigimode);
                ROS_INFO("***** GOBI:  TriggerInMode is '%lu' Sets the current trigger mode. Values:Free running(0), Triggered(1)", trigimode);        
                long trigienable = 0;
                errorCode = XC_GetPropertyValueL(handle, "TriggerInEnable", &trigienable);
                ROS_INFO("***** GOBI:  TriggerInEnable is '%lu' Enables the trigger input. Values: Off(0), On(1)", trigienable);        
                long trigisens = 0;
                errorCode = XC_GetPropertyValueL(handle, "TriggerInSensitivity", &trigisens);
                ROS_INFO("***** GOBI:  TriggerInSensitivity is '%lu' When the camera captures an image. Values: Level(0), Edge(1)", trigisens);
                long trigipol = 0;
                errorCode = XC_GetPropertyValueL(handle, "TriggerInPolarity", &trigipol);
                ROS_INFO("***** GOBI:  TriggerInPolarity is '%lu' Whether images are captured if the signal is high or low. "
                                        "Values: Level low/Falling edge(0), Level high/Rising edge(1)", trigipol);
                long trigoenable = 0;
                errorCode = XC_GetPropertyValueL(handle, "TriggerOutEnable", &trigoenable);
                ROS_INFO("***** GOBI:  TriggerOutEnable is '%lu' Enables trigger output. Values: Off(0), On(1)", trigoenable);
                long trigopolarity = 0;
                errorCode = XC_GetPropertyValueL(handle, "TriggerOutPolarity", &trigopolarity);
                ROS_INFO("***** GOBI:  TriggerOutPolarity is '%lu' Values: Active low (0), Active high (1)", trigoenable);
                long trigowdith = 0;
                errorCode = XC_GetPropertyValueL(handle, "TriggerOutWidth", &trigowdith);
                ROS_INFO("***** GOBI:  TriggerOutWidth is '%lu' Duration of the active output after a trigger event. Values: Time in microseconds", trigowdith);
                long minimumFrameTime = 0; 
                errorCode = XC_GetPropertyValueL(handle, "MinimumFrameTime", &minimumFrameTime);
                float hz = 1000000.0 / minimumFrameTime;
                ROS_INFO("***** GOBI:  MinimumFrameTime is '%lu' Values: Time in microseconds or %.1f hz", minimumFrameTime,hz);  
            }
            ExecuteCalibration(handle); 
            return true;        
        }

        /*
         *  Utility function to handle error messages 
         */        
        bool HandleError(ErrCode errCode, const char * msg) {
            const int sz = 2048;
            char err_msg[sz];

            XC_ErrorToString(errCode, err_msg, sz);
            if (errCode != 0){
                ROS_ERROR("***** GOBI:  %s: %s (%lu)", msg, err_msg, errCode);
            }
            return I_OK == errCode;
        }        
        
        void AbortSession() {
            ROS_ERROR("***** GOBI:  Aborting session.");
            CleanupSession();
            exit(-1);
        }

        void CleanupSession() {
            /* cleanup frame buffer */
            if (this->frameBuffer != 0) free(this->frameBuffer);
            this->frameBuffer = 0;

            /* make sure capturing has stopped */
            if (XC_IsCapturing(this->handle)) XC_StopCapture(this->handle);

            /* close the session */
            XC_CloseCamera(this->handle);
        }
       
        void start_capture(){
            ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
            if ((errorCode = XC_StartCapture(this->handle)) != I_OK)
            {
                ROS_ERROR("***** GOBI:  Could not start capturing, errorCode: %lu", errorCode);
            }
            else if (XC_IsCapturing(this->handle)) // When the camera is capturing ...
            {    
            
                // Load the color profile delivered with this sample.
                if (errorCode = XC_LoadColourProfile(handle, "/home/user1/catkin_ws/src/usma_bhg/resources/ThermalBlue.png") != I_OK)
                {
                    ROS_ERROR("***** GOBI:  Problem while loading the desired colorprofile, errorCode: %lu", errorCode);
                }
                else
                {
                    ROS_INFO("***** GOBI:  Successfully loaded the desired colorprofile.");
                }        
                // Set the colourmode so that the last loaded colorprofile is used.
                XC_SetColourMode(handle, ColourMode_Profile);
                
                // TODO use the camera calibration file.
                XC_LoadCalibration(this->handle, "pack.xca", XLC_StartSoftwareCorrection);

                // Load thermography filter
                //int thermalFilterId = XC_FLT_Queue(this->handle, "Thermography", 0);

                // Determine native framesize.
                //this->frameSize = XC_GetFrameSize(this->handle);
                this->frameSize = XC_GetWidth(handle) * XC_GetHeight(handle); // currently = 307200 which is 640x480

                // Initialize the 16-bit buffer.
                //this->frameBuffer = new word[this->frameSize / 2];  
                this->frameBuffer = new dword[this->frameSize];  
            }
        }

        int retrieve_frame(){
            ErrCode errorCode = 0; 
            //if ((errorCode = XC_GetFrame(this->handle, FT_NATIVE, XGF_Blocking, this->frameBuffer, this->frameSize)) != I_OK)
            //if ((errorCode = XC_GetFrame(this->handle, FT_32_BPP_RGB, XGF_Blocking, this->frameBuffer, this->frameSize * 3)) != I_OK)
            //std::string crnt_time = "";
            if ((errorCode = XC_GetFrame(this->handle, FT_32_BPP_RGBA, XGF_Blocking, this->frameBuffer, this->frameSize * 4)) != I_OK)
            {
                if (errorCode == 10008){
                    ROS_INFO_THROTTLE(10,"***** GOBI:  Retrieve frame timed out waiting for frame (possibly not triggered), Code %lu", errorCode);                
                }
                else{
                    ROS_ERROR_THROTTLE(10,"***** GOBI:  Problem while fetching frame, errorCode %lu", errorCode);
                }
            }
            else {
                cv::Mat cv_image(cv::Size(640, 480), CV_8UC4, this->frameBuffer); 
                cv::cvtColor(cv_image , cv_image, CV_RGBA2BGR);
                this->crnt_time = make_datetime_stamp();
                if (this->record){
                    this->image_filename = image_folder + "GOBI" + to_string(this->serial_num) + "_" + this->crnt_time + ".png";
                    //errorCode = XC_SaveData(this->handle, "output.png", XSD_SaveThermalInfo | XSD_Force16); 
                    cv::imwrite( this->image_filename, cv_image);
                    this->csvOutfile << make_logentry() << std::endl;
                    this->saved_count++;
                }

                cv::putText(cv_image, this->crnt_time, cvPoint(30,400), 
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(1,1,1), 2);
                cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
                cv_ptr->encoding = "bgr8";
                cv_ptr->header.stamp =  ros::Time::now();
                cv_ptr->header.frame_id = "gobi";
                cv_ptr->image = cv_image;
                this->image_pub_.publish(cv_ptr->toImageMsg());  
            }
            return int(errorCode);
        }
        
        void clean_up(){
        // NOTE: This function can't use ROS_INFO since ROS is shutdown it is run.
            printf("***** GOBI:  Conducting ROS clean up\n");
            ErrCode errorCode = 0; 
            // When the camera is still capturing, ...
            if(XC_IsCapturing(this->handle))
            {
                // ... stop capturing.
                printf("***** GOBI:  Stop capturing.\n");
                if ((errorCode = XC_StopCapture(this->handle)) != I_OK)
                {
                    printf("***** GOBI:  Could not stop capturing, errorCode: %lu\n", errorCode);
                }
            }

            // When the handle to the camera is still initialised ...
            if (XC_IsInitialised(this->handle))
            {
                printf("***** GOBI:  Closing connection to camera.\n");
                XC_CloseCamera(this->handle);
            }

            printf("***** GOBI:  Clearing buffers.\n");
            if (this->frameBuffer != 0)
            {
                delete [] this->frameBuffer;
                this->frameBuffer = 0;
            }       
        }

        // Make a string with the date time stamp formatted for file names. Includes milliseconds.
        std::string make_datetime_stamp(){
            char buffer [80];    
            ros::Time ros_timenow = ros::Time::now();
            std::time_t raw_time = static_cast<time_t>(ros_timenow.toSec());    
            struct tm * local_tm = localtime(&raw_time);        
            int ros_millisec = int((ros_timenow.nsec)/1000);         
            strftime (buffer,80,"%Y%m%d_%H%M%S",local_tm);
            sprintf(buffer,"%s_%06d", buffer, ros_millisec);
            std::string datetime_stamp(buffer);
            return datetime_stamp;                    
        }
        
        bool create_directories(){
            this->data_dir.pop_back();
            std::string dir_time(data_dir.substr(data_dir.rfind("/")));

            this->image_folder = data_dir + "/GOBI_SN_" + to_string(this->serial_num) + "/";
            string csv_filename   = data_dir + dir_time + "_gobi.csv";

            if (mkdir(this->image_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1) {
                if( errno == 0 ) { // TODO fix this to return accurate feedback
                    // TODO figure out why this check for zero? 
                    ROS_INFO("***** GOBI:  0 Directory does not exist and MKDIR failed the errno is %i",errno ); 
                }
                else if (errno == 17){
                    // The directory already exists. Do nothing
                }
                else{
                    ROS_INFO("***** GOBI: Directory does not exist and MKDIR failed the errno is %i",errno );        
                }
            }    
            else {
                //ROS_INFO("***** GOBI:  Created Data Directory and establishing csv file" );  
                ROS_INFO("***** GOBI:  Data directory is: [%s]", this->image_folder.c_str());
                
                if (csvOutfile.is_open()){ // Close the old csv file
                    ROS_INFO("***** GOBI:  GOBI CSV File is already open: [%s]", csv_filename.c_str());
                    csvOutfile.close();
                }                        
                csvOutfile.open(csv_filename, std::ios_base::app); // open new csv file
                if (!csvOutfile){
                    ROS_ERROR("***** GOBI:  ERROR   FAILED TO OPEN GOBI CSV File: %s,  [%s]", strerror(errno), csv_filename.c_str()); 
                    csvOutfile.open(csv_filename, std::ios_base::app); // DML is unrealable in creating the initial file           
                    ROS_ERROR("***** GOBI:  ERROR   BLINDLY TRYIED TO OPEN csv AGAIN: %s,  [%s]", strerror(errno), csv_filename.c_str());           
                } 
                ROS_INFO("***** GOBI:  CSV file is: [%s]", csv_filename.c_str()); 
                csvOutfile << make_header() << endl;            
            }                      
        }
        
        void dirCallback(const std_msgs::String::ConstPtr& msg)
        {
            this->data_dir = msg->data.c_str();
            create_directories();
        }
        
        // Make the header for the csv file
        // TODO clean this up, a lot of unused columns that printing zeros. Formating of code needs cleanup also.
        string make_header()
        {
            string header = "";
            header = "filename,rostime,rel_alt.monotonic,rel_alt.amsl,rel_alt.local,rel_alt.relative,";
            header += "gps_fix.status.status,gps_fix.status.service,gps_fix.latitude,gps_fix.longitude,gps_fix.altitude,";
            header += "mag_data.magnetic_field.x,mag_data.magnetic_field.y,mag_data.magnetic_field.z,";
            header += "imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w,"; 
            header += "imu_data.angular_velocity.x,imu_data.angular_velocity.y,imu_data.angular_velocity.z,";
            header += "imu_data.linear_acceleration.x,imu_data.linear_acceleration.y,imu_data.linear_acceleration.z,";
            header += "vel_gps.twist.linear.x,vel_gps.twist.linear.y,vel_gps.twist.linear.z,";
            header += "vel_gps.twist.angular.x,vel_gps.twist.angular.y,vel_gps.twist.angular.z,";
            header += "temp_imu.temperature";
            return header;
        }        

        string make_logentry()
        {            
            string alt_str = image_filename + "," + this->crnt_time + "," 
                      + to_string(rel_alt.monotonic) + "," + to_string(rel_alt.amsl) + "," 
                      + to_string(rel_alt.local) + "," + to_string(rel_alt.relative); 
            string gps_str = to_string(gps_fix.status.status) + "," + to_string(gps_fix.status.service) 
                      + "," + to_string(gps_fix.latitude) + "," + to_string(gps_fix.longitude) + "," + to_string(gps_fix.altitude);
            string mag_str = to_string(mag_data.magnetic_field.x) + "," + to_string(mag_data.magnetic_field.y) 
                      + "," + to_string(mag_data.magnetic_field.z);    
            string imu_str = to_string(imu_data.orientation.x) + "," + to_string(imu_data.orientation.y) + "," 
                      + to_string(imu_data.orientation.z) + "," + to_string(imu_data.orientation.w) + ",";
            imu_str += to_string(imu_data.angular_velocity.x) + "," + to_string(imu_data.angular_velocity.y) 
                      + "," + to_string(imu_data.angular_velocity.z) + ",";
            imu_str += to_string(imu_data.linear_acceleration.x) + "," + to_string(imu_data.linear_acceleration.y) 
                      + "," + to_string(imu_data.linear_acceleration.z);    
            string vel_str = to_string(vel_gps.twist.linear.x) + "," + to_string(vel_gps.twist.linear.y) + "," + to_string(vel_gps.twist.linear.z);
            vel_str += to_string(vel_gps.twist.angular.x) + "," + to_string(vel_gps.twist.angular.y) + "," + to_string(vel_gps.twist.angular.z);  
            string temp_str = to_string(temp_imu.temperature);
            string output = alt_str + "," + gps_str + "," + mag_str + "," + imu_str + "," + vel_str + "," + temp_str;
            return output;
        }
        
        string char_array_to_string(char* char_array)
        {
            string my_string(char_array);

            return my_string;
        }
        
        void mag_cb(const sensor_msgs::MagneticField::ConstPtr& msg)
        {
            mag_data = *msg;
        }

        void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
        {
            imu_data = *msg;
        }

        void alt_cb(const mavros_msgs::Altitude::ConstPtr& msg)
        {
            rel_alt = *msg;
        }

        void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
        {
            gps_fix = *msg;
        }

        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
        {
            vel_gps = *msg;
        }

        void temp_cb(const sensor_msgs::Temperature::ConstPtr& msg)
        {
            temp_imu = *msg;
        }

        void recordCallback(const std_msgs::Bool msg)
        {
            this->record = msg.data; 
            ROS_INFO("***** GOBI:  RECORD CALLBACK FIRED WITH A VALUE OF: [%d]", this->record);
        }
};
  
int main(int argc, char **argv)
{
    // start the ros node
    ros::init(argc, argv, "gobi_trigger");
    ros::NodeHandle nh;

    // get ros param for trigger mode. 0 - trigger in, 1 - trigger out, 2- no trigger
    int use_trig; 
    if (nh.hasParam("/camera/gobi/use_trig")){  
        nh.getParam("/camera/gobi/use_trig", use_trig);
    }
    else{ // Default to no trigger mode
        use_trig = 2;
    }
    //ROS_INFO("Use trigger mode is set to: %i",use_trig);
    int capture_hz; 
    if (nh.hasParam("/camera/gobi/capture_hz")){  
        nh.getParam("/camera/gobi/capture_hz", capture_hz);
    }
    else{ // Default to 5 hz
        capture_hz = 5;
    }
    int delay; 
    if (nh.hasParam("/camera/gobi/delay")){  
        nh.getParam("/camera/gobi/delay", delay);
    }
    else{ // Default to 5 hz
        delay = 20723;
    }
    //ROS_INFO("Images will be captured at: %i hz.",capture_hz); 

    // Setup camera and start capturing 
    GobiBHG gobi_cam(&nh); 
    gobi_cam.retrieve_info(false); // Not using this as it prints all cameras. 
    gobi_cam.initialize_cam(use_trig, capture_hz, delay);
    gobi_cam.start_capture();
 
    // Big while loop, continuously publish the images
    uint64_t n=0;
    ros::Rate loop_rate(40); //This should be faster than the camera capture rate.  
    // NOTE: We are limiting the camera to 20HZ frames and this loop uses a blocking call 
    // for get frame. Therefore this loop will not go any faster than the camera. By setting
    // the loop rate to 25hz, we attempt to have the loop wait at the blocking call for the image.
    while (ros::ok())
    {
        if(gobi_cam.retrieve_frame() ==0){        
            n++;
            ROS_INFO_THROTTLE(10,"***** GOBI:  Grabbed Image %lu, and saved %d", n, gobi_cam.get_savedcount());
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("***** GOBI:  Received ROS shutdown command");

    return 0;
}
