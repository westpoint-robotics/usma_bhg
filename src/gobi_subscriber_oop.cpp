#include "stdio.h" // C Standard Input/Output library.
#include "XCamera.h" // Xeneth SDK main header.
#include "XFilters.h" // Xeneth SDK main header.
#include<string>

#include "ros/ros.h"
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>     // std::string, std::to_string


using namespace std;


class GobiBHG {
    private:
        XCHANDLE handle = 0; // Handle to the camera        
        dword *frameBuffer; // 16-bit buffer to store the capture frame.
        dword frameSize; // The size in bytes of the raw image.
        bool is_initialized;
        int serial_num;
        image_transport::Publisher image_pub_;
       
    public:
        GobiBHG(ros::NodeHandle *nh){
            frameBuffer = 0; 
            frameSize = 0;  
            is_initialized = false;  
            serial_num = 0;
            image_transport::ImageTransport it_(*nh);
            image_pub_ = it_.advertise("gobi_image", 1); // TODO Namespace this to /camera/gobi/ , use the same pattern as FLIR           
        }
        
        ~GobiBHG(){
            printf("**** GOBI **** Starting GOBI clean up as part of shutdown process\n");
            this->clean_up();
        }
        
        int retrieve_info(){
            ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
            unsigned int deviceCount = 0;
            if ((errorCode = XCD_EnumerateDevices(NULL, &deviceCount, XEF_EnableAll)) != I_OK) {
                ROS_INFO("**** GOBI **** An error occurred while enumerating the devices. errorCode: %i", int(errorCode));
                return -1;
            }
            if (deviceCount == 0) {
                ROS_INFO("**** GOBI **** Enumeration was a success but no devices were found!");
                return 0;
            }         
            XDeviceInformation *devices = new XDeviceInformation[deviceCount];
            if ((errorCode = XCD_EnumerateDevices(devices, &deviceCount, XEF_UseCached)) != I_OK) {
                ROS_INFO("**** GOBI **** Error while retrieving the cached device information structures. errorCode: %i", int(errorCode));
                delete [] devices;
                return -1;
            }

            /*  All discovered devices are now available in our local array and we are now able 
             *  to iterate the list and output each item in the array */

            for(unsigned int i = 0; i < deviceCount; i++) {
                XDeviceInformation * dev = &devices[i];
                ROS_INFO("**** GOBI **** device[%i] %s @ %s (%s) ", i, dev->name, dev->address, dev->transport);
                ROS_INFO("**** GOBI **** PID: %4X", dev->pid); 
                ROS_INFO("**** GOBI **** Serial: %i", dev->serial);
                ROS_INFO("**** GOBI **** URL: %s", dev->url);
                ROS_INFO("**** GOBI **** State: %s", dev->state == XDS_Available ? "Available" : dev->state == XDS_Busy ? "Busy" : "Unreachable");
                if (std::string(dev->name).rfind("Gobi") == 0){
                    this->serial_num = dev->serial;                    
                }
            }
            ROS_INFO("**** GOBI **** Gobi serial number is: %i", this->serial_num);
            delete [] devices;   
            return 0;    
        }
        
        void initialize_cam(int is_trigMode = 2){
            ErrCode errCode = I_OK;
            // Open a connection to the first detected camera by using connection string cam://0
            ROS_INFO("**** GOBI **** Opening connection to cam://0");
            this->handle = XC_OpenCamera("cam://0"); 
            if(XC_IsInitialised(handle)){             
                ROS_INFO("**** GOBI **** GOBI is initialized.");
                this->is_initialized = true;    
                
                /* retrieve camera product id (PID)  and serial number*/
                long pid = 0;
                long ser = 0;
                errCode = XC_GetPropertyValueL(this->handle, "_CAM_PID", &pid);
                if (!HandleError(errCode, "Retrieving the camera PID")) AbortSession(); 
                errCode = XC_GetPropertyValueL(this->handle, "_CAM_SER", &ser);
                if (!HandleError(errCode, "Retrieving the camera serial number")) AbortSession();        
                ROS_INFO("**** GOBI **** Connected to camera with product id (PID) 0x%ld and serial number %lu", pid, ser);

//                /* Check for the Gobi-640-GigE (F027) and in trigger mode*/
//                if (pid == 0xF027 and is_trigMode) {

//                    /* configure camera in external triggered mode */
//                    if (!SetupExternalTriggeredMode_F027(handle)) AbortSession();

//                    /* configure camera to disable the automatic shutter calibration process */
//                    //if (!SetupShutterControl_F027(handle)) AbortSession();
//                }
//                else if (pid == 0xF027) {
//                    /* configure camera with no trigger */
//                    if (!Setup_F027(handle)) AbortSession();
//                }
                Setup_F027(this->handle, is_trigMode);
                
                //SetupExternalTriggeredMode_F027(this->handle);
            }
            else{
                ROS_INFO("**** GOBI **** GOBI initialization failed");
                this->is_initialized = false;    
            }
        }

        /*
         *  In Setup_F027 we configure the camera in one of three modes: 
         *  Mode 0: external trigger in mode with rising edge activation
         *  Mode 1: in external trigger out mode 
         *  Mode 2: free run no trigger
         */
        bool Setup_F027(XCHANDLE handle, int trig_mode = 3) {
            ROS_INFO("========================================Trigger mode is: %d",trig_mode);
            ErrCode errorCode = I_OK;
            errorCode = XC_SetPropertyValueL(handle, "GainControl", 0, "");
            if (!HandleError(errorCode, " * Set GainControl"))
                return false; 
            errorCode = XC_SetPropertyValueL(handle, "OffsetControl", 0, "");
            if (!HandleError(errorCode, " * Set OffsetControl"))
                return false;                             
            errorCode = XC_SetPropertyValueL(handle, "AutoCorrectionEnabled", 1, "");
            if (!HandleError(errorCode, " * Set AutoCorrectionEnabled"))
                return false;  
           
            if (trig_mode == 0){
                ROS_INFO("**** GOBI **** Configuring GOBI camera in external trigger in mode with rising edge activation");
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
                errorCode = XC_SetPropertyValueL(handle, "TriggerOutWidth", 30, "");
                if (!HandleError(errorCode, " * Set TriggerOutWidth  ")) 
                    return false;
            }
            else if (trig_mode == 1){
                ROS_INFO("**** GOBI **** Configuring GOBI camera in external trigger out mode");        
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
                errorCode = XC_SetPropertyValueL(handle, "TriggerOutWidth", 2500, "");
                if (!HandleError(errorCode, " * Set TriggerOutWidth  ")) 
                    return false; 
                errorCode = XC_SetPropertyValueL(handle, "AutoModeUpdate", 1, "");
                if (!HandleError(errorCode, " * Set auto mode"))
                    return false;             
            }
            else // no trigger mode
            {
                ROS_INFO("**** GOBI **** Configuring GOBI camera in free run mode with no trigger.");
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
            
            long automode = 1;
            errorCode = XC_GetPropertyValueL(handle, "AutoModeUpdate", &automode);
            ROS_INFO("**** GOBI **** AutoModeUpdate '%lu' ", automode);             
            long trigimode = 0; 
            errorCode = XC_GetPropertyValueL(handle, "TriggerInMode", &trigimode);
            ROS_INFO("**** GOBI **** TriggerInMode is '%lu' Values:Free running(0), Triggered(1)", trigimode);        
            long trigienable = 0;
            errorCode = XC_GetPropertyValueL(handle, "TriggerInEnable", &trigienable);
            ROS_INFO("**** GOBI **** TriggerInEnable is '%lu' Values: Off(0), On(1)", trigienable);        
            long trigisens = 0;
            errorCode = XC_GetPropertyValueL(handle, "TriggerInSensitivity", &trigisens);
            ROS_INFO("**** GOBI **** TriggerInSensitivity is '%lu' Values: Level(0), Edge(1)", trigisens);
            long trigipol = 0;
            errorCode = XC_GetPropertyValueL(handle, "TriggerInPolarity", &trigipol);
            ROS_INFO("**** GOBI **** TriggerInPolarity is '%lu' Values: Level low/Falling edge(0), Level high/Rising edge(1)", trigipol);
            long trigoenable = 0;
            errorCode = XC_GetPropertyValueL(handle, "TriggerOutEnable", &trigoenable);
            ROS_INFO("**** GOBI **** TriggerOutEnable is '%lu' Values: Off(0), On(1)", trigoenable);
            long trigopolarity = 0;
            errorCode = XC_GetPropertyValueL(handle, "TriggerOutPolarity", &trigopolarity);
            ROS_INFO("**** GOBI **** TriggerOutEnable is '%lu' Values: Off(0), On(1)", trigoenable);
            long trigowdith = 0;
            errorCode = XC_GetPropertyValueL(handle, "TriggerOutWidth", &trigowdith);
            ROS_INFO("**** GOBI **** TriggerOutWidth is '%lu' Values: Time in microseconds", trigowdith);
            
            return true;        
        }

        /*
         *  Utility function to handle error messages 
         */        
        bool HandleError(ErrCode errCode, const char * msg) {
            const int sz = 2048;
            char err_msg[sz];

            XC_ErrorToString(errCode, err_msg, sz);
            ROS_INFO("**** GOBI **** %s: %s (%lu)", msg, err_msg, errCode);

            return I_OK == errCode;
        }        
        
        void AbortSession() {
            ROS_INFO("**** GOBI **** Aborting session.");
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
                ROS_INFO("**** GOBI **** Could not start capturing, errorCode: %lu", errorCode);
            }
            else if (XC_IsCapturing(this->handle)) // When the camera is capturing ...
            {    
            
                // Load the color profile delivered with this sample.
                if (errorCode = XC_LoadColourProfile(handle, "/home/user1/catkin_ws/src/usma_bhg/resources/ThermalBlue.png") != I_OK)
                {
                    ROS_INFO("**** GOBI **** Problem while loading the desired colorprofile, errorCode: %lu", errorCode);
                }
                else
                {
                    ROS_INFO("**** GOBI **** Successfully loaded the desired colorprofile.");
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
            if ((errorCode = XC_GetFrame(this->handle, FT_32_BPP_RGBA, XGF_Blocking, this->frameBuffer, this->frameSize * 4)) != I_OK)
            {
                if (errorCode == 10008){
                    ROS_INFO("**** GOBI **** Retrieve frame timed out waiting for frame (possibly not triggered), errorCode %lu", errorCode);                
                }
                else{
                    ROS_INFO("**** GOBI **** Problem while fetching frame, errorCode %lu", errorCode);
                }
            }
            //errorCode = XC_SaveData(this->handle, "output.xpng", XSD_SaveThermalInfo | XSD_Force16);
            return int(errorCode);
        }
        
        void publish_frame(){
            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
            cv::Mat cv_image(cv::Size(640, 480), CV_8UC4, this->frameBuffer);
            cv_ptr->encoding = "rgba8";
            cv_ptr->header.stamp =  ros::Time::now();
            cv_ptr->header.frame_id = "gobi";
            cv_ptr->image = cv_image;
            this->image_pub_.publish(cv_ptr->toImageMsg());            
        }
        
        void clean_up(){
        // NOTE: This function must use printf since ROS is shutdown it is run.
            printf("**** GOBI **** Conducting ROS clean up\n");
            ErrCode errorCode = 0; 
            // When the camera is still capturing, ...
            if(XC_IsCapturing(this->handle))
            {
                // ... stop capturing.
                printf("**** GOBI **** Stop capturing.\n");
                if ((errorCode = XC_StopCapture(this->handle)) != I_OK)
                {
                    printf("**** GOBI **** Could not stop capturing, errorCode: %lu\n", errorCode);
                }
            }

            // When the handle to the camera is still initialised ...
            if (XC_IsInitialised(this->handle))
            {
                printf("**** GOBI **** Closing connection to camera.\n");
                XC_CloseCamera(this->handle);
            }

            printf("**** GOBI **** Clearing buffers.\n");
            if (this->frameBuffer != 0)
            {
                delete [] this->frameBuffer;
                this->frameBuffer = 0;
            }       
        }
//        
//        string make_data_directory(){
//            // Get home directory
//            const char *tmpdir;
//            std::string homedir;
//            // check the $HOME environment variable, and if that does not exist, use getpwuid
//            if ((tmpdir = getenv("HOME")) == NULL) {
//                tmpdir = getpwuid(getuid())->pw_dir;
//            }
//            homedir.assign(tmpdir);
//        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gobi_trigger");
    ros::NodeHandle nh;
    int use_trig; 
    if (ros::param::has("/gobi_trigger/use_trig")) {
        
        ros::param::get("/gobi_trigger/use_trig", use_trig);
    }
    else{
        use_trig = 2;
    }
    ROS_INFO("Use trigger mode is set to: %i",use_trig);  
    GobiBHG gobi_cam(&nh); // instantiate gobi class
    gobi_cam.retrieve_info();
    gobi_cam.initialize_cam(use_trig);
    gobi_cam.start_capture();

    uint64_t n=0;
    ros::Rate loop_rate(20); //TODO how fast should this be?    
    while (ros::ok())
    {
        if(gobi_cam.retrieve_frame() ==0){        
            n++;
            ROS_INFO("**** GOBI **** Received frame %lu", n);
            gobi_cam.publish_frame();
        }
    }
    ROS_INFO("**** GOBI **** Received ROS shutdown command");
    

    
    return 0;
}
