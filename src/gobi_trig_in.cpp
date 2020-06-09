// Header inclusions /////////////////////////////////////////////////////

#include <stdlib.h>     // C Standard Common Library
#include <stdio.h>      // C Standard Input/Output library.

#include "XCamera.h"    // Xeneth SDK main header.
// DML VVV

#include "ros/ros.h"
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


using namespace cv;
// DML ^^^
// Utility ///////////////////////////////////////////////////////////////
#include <unistd.h>
#define CRLF    "\r\n"
#define Sleep(t) usleep((t) * 1000)



// Function prototypes ///////////////////////////////////////////////////

bool HandleError(ErrCode errCode, const char * msg);
void CleanupSession();
void AbortSession();

bool SetupImageFormatAndAcquisition(XCHANDLE handle);

bool SetupExternalTriggeredMode_F027(XCHANDLE handle);
bool SetupShutterControl_F027(XCHANDLE handle);
bool ExecuteCalibration_F027(XCHANDLE handle);
void WaitForCalibration_F027(XCHANDLE handle);

// Global variables //////////////////////////////////////////////////////

/*
 *  Set the URL of the camera you want to connect to.
 *  By using "cam://0" as the URL a connection will be established
 *  to the first discovered camera. It is also possible to make a
 *  direct connection to a GigE Vision camera when the IP is known.
 *  The format of the URL for a GigE Vision camera will be "gev://<ip-address>"
 *  Make sure to check out the "Enumerate devices" to learn how device
 *  discovery is performed.
 */

const char * url = "cam://0";


/*
 *  Handle to the XCamera-object
 */

XCHANDLE handle = 0;


/*
 *  Variables to hold the frame size and buffer
 */

//unsigned int framesize = 0;
//unsigned short * framebuffer = 0;

dword *framebuffer = 0; // 16-bit buffer to store the capture frame.
dword framesize = 0; // The size in bytes of the raw image.

// Entry point ///////////////////////////////////////////////////////////

int main(int argc, char **argv) {  
    ros::init(argc, argv, "gobi_trigger");
    ros::NodeHandle n;

    image_transport::ImageTransport it_(n);
    image_transport::Publisher image_pub_ = it_.advertise("gobi_image", 1);
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
      
    ErrCode errCode = I_OK;
    long pid = 0;
    long ser = 0;

    printf("Open connection to '%s'" CRLF, url);
    handle = XC_OpenCamera(url);

    if (XC_IsInitialised(handle)) {     
        printf("Successfully established a connection to '%s'." CRLF, url);
        
        /* retrieve camera product id (PID) */
        errCode = XC_GetPropertyValueL(handle, "_CAM_PID", &pid);
        if (!HandleError(errCode, "Retrieving the camera PID")) AbortSession(); 
        errCode = XC_GetPropertyValueL(handle, "_CAM_SER", &ser);
        if (!HandleError(errCode, "Retrieving the camera serial number")) AbortSession();        
        printf("Connected to camera with product id (PID) 0x%ld and serial number %lu" CRLF CRLF, pid, ser);

        /* Check for the Gobi-640-GigE (F027) */
        if (pid == 0xF027) {

            /* configure camera in external triggered mode */
//            if (!SetupExternalTriggeredMode_F027(handle)) AbortSession();

            /* configure camera to disable the automatic shutter calibration process */
            if (!SetupShutterControl_F027(handle)) AbortSession();

        }
        else {
            printf("Connected to an unsupported camera." CRLF);
            AbortSession();
        }
        ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.

        /* Setup image format and acquisition parameters */
        if (!SetupImageFormatAndAcquisition(handle)) AbortSession();
        

        if (errorCode = XC_LoadColourProfile(handle, "/home/user1/catkin_ws/src/usma_bhg/resources/ThermalBlue.png") != I_OK)
        {
            ROS_ERROR("*** Gobi ***: Problem while loading the desired colorprofile, errorCode: %lu", errorCode);
        }
        else
        {
            ROS_INFO("*** Gobi ***: Successfully loaded the desired colorprofile.");
        }
        
        // Set the colourmode so that the last loaded colorprofile is used.
        XC_SetColourMode(handle, ColourMode_Profile);
        
        /* assign frame buffer */
        //framesize = XC_GetFrameSize(handle);
        //framebuffer = (unsigned short *)malloc(framesize);
        framesize = XC_GetWidth(handle) * XC_GetHeight(handle); // currently = 307200 which is 640x480
        framebuffer = new dword[framesize];

        if (0 == framebuffer) {
            printf("Could not allocate frame buffer memory." CRLF);
            AbortSession();
        }

        /* start capture */
        XC_StartCapture(handle);
        WaitForCalibration_F027(handle);


        // DML get props to check
        errCode = XC_SetPropertyValueL(handle, "AutoModeUpdate", 0, "");
        if (!HandleError(errCode, " * Set auto mode"))
            return false;
        errCode = XC_SetPropertyValueL(handle, "TriggerDirection", 1, "");
        if (!HandleError(errCode, " * Set trigger direction"))
            return false;
        errCode = XC_SetPropertyValueL(handle, "TriggerInMode", 0, "");
        if (!HandleError(errCode, " * Set trigger input mode")) 
            return false;            
        errCode = XC_SetPropertyValueL(handle, "TriggerInEnable", 1, "");
        if (!HandleError(errCode, " * Enable trigger input"))
            return false;            
        errCode = XC_SetPropertyValueL(handle, "TriggerInSensitivity", 0, "");
        if (!HandleError(errCode, " * Set trigger input sensitivity"))
            return false;            
        errCode = XC_SetPropertyValueL(handle, "TriggerInPolarity", 1, "");
        if (!HandleError(errCode, " * Set trigger input polarity"))
            return false;                    
        errCode = XC_SetPropertyValueL(handle, "TriggerOutEnable", 0, "");
        if (!HandleError(errCode, " * Disable trigger output"))
            return false;            
        errCode = XC_SetPropertyValueL(handle, "TriggerOutPolarity", 0, "");
        if (!HandleError(errCode, " * Set TriggerOutPolarity")) 
            return false;
        errCode = XC_SetPropertyValueL(handle, "TriggerOutWidth", 30, "");
        if (!HandleError(errCode, " * Set TriggerOutWidth  ")) 
            return false;

        long automode = 0;
        errorCode = XC_GetPropertyValueL(handle, "AutoModeUpdate", &automode);
        printf("AutoModeUpdate '%lu' " CRLF, automode);             
        long trigdir = 0; 
        errorCode = XC_GetPropertyValueL(handle, "TriggerDirection", &trigdir);
        printf("TriggerDirection is '%lu' Values: Trigger input(0), Trigger output(1)" CRLF, trigdir);
        long trigimode = 0;
        errorCode = XC_GetPropertyValueL(handle, "TriggerInMode", &trigimode);
        printf("TriggerInMode is '%lu' Values:Free running(0), Triggered(1)" CRLF, trigimode);        
        long trigienable = 0;
        errorCode = XC_GetPropertyValueL(handle, "TriggerInEnable", &trigienable);
        printf("TriggerInEnable is '%lu' Values: Off(0), On(1)" CRLF, trigienable);        
        long trigisens = 0;
        errorCode = XC_GetPropertyValueL(handle, "TriggerInSensitivity", &trigisens);
        printf("TriggerInSensitivity is '%lu' Values: Level(0), Edge(1)" CRLF, trigisens);
        long trigipol = 0;
        errorCode = XC_GetPropertyValueL(handle, "TriggerInPolarity", &trigipol);
        printf("TriggerInPolarity is '%lu' Values: Level low/Falling edge(0), Level high/Rising edge(1)" CRLF, trigipol);
        long trigoenable = 0;
        errorCode = XC_GetPropertyValueL(handle, "TriggerOutEnable", &trigoenable);
        printf("TriggerOutEnable is '%lu' Values: Off(0), On(1)" CRLF, trigoenable);
        long trigopolarity = 0;
        errorCode = XC_GetPropertyValueL(handle, "TriggerOutPolarity", &trigopolarity);
        printf("TriggerOutPolarity is '%lu' Values: Active low (0), Active high (1)" CRLF, trigopolarity);
        long trigowdith = 0;
        errorCode = XC_GetPropertyValueL(handle, "TriggerOutWidth", &trigowdith);
        printf("TriggerOutWidth is '%lu' Values: Time in microseconds" CRLF, trigowdith);
        printf(CRLF);

        // DML ^^^^^



        /* main loop */
    ros::Rate loop_rate(20); //TODO how fast should this be?

    uint64_t n=0;
    while (ros::ok())
    {
        /* 
         *  Grab a frame from the internal buffers if one is available.
         *  If no frames were captured yet, or we are polling faster than 
         *  frames are arriving, GetFrame will return E_NO_FRAME.
         */

        //ErrCode errCode = XC_GetFrame(handle, FT_NATIVE, 0, framebuffer, framesize);
        ErrCode errCode = XC_GetFrame(handle, FT_32_BPP_RGBA, XGF_Blocking, framebuffer, framesize * 4 /* bytes per pixel */);

        if (I_OK == errCode) { 
            n++;
            printf("Received frame %lu \r", n);
            fflush(stdout);
            cv::Mat cv_image(cv::Size(640, 480), CV_8UC4, framebuffer);
            cv_ptr->encoding = "rgba8";
            cv_ptr->header.stamp =  ros::Time::now();
            cv_ptr->header.frame_id = "/gobi";
            cv_ptr->image = cv_image;
            image_pub_.publish(cv_ptr->toImageMsg());
        }
        else if (E_NO_FRAME != errCode) {
            printf(CRLF);
            HandleError(errCode, "while grabbing frame");
            AbortSession();
        }
        
        if (n % 200 == 0){
            printf("Running Calibration again \r");
            ExecuteCalibration_F027(handle);
            printf("Done with Calibration again \r");
           }
        
        ros::spinOnce();
        loop_rate.sleep();
    } 

//            /* Ask the user to perform another trigger burst cycle */
//            printf(CRLF "Enter 'y' to perform another burst: ");
//            char ch = (char)fgetc(stdin);
//            for (int c = getc(stdin); c != '\n'; c = getc(stdin));

//            /* any other answer except for 'y' breaks the main loop */
//            if (ch == 'y') ExecuteCalibration_F027(handle);
//            else break;
//        }

//        printf(CRLF);
        
        /* clean-up */
        printf("Closing session" CRLF);        
        CleanupSession();

    }
    else {
        printf("Initialization failed" CRLF);
    }

    return 0;
}


// Function implementations //////////////////////////////////////////////

/*
 *  Utility function to handle error messages 
 */

bool HandleError(ErrCode errCode, const char * msg) {
    const int sz = 2048;
    char err_msg[sz];

    XC_ErrorToString(errCode, err_msg, sz);
    printf("%s: %s (%lu)" CRLF, msg, err_msg, errCode);

    return I_OK == errCode;
}


/*
 *  In SetupExternalTriggeredMode_F027 we configure the camera in an 
 *  external trigger mode where images will be captured on the rising edge
 *  of a square wave signal. For this we have to set the trigger 
 *  properties accordingly to reflect the working mode we are after.
 */

bool SetupExternalTriggeredMode_F027(XCHANDLE handle) {

    ErrCode errCode = I_OK;

    printf("Configuring camera in external triggered mode with rising edge activation" CRLF);
    
    /*
     *  TriggerOutEnable = Disabled (0), Enabled (1)
     * -----------------------------------------------------------------
     *  Although the TriggerDirection works as a multiplexer for the 
     *  trigger connection on the camera, it is still good practice to 
     *  make sure the trigger output block is completely disabled by setting
     *  its value to 0.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerOutEnable", 0, "");
    if (!HandleError(errCode, " * Disable trigger output"))
        return false;


    /* 
     *  TriggerDirection = TriggerInput (0), TriggerOutput (1)
     * ---------------------------------------------------------------------
     *  The trigger direction will have to be configured such that the 
     *  trigger connection on the camera is in a sinking (input) configuration.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerDirection", 0, "");
    if (!HandleError(errCode, " * Set trigger direction"))
        return false;


    /*
     *  TriggerInMode = FreeRunning (0), ExternalTriggered (1)
     * -----------------------------------------------------------------
     *  Set the TriggerInMode to ExternalTriggered to put the camera 
     *  in triggered mode. FreeRunning-mode means the camera is 
     *  continuously triggered by an internal source.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInMode", 1, "");
    if (!HandleError(errCode, " * Set trigger input mode")) 
        return false;


    /*
     *  TriggerInSensitivity = Level (0), Edge (1)
     * -----------------------------------------------------------------
     *  We are performing edge triggers.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInSensitivity", 1, "");
    if (!HandleError(errCode, " * Set trigger input sensitivity"))
        return false;


    /*
     *  TriggerInPolarity = LowFalling (0), HighRising (1)
     * -----------------------------------------------------------------
     *  And we want to capture a frame on the rising edge.
     */
    
    errCode = XC_SetPropertyValueL(handle, "TriggerInPolarity", 1, "");
    if (!HandleError(errCode, " * Set trigger input polarity"))
        return false;


    /*
     *  TriggerInDelay = 0
     * -----------------------------------------------------------------
     *  This configures the time in microseconds between the trigger 
     *  event and the moment the camera actually starts integrating a 
     *  new frame. In this use case no delay is required. 
     */
    
    errCode = XC_SetPropertyValueL(handle, "TriggerInDelay", 0, "");
    if (!HandleError(errCode, " * Set trigger input delay"))
        return false;


    /*  
     *  TriggerInSkip = 0
     * -----------------------------------------------------------------
     *  Skip n triggers before allowing the trigger to capture a frame.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInSkip", 0, "");
    if (!HandleError(errCode, " * Set trigger input skip"))
        return false;

        
    /*
     *  TriggerInTiming = Optimal (0), Custom (1)
     * -----------------------------------------------------------------
     *  In the optimal mode, the sensor is constantly running and acquiring 
     *  images preventing the *  sensor to cool down due to an irregular or 
     *  too slow external trigger rate. If an external trigger is active, 
     *  the next frame from the sensor is sent out on the interface. 
     *  The latency between a trigger pulse and the actual acquisition start
     *  is in the worst case 1 frame time. In this mode, the trigger input 
     *  delay should be considered as a minimum trigger input delay. 
     *  As soon as an external trigger pulse is detected, a timer is 
     *  started to implement the trigger input delay. When the timer expires,
     *  a trigger pulse is generated. The next frame will be sent out.
     *
     *  In the custom mode, the sensor only acquires frames when an 
     *  external trigger is active. This might degrade the image quality 
     *  if the frequency of the external triggers is not high enough or 
     *  when irregular external triggers are applied.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInTiming", 0, "");
    if (!HandleError(errCode, " * Set trigger input timing"))
        return false;


    /*
     *  TriggerInEnable = Disabled (0), Enabled (1)
     * -----------------------------------------------------------------
     *  Enable the trigger input block
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInEnable", 1, "");
    if (!HandleError(errCode, " * Enable trigger input"))
        return false;

    printf(CRLF);
    return true;
}


/*
 *  In SetupShutterControl_F027 we disable the automatic shutter correction.
 *  When this is set to enabled it is possible that triggers being received 
 *  during a calibrate cycle are not processed by the camera.
 *  To make sure the image does not drift and stays corrected the camera has
 *  to be occasionally calibrated by stopping / starting the acquisition or
 *  executing the "Calibrate"-property by setting its value to 1.
 */

bool SetupShutterControl_F027(XCHANDLE handle) {

    ErrCode errCode = I_OK;

    printf("Configuring camera to disable the automatic shutter control: " CRLF);

    /* 
     *  AutoCorrectionEnabled = Disabled (0), Enabled (1)
     * -----------------------------------------------------------------
     *  Disable the automatic internal calibration. 
     */

    errCode = XC_SetPropertyValueL(handle, "AutoCorrectionEnabled", 0, "");
    if (!HandleError(errCode, " * Disable auto correction"))
        return false;
    
    printf(CRLF);
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

bool ExecuteCalibration_F027(XCHANDLE handle) {

    ErrCode errCode = I_OK;
    
    errCode = XC_SetPropertyValueL(handle, "Calibrate", 1, "");
    if (!HandleError(errCode, "Perform calibration"))
        return false;

    WaitForCalibration_F027(handle);

    return true;
}
    
void WaitForCalibration_F027(XCHANDLE handle) {
    Sleep(2500);
}


/*
 *  Setup image format and acquisition parameters
 */

bool SetupImageFormatAndAcquisition(XCHANDLE handle) {

    ErrCode errCode = I_OK;

    /* Acquisition should be halted when the image format will change */
    boole wasCapturing = XC_IsCapturing(handle);
    XC_StopCapture(handle);

    printf("Setup image format and acquisition parameters" CRLF);
    
    /* 
     *  ExposureTimeAbs = Integer (1 ... 80)
     * -----------------------------------------------------------------
     */

    errCode = XC_SetPropertyValueL(handle, "ExposureTimeAbs", 25, "");
    if (!HandleError(errCode, " * Set integration time"))
        return false;


    /* 
     *  Width = Integer (160 ... 640)
     * -----------------------------------------------------------------
     */

    errCode = XC_SetPropertyValueL(handle, "Width", XC_GetMaxWidth(handle), "");
    if (!HandleError(errCode, " * Set maximum width"))
        return false;


    /* 
     *  Height = Integer (120 ... 480)
     * -----------------------------------------------------------------
     */

    errCode = XC_SetPropertyValueL(handle, "Height", XC_GetMaxHeight(handle), "");
    if (!HandleError(errCode, " * Set maximum height"))
        return false;



    errCode = XC_SetPropertyValueL(handle, "GainControl", 1, "");
    if (!HandleError(errCode, " * Set GainControl"))
        return false;



    /* enable acquisition if it was already running when entering this function */
    if (wasCapturing) XC_StartCapture(handle);

    printf(CRLF);
    return true;
}

void CleanupSession() {
    /* cleanup frame buffer */
    if (framebuffer != 0) free(framebuffer);
    framebuffer = 0;

    /* make sure capturing has stopped */
    if (XC_IsCapturing(handle)) XC_StopCapture(handle);

    /* close the session */
    XC_CloseCamera(handle);
}

void AbortSession() {
    printf("Aborting session." CRLF);
    CleanupSession();
    exit(-1);
}
