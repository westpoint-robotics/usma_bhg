//NBL: ROS Compliance
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "stdio.h" // C Standard Input/Output library.
#include "XCamera.h" // Xeneth SDK main header.
#include <iostream>   // std::cout
#include <string>     // std::string, std::to_string
#include <chrono>
#include <ctime>

using namespace std;
using namespace std::chrono;

system_clock::time_point now;
time_t tt;
tm utc_tm;
tm local_tm;
tm last_tm;
int timestamp_modifier_i;
char timestamp_modifier_c;


//NBL: ROS Compliance
std_msgs::String record;
string recordData = "1";
bool camerasInitialized;

unsigned int imageCnt;

//NBL: ROS Compliance
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    record = *msg;
 
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/*
int AcquireImage()
{
}*/

int main(int argc, char **argv)
{
    // Variables
    XCHANDLE handle = 0; // Handle to the camera
    ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
    dword *frameBuffer = 0; // 16-bit buffer to store the capture frame.
    dword frameSize = 0; // The size in bytes of the raw image.

    // Open a connection to the first detected camera by using connection string cam://0
    printf("Gobi: opening connection to cam://0\n");
    handle = XC_OpenCamera("cam://0");
    printf("Gobi: handle %d\n", handle);
    //NBL: ROS Compliance
    ros::init(argc, argv, "bool_sub");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("record", 1000, chatterCallback);
    imageCnt = 0;
    ros::Rate loop_rate(10);

    //NBL: Loop until connection is initialised?
    // When the connection is initialised, ...
    while(!XC_IsInitialised(handle))
    {
        printf("Gobi: initialization failed\n");
    }
    printf("Gobi: initialization succeeded\n");

    while (ros::ok())
    {
        //Can be stopped with a Ctrl-C, but otherwise, loop forever.
        //Check record subscription.
        recordData = record.data.c_str();
        
        //NBL: record = 1      
        if(recordData == "1")
        {
            //NBL: Loop until connection is initialised?
            // When the connection is initialised, ...
            while(!XC_IsInitialised(handle))
            {
                printf("Gobi: initialization failed\n");
            }
            printf("Gobi: initialization succeeded\n");
            /*
            if(handle == 0)
            {
                handle = XC_OpenCamera("cam://0");
            }
            if(!XC_IsInitialised(handle))
            {
                printf("Initialization failed\n");
            }
            printf("Initialization succeeded\n");
            */
            // ... start capturing
            printf("Gobi: start capturing.\n");
            if ((errorCode = XC_StartCapture(handle)) != I_OK)
            {
                printf("Could not start capturing, errorCode: %lu\n", errorCode);
            }
            else if (XC_IsCapturing(handle)) // When the camera is capturing ...
            {
                // Load the color profile delivered with this sample.
                if ((errorCode = XC_LoadColourProfile(handle, "/home/user1/Documents/Software/Xeneth_2.7/Colour_profiles/Thermal_Blue.png")) != I_OK)
                {
                   printf("Problem while loading the desired colorprofile, errorCode: %lu\n", errorCode);
                }
                else
                {
                    printf("handle = %d... color profile errorCode = %lu\n", handle, errorCode);
                }

                // Set the colourmode so that the last loaded colorprofile is used.
                XC_SetColourMode(handle, ColourMode_Profile);
                
                // Determine framesize for a 32-bit buffer.
                frameSize = XC_GetWidth(handle) * XC_GetHeight(handle);

                // Initialize the 32-bit buffer.
                frameBuffer = new dword[frameSize];
//                // Determine native framesize.
//                frameSize = XC_GetFrameSize(handle);

//                // Initialize the 16-bit buffer.
//                frameBuffer = new word[frameSize / 2];
                
                // ... grab a frame from the camera.
                printf("Grabbing a frame - FT_32_BPP_RGBA.\n");
                if ((errorCode = XC_GetFrame(handle, FT_32_BPP_RGBA, XGF_Blocking, frameBuffer, frameSize * 4 /* bytes per pixel */)) != I_OK)
                {
                   printf("Gobi: problem while fetching frame, errorCode %lu\n", errorCode);
                }
                else
                {
                    dword pixel = frameBuffer[0];
                    byte b = (pixel >> 0)  & 0xff;
                    byte g = (pixel >> 8)  & 0xff;
                    byte r = (pixel >> 16) & 0xff;
                    byte a = (pixel >> 24) & 0xff;

                    printf("Pixel value: r = %u, g = %u, b = %u\n", r, g, b);
                
                    //NBL: Grab time and date information for creating a timestamp for the image filenames.        
                    now = system_clock::now();
                    tt = system_clock::to_time_t(now);
                    utc_tm = *gmtime(&tt);
                    local_tm = *localtime(&tt);
                    string dateTime = to_string(utc_tm.tm_year + 1900) + '-' + to_string(utc_tm.tm_mon + 1) + '-' + to_string(utc_tm.tm_mday) + '_' 
                                    + to_string(utc_tm.tm_hour) + '-' + to_string(utc_tm.tm_min) + '-' + to_string(utc_tm.tm_sec);
                 
                    if((local_tm.tm_hour == last_tm.tm_hour) &&
                       (local_tm.tm_min == last_tm.tm_min) &&
                       (local_tm.tm_sec == last_tm.tm_sec))
                    {
                        timestamp_modifier_c = static_cast<char>(timestamp_modifier_i);
                        dateTime += timestamp_modifier_c;
                        timestamp_modifier_i ++;
                    }
                    else
                    {
                        timestamp_modifier_i = 97;
                    }
                    last_tm = local_tm;
                    
                    string imageDirectory = "/home/user1/Data/"; 
                    string imageFilename = imageDirectory + "Xenic-Gobi" + "_" + dateTime + ".xpng";
                    
                    if((errorCode = XC_SaveData(handle, imageFilename.c_str(), XSD_SaveThermalInfo | XSD_RFU_1)) != I_OK)
                    {
                        printf("Gobi: problem saving data, errorCode %lu\n", errorCode);
                    }else{
                        printf("Gobi: saved successfully!\n");
                    }
                }
            }        
        }//NBL: /record = 1
        //NBL: record = 0
        else
        {

            printf("Clearing buffers.\n");
            if (frameBuffer != 0)
            {
                delete [] frameBuffer;
                frameBuffer = 0;
            }
            //NBL: /record = 0
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // Cleanup.

    // When the camera is still capturing, ...
    if(XC_IsCapturing(handle))
    {
        // ... stop capturing.
        printf("Stop capturing.\n");
        if ((errorCode = XC_StopCapture(handle)) != I_OK)
        {
            printf("Could not stop capturing, errorCode: %lu\n", errorCode);
        }
    }

    // When the handle to the camera is still initialised ...
    if (XC_IsInitialised(handle))
    {
        printf("Gobi: closing connection to camera.\n");
        XC_CloseCamera(handle);
    }
    return 0;
}
