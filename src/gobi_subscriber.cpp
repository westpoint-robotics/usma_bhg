//NBL: ROS Compliance
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include <sys/stat.h> // mkdir command
#include "stdio.h" // C Standard Input/Output library.
#include "XCamera.h" // Xeneth SDK main header.
#include <iostream>   // std::cout
#include <locale>
#include <string>     // std::string, std::to_string
#include <chrono>
#include <ctime>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

using namespace std;
using namespace std::chrono;

double ros_now;
//NBL: ROS Compliance
std_msgs::Bool record;
string img_dir;
//string recordData = "1";
bool camerasInitialized;

unsigned int imageCount;

//NBL: ROS Compliance
void chatterCallback(const std_msgs::Bool::ConstPtr& msg)
{
    record = *msg;
 
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void dirCallback(const std_msgs::String::ConstPtr& msg)
{
    img_dir = msg->data.c_str(); 
    string imgtmp = "/home/user1/Data/"+img_dir+ "/GOBI_000088/";
    //ROS_INFO("******* Image directory is : [%s] *******", imgtmp.c_str());
    if (mkdir(imgtmp.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
    {
        if( errno == 0 ) { // TODO fix this to return accurate feedback
            // does not exists
            ROS_INFO("******* Image directory created: %i [%s] *******", errno, img_dir.c_str());  
        }
    }
}

/*
int AcquireImage()
{
}*/

int main(int argc, char **argv)
{
    //NBL: ROS Compliance{
    ros::init(argc, argv, "gobi_capture");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("record", 1000, chatterCallback);
    ros::Subscriber dir_sub = n.subscribe("directory", 1000, dirCallback);

    // Variables
    XCHANDLE handle = 0; // Handle to the camera
    ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
    dword *frameBuffer = 0; // 16-bit buffer to store the capture frame.
    dword frameSize = 0; // The size in bytes of the raw image.
    imageCount = 0;

    ros_now = ros::WallTime::now().toSec() * 1e-6;
    ROS_INFO("\n\nROS NOW = %.1f\n\n", ros_now);

    // Open a connection to the first detected camera by using connection string cam://0
    ROS_INFO("*** Gobi ***: opening connection to cam://0");
    handle = XC_OpenCamera("cam://0");
    if(XC_IsInitialised(handle))
    {    
      // ... start capturing
      ROS_INFO("*** Gobi ***: Is initialized.");
      if (errorCode = XC_StartCapture(handle) != I_OK)
      {
        ROS_ERROR("*** Gobi ***: Could not start capturing, errorCode: %lu\n", errorCode);
        exit(1);
      }
      else if (XC_IsCapturing(handle)) // When the camera is capturing ...
      {
        ROS_INFO("*** Gobi ***: Successfully capturing.");
        // Load the color profile delivered with this sample.
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

        // Determine framesize for a 32-bit buffer.
        frameSize = XC_GetWidth(handle) * XC_GetHeight(handle);

        // Initialize the 32-bit buffer.
        frameBuffer = new dword[frameSize];
        }
    }
    else
    {
        ROS_ERROR("*** Gobi ***: Failed to initialize. EXITING NOW!");
        exit(1);   
    }
    //Initialize ros time ONCE
    ros::Time::init();
    ros::Rate loop_rate(20); //TODO how fast should this be?
    while (ros::ok())
    {
        //Can be stopped with a Ctrl-C, but otherwise, loop forever.
        
        //NBL: record = true      
        if(record.data)
        {
                // Initialize the 32-bit buffer.
                frameBuffer = new dword[frameSize];

                //Before taking a picture, grab timestamp to record to filename                 
                time_t tt;
                tm local_tm;
                tm last_tm;

                ros::Time rosTimeSinceEpoch = ros::Time::now();
                std::time_t raw_time = static_cast<time_t>(rosTimeSinceEpoch.toSec());    
                local_tm = *localtime(&raw_time);

                char dateTime [50];
                int n;
                n=sprintf (dateTime, "%d%02d%02d_%02d%02d%02d_%03d", local_tm.tm_year + 1900, local_tm.tm_mon + 1, local_tm.tm_mday, local_tm.tm_hour, local_tm.tm_min, local_tm.tm_sec, rosTimeSinceEpoch.nsec);

                string imageDirectory = "/home/user1/Data/"+img_dir+ "/GOBI_000088/"; 
                string imageFilename =  "/home/user1/Data/"+img_dir+ "/GOBI_000088/Xenic-Gobi" + "_" + dateTime + ".xpng";

                // ... grab a frame from the camera.
                if ((errorCode = XC_GetFrame(handle, FT_32_BPP_RGBA, XGF_Blocking, frameBuffer, frameSize * 4 /* bytes per pixel */)) != I_OK)
                {
                    ROS_ERROR("*** Gobi ***: problem while fetching frame, errorCode %lu", errorCode);
                }
                else
                {         
                    // TODO IF successful grab do a deep copy and write it to disk.
                    

                    //ROS_INFO("=*=*=*=* Image directory is : [%s] =*=*=*=*", imageFilename.c_str());                    
                    if((errorCode = XC_SaveData(handle, imageFilename.c_str(), XSD_SaveThermalInfo | XSD_RFU_1)) != I_OK)
                    {
                        ROS_ERROR("*** Gobi ***: problem saving data, errorCode %lu\n", errorCode);
                    }else{
                        imageCount += 1;
                        ROS_INFO("*** Gobi ***: %s | %d\n", dateTime, imageCount);
                        //ROS_INFO("*** Gobi ***: saved successfully!\n");
                    }
                }
           
        }//NBL: /record = 1
        //NBL: record = 0
        else
        {

            //printf("Clearing buffers.\n");
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
