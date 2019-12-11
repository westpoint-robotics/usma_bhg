#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

//http://wiki.ros.org/spinnaker_sdk_camera_driver
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <ctime>


using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace std::chrono;

std_msgs::String record;
string recordData = "";
bool camerasInitialized;
bool acquisitionModeSet;

system_clock::time_point now;
time_t tt;
tm utc_tm;
tm local_tm;
tm last_tm;
int timestamp_modifier_i;
char timestamp_modifier_c;

//string deviceSerialNumber [] = {};
string deviceModelNameS = "";
CameraList camList;
unsigned int imageCnt;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    record = *msg;
 
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int SetAcquisitionMode(CameraPtr pCam, INodeMap & nodeMap){
    int result = 0;
    try
    {
        // Set acquisition mode to continuous
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            ROS_INFO("Unable to set acquisition mode to continuous (node retrieval). Aborting...\n\n");
            return -1;
        }

        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            ROS_INFO("Unable to set acquisition mode to continuous (entry 'continuous' retrieval). Aborting...\n\n");
            return -1;
        }

        int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        //ROS_INFO("Acquisition mode set to continuous...\n");
    }
    catch (Spinnaker::Exception &e)
    {
        ROS_INFO("Error: %s\n", e.what());
        result = -1;
    }
    return result;
}

int AcquireImage(CameraPtr pCam, INodeMap & nodeMap, int cameraIndx)
{
    int result = 0;
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
    
    //Grab its Serial Number
    CStringPtr deviceModelName = nodeMap.GetNode("DeviceModelName");
    deviceModelNameS = deviceModelName->GetValue().c_str();
    //deviceSerialNumber[i] = deviceModelNameS;
    //ROS_INFO("serial number: %s", deviceModelNameS);
 
    string imageDirectory = "/home/user1/Data/"; 
    string imageFilename = imageDirectory + deviceModelNameS + "_" + dateTime;

    try
    {
        pCam->BeginAcquisition();

            
        // Retrieve and convert images
        
        imageCnt++;
        
        // Retrieve the next received image
        //ROS_INFO("Acquire image %d", imageCnt);
        ImagePtr pResultImage = pCam->GetNextImage();
            
        try
        {
            if (pResultImage->IsIncomplete())
            {
                ROS_INFO("FLIR Image incomplete with image status %d...\n\n", pResultImage->GetImageStatus());
            }
            else
            {
                //cout << "Grabbed image " << imageCnt << ", width = " << pResultImage->GetWidth() << ", height = " << pResultImage->GetHeight() << endl;
                imageFilename = imageFilename + ".ppm";
                //cout << "FLIR grabbed image at " << __TIME__ << " to save to " << imageFilename.c_str() << endl;
                
                //ROS_INFO("Save image %d", imageCnt);
                pResultImage->Save(imageFilename.c_str(), PPM);
                //cout << "Saved image file " << imageFilename << endl;
                
                imageFilename = imageDirectory + deviceModelNameS + "_" + dateTime;

                // Deep copy image into image vector
                //images.push_back(pResultImage->Convert(PixelFormat_RGB8, HQ_LINEAR));//PixelFormat_Mono8, HQ_LINEAR));
            }
            // Release image
            pResultImage->Release();
        }
        catch (Spinnaker::Exception &e)
        {
            cout << "Error: " << e.what() << endl;
            result = -1;
        }

        //std::cout << "Done capturing.  Please come again." << endl << endl;
        
        // End acquisition
        pCam->EndAcquisition();
        
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
    
       
}

int RunCameras(CameraList &camList, unsigned int numCameras)
{
    int result;
    CameraPtr pCam;  
    
    // Run example on each camera
    //ROS_INFO("acquisitionModeSet: %d", acquisitionModeSet);
    //ROS_INFO("Initialize, or acquire images from %d cameras", numCameras);
    //Go through each camera and:
    for (unsigned int i = 0; i < numCameras; i++)
    {
        pCam = camList.GetByIndex(i);
        
   
        if(camerasInitialized == false && i < numCameras)
        {
            try
            {
                pCam->Init(); 
                //Retrieve GenICam nodemap
                INodeMap & nodeMap = pCam->GetNodeMap();
                //Set acquisition mode to continuous as needed
                if(acquisitionModeSet == false)
                {
                    SetAcquisitionMode(pCam, nodeMap);   
                }       
                //
            }
            catch (Spinnaker::Exception &e)
            {
                ROS_INFO("Error: %s", e.what());
                return -1;
            }
            
        }
   
        //ROS_INFO("camerasInitialized: %d, acquisitionModeSet: %d", camerasInitialized, acquisitionModeSet);
        if(camerasInitialized == true && acquisitionModeSet == true)
        {
            INodeMap & nodeMap = pCam->GetNodeMap();
            result = result | AcquireImage(pCam, nodeMap, i);
        }

    }
    if(camerasInitialized == false)
    {
        camerasInitialized = true;
    }
    if(acquisitionModeSet == false)
    {
        acquisitionModeSet = true;
    }
    return result;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bool_sub");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("record", 1000, chatterCallback);
    imageCnt = 0;
    ros::Rate loop_rate(10);

    //NBL: Camera initialization
    unsigned int numCameras = 1;
    camerasInitialized = false;   
    acquisitionModeSet = false;
    
    now = system_clock::now();
    tt = system_clock::to_time_t(now);
    utc_tm = *gmtime(&tt);
    local_tm = *localtime(&tt);
    last_tm = local_tm;
    timestamp_modifier_i = 97;
    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();
    
    // Retrieve list of cameras from the system
    camList = system->GetCameras();
    //ROS_INFO("numCameras = camList.GetSize();");
    numCameras = camList.GetSize();
    
    ROS_INFO("Number of cameras detected: %d", numCameras);

    // Prompt to check connections if there are no cameras
    if (numCameras == 0)
    {
        ROS_INFO("camList.Clear();");
        // Clear camera list before releasing system
        camList.Clear();
        ROS_INFO("system->ReleaseInstance();");
        // Release system
        system->ReleaseInstance();

        ROS_INFO("I am not finding any cameras connected.  Please press ENTER, and I will recheck in 5 seconds.");
        getchar();
        ros::Duration(5.0).sleep(); 
        ROS_INFO("numCameras = %d", numCameras);
    }
    
    //CameraPtr pCam;
    /*
    string deviceSNs [] = {};
    */
    //Check for cameras, and populate camList with all cameras found.
    while (ros::ok())
    {
        //Can be stopped with a Ctrl-C, but otherwise, loop forever.
        //Check record subscription.
        recordData = record.data.c_str();
        ROS_INFO("recordData: %s", recordData.c_str());
        //If record == true, our end user wants to record data from the cameras.
        if(recordData == "1")
        {
            //Grab and save an image
            //ROS_INFO("RunCameras()");
            RunCameras(camList, numCameras);
                
        }
        //If record == false, the end user does not want to record data.  Either: don't start yet, or stop recording.
        if(recordData == "0")
        {
            if(camerasInitialized == true){
                //ROS_INFO("Go through each camera and:");
                //ROS_INFO("Deinitialize/release the camera.");
                //Go through each camera and:
                for (unsigned int i = 0; i < numCameras; i++)
                {
                    
                    //Deinitialize/release the camera.
                    camList.GetByIndex(i)->DeInit();
                }
                camerasInitialized = false;
                acquisitionModeSet = false;
            }
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    //pCam = NULL;
    // Clear camera list before releasing system
    camList.Clear();
    // Release system
    system->ReleaseInstance();
    
    return 0;
}
