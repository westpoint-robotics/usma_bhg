//NBL: ROS Compliance
#include "ros/ros.h"
//NBL: MAVROS libraries
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
//NBL: ROS Pub/Sub messages
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include <sys/stat.h> // mkdir command
#include "stdio.h"    // C Standard Input/Output library.
#include "XCamera.h"  // Xeneth SDK main header.
#include <iostream>   // std::cout
#include <fstream>    // CSV file manipulation
#include <locale>
#include <string>     // std::string, std::to_string
#include <chrono>
#include <ctime>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace std::chrono;

double ros_now;
//NBL: ROS Compliance
std_msgs::Bool record;
string img_dir;
ros::Time rosTimeSinceEpoch;
std::time_t raw_time;    
tm local_tm;

char dateTime [50];
int ros_millisec;
//int n;
string imageDirectory; 
string imageFilename;
string cvFilename;
//string recordData = "1";
bool camerasInitialized;

unsigned int imageCount;

/*** GOBI CSV CODE ***/
string csvFilename = "";
std::ofstream csvOutfile;

sensor_msgs::MagneticField mag_data;
sensor_msgs::Imu imu_data;

/*
//timestamp_data = "";
//is_recording = False;
rel_alt = Altitude();
gps_fix = NavSatFix();
imu_mag = MagneticField();
imu_data = Imu();
vel_gps = TwistStamped();
temp_imu = Temperature();
*/
string char_array_to_string(char* char_array)
{
    string my_string(char_array);

    return my_string;
}

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
    string alt_str; 
    string gps_str; 
    string mag_str;    
    string imu_str;    
    string vel_str;  
    string temp_str;
    string output;
    /*
    alt_str = imageFilename + "," + str(rel_alt.monotonic) + "," + str(rel_alt.amsl) + "," + str(rel_alt.local) + "," + str(rel_alt.relative); 
    gps_str = str(gps_fix.status.status) + "," + str(gps_fix.status.service) + "," + str(gps_fix.latitude) + "," + str(gps_fix.longitude) + "," + str(gps_fix.altitude); 
    mag_str = str(imu_mag.magnetic_field.x) + "," + str(imu_mag.magnetic_field.y) + "," + str(imu_mag.magnetic_field.z);
    */
    alt_str = imageFilename + "," + char_array_to_string(dateTime) + ",0,0,0,0"; 
    gps_str = "0,0,0,0,0"; 
    mag_str = to_string(mag_data.magnetic_field.x) + "," + to_string(mag_data.magnetic_field.y) + "," + to_string(mag_data.magnetic_field.z);    
    imu_str = to_string(imu_data.orientation.x) + "," + to_string(imu_data.orientation.y) + "," + to_string(imu_data.orientation.z) + "," + to_string(imu_data.orientation.w) + ",";
    imu_str += to_string(imu_data.angular_velocity.x) + "," + to_string(imu_data.angular_velocity.y) + "," + to_string(imu_data.angular_velocity.z) + ",";
    imu_str += to_string(imu_data.linear_acceleration.x) + "," + to_string(imu_data.linear_acceleration.y) + "," + to_string(imu_data.linear_acceleration.z);    
    vel_str = "0,0,0,";
    vel_str += "0,0,0";  
    temp_str = "0";
    output = alt_str + "," + gps_str + "," + mag_str + "," + imu_str + "," + vel_str + "," + temp_str;
    return output;
}
/*
void alt_cb(msg)
{
    rel_alt = msg;
}        

void gps_cb(msg)
{
    gps_fix = msg;
}    

void mag_cb(msg)
{
    imu_mag = msg;
}    

void imu_cb(msg)
{
    imu_data = msg;
}    

void vel_cb(msg)
{
    vel_gps = msg;
}

void temp_cb(msg)
{
    temp_imu = msg;  
}
*/
/*** GOBI CSV CODE ***/

//NBL: ROS Compliance
void mag_cb(const sensor_msgs::MagneticField::ConstPtr& msg){
     /*
     ROS_INFO("*** Gobi ***:\nlinear acceleration\
                 \nx: [%f]\ny:[%f]\nz:[%f]", msg->linear_acceleration.x,
                 msg->linear_acceleration.y, msg->linear_acceleration.z);
    */
    mag_data = *msg;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
     /*
    ROS_INFO("*** Gobi ***:\nlinear acceleration\
                 \nx: [%f]\ny:[%f]\nz:[%f]", msg->linear_acceleration.x,
                 msg->linear_acceleration.y, msg->linear_acceleration.z);
    */
    imu_data = *msg;
    //imu_data = msg->data.c_str();
}

void recordCallback(const std_msgs::Bool::ConstPtr& msg)
{
    record = *msg;
 
    //ROS_INFO("*** Gobi *** recordCallback: [%s]", record);
}

void dirCallback(const std_msgs::String::ConstPtr& msg)
{
    img_dir       = msg->data.c_str(); 
    string imgtmp = "/home/user1/Data/" + img_dir + "/GOBI000088/";
    csvFilename   = "/home/user1/Data/" + img_dir + "/" + img_dir + "_gobi.csv";
    
    //ROS_INFO("******* Image directory is : [%s] *******", imgtmp.c_str());
    if (mkdir(imgtmp.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
    {
        if( errno == 0 ) { // TODO fix this to return accurate feedback
            // does not exists
              
        }
    }    
    else
    {
        csvOutfile.open(csvFilename, std::ios_base::app); // append instead of overwrite 
        csvOutfile << make_header() << endl;   
        ROS_INFO("*** Gobi ***: image directory created %i [%s]; along with CSV file %s\n", errno, img_dir.c_str(), csvFilename.c_str());
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
    ros::Subscriber record_sub = n.subscribe("record", 1000, recordCallback);
    ros::Subscriber dir_sub = n.subscribe("directory", 1000, dirCallback);
    ros::Subscriber mag_sub = n.subscribe("/mavros/mag/data", 1000, mag_cb);
    ros::Subscriber imu_sub = n.subscribe("/mavros/imu/data", 1000, imu_cb);

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

        /*        
        // Load the color profile delivered with this sample.
        if (errorCode = XC_LoadColourProfile(handle, "/home/user1/catkin_ws/src/usma_bhg/resources/ThermalBlue.png") != I_OK)
        {
            ROS_ERROR("*** Gobi ***: Problem while loading the desired colorprofile, errorCode: %lu", errorCode);
        }
        else
        {
            ROS_INFO("*** Gobi ***: Successfully loaded the desired colorprofile.");
        }
        */
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
                //frameBuffer = new dword[frameSize];

                //Before taking a picture, grab timestamp to record to filename                 
                rosTimeSinceEpoch = ros::Time::now();
                raw_time = static_cast<time_t>(rosTimeSinceEpoch.toSec());    
                local_tm = *localtime(&raw_time);

                ros_millisec = int((rosTimeSinceEpoch.nsec)/1000000);
                int n=sprintf (dateTime, "%d%02d%02d_%02d%02d%02d_%03d", local_tm.tm_year + 1900, local_tm.tm_mon + 1, local_tm.tm_mday, local_tm.tm_hour, local_tm.tm_min, local_tm.tm_sec, ros_millisec);

                imageDirectory = "/home/user1/Data/"+img_dir+ "/GOBI000088/"; 
                imageFilename =  "/home/user1/Data/"+img_dir+ "/GOBI000088/GOBI000088" + "_" + dateTime + ".png";
                cvFilename =  "/home/user1/Data/"+img_dir+ "/GOBI000088/GOBI000088" + "_" + dateTime + ".jpg";

                // ... grab a frame from the camera.
                //if ((errorCode = XC_GetFrame(handle, FT_16_BPP_GRAY, XGF_Blocking, frameBuffer, frameSize * 2 /* bytes per pixel */)) != I_OK)
                //if ((errorCode = XC_GetFrame(handle, FT_32_BPP_GRAY, XGF_Blocking, frameBuffer, frameSize * 4 /* bytes per pixel */)) != I_OK)
                if ((errorCode = XC_GetFrame(handle, FT_32_BPP_RGBA, XGF_Blocking, frameBuffer, frameSize * 4 /* bytes per pixel */)) != I_OK)
                {
                    ROS_ERROR("*** Gobi ***: problem while fetching frame, errorCode %lu", errorCode);
                }
                else
                {         
                    // TODO IF successful grab do a deep copy and write it to disk.
                    //cv::Mat cv_image(cv::Size(640, 480), CV_16UC1, frameBuffer);
                    //cv::Mat cv_image(cv::Size(640, 480), CV_32FC1, frameBuffer);
                    //cv::imwrite( cvFilename, cv_image );
                    //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
                    //cv::imshow( "Display window", cv_image );                   // Show our image inside it.

                    //cv::waitKey(0);


                    //ROS_INFO("=*=*=*=* Image directory is : [%s] =*=*=*=*", imageFilename.c_str());                    
                    if((errorCode = XC_SaveData(handle, imageFilename.c_str(), XSD_SaveThermalInfo | XSD_RFU_1)) != I_OK)
                    {
                        ROS_ERROR("*** Gobi ***: problem saving data, errorCode %lu\n", errorCode);
                    }else
                    {
                        imageCount += 1;
                        //NBL When you save your image, update your CSV file
                        /***  CSV FILE UPDATE ***/
                        csvOutfile << make_logentry() << endl;

                        
                        //csvOutfile << "Data";
                        /***  CSV FILE UPDATE ***/
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
