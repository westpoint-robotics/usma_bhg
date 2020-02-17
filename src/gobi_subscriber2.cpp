//NBL: ROS Compliance
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <sstream>


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
    
    ros::Publisher time_pub = n.advertise<std_msgs::String>("gobi_time", 1000);
    
    //Initialize ros time ONCE
    ros::Time::init();
    ros::Rate loop_rate(10); //TODO how fast should this be?
    while (ros::ok())
    {
        //Can be stopped with a Ctrl-C, but otherwise, loop forever.
        
        //NBL: record = true      
        if(record.data)
        {
                //Before taking a picture, grab timestamp to record to filename                 
                ros::Time rosTimeSinceEpoch = ros::Time::now();
                std::time_t raw_time = static_cast<time_t>(rosTimeSinceEpoch.toSec());    
                tm local_tm = *localtime(&raw_time);

                char dateTime [50];
                int ros_millisec = int((rosTimeSinceEpoch.nsec)/1000000);
                int n=sprintf (dateTime, "%d%02d%02d_%02d%02d%02d_%03d", local_tm.tm_year + 1900, local_tm.tm_mon + 1, local_tm.tm_mday, local_tm.tm_hour, local_tm.tm_min, local_tm.tm_sec, ros_millisec);

                string imageDirectory = "/home/user1/Data/"+img_dir+ "/GOBI000088/"; 
                string imageFilename =  "/home/user1/Data/"+img_dir+ "/GOBI000088/GOBI000088" + "_" + dateTime + ".xpng";

                std_msgs::String msg;
                std::stringstream ss;

                ss << imageFilename;
                msg.data = ss.str();
                ROS_INFO("%s", msg.data.c_str());
                time_pub.publish(msg);
                
        }//NBL: /record = 1
        //NBL: record = 0
        else
        {


        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
