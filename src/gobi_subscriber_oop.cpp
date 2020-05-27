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
#include <unistd.h>    // used to get home directory
#include <sys/types.h> // used to get home directory
#include <pwd.h>       // used to get home directory

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
#include <boost/algorithm/string.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace std::chrono;

class GobiBHG {
    private:
        //NBL: ROS Compliance
        std_msgs::Bool record; // PVT State Variable 
        string imageFilename; //PVT State Variable        
        string data_dir; //PVT State Variable do this in constructor?
        char dateTime [50]; // Questionable State Variable
        unsigned int imageCount; // PVT State Variable
        std::ofstream csv_outfile; // File object for the csv file
        string csv_filename; // File name of the csv file
        std::string homedir;
        
        
        dword *frameBuffer; // 16-bit buffer to store the capture frame.
        dword frameSize; // The size in bytes of the raw image.
    
        

        // State updated by subscriber callbacks
        sensor_msgs::MagneticField mag_data;
        sensor_msgs::Imu imu_data;
        mavros_msgs::Altitude rel_alt;
        sensor_msgs::NavSatFix gps_fix;
        geometry_msgs::TwistStamped vel_gps;
        sensor_msgs::Temperature temp_imu;    
        
        ros::Subscriber record_sub;
        ros::Subscriber dir_sub;
        ros::Subscriber alt_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber vel_sub; 
        ros::Subscriber mag_sub;
        ros::Subscriber imu_sub;
        ros::Subscriber temp_sub;                

        image_transport::ImageTransport it_(n);
        image_transport::Publisher;
        
        
          
}

GobiBHG::GobiBHG(ros::NodeHandle *nh) {
    data_dir = "/temp/gobi/"; 
    
    frameBuffer = 0;
    frameSize = 0; 
    

    homedir = get_homedir();

    record_sub  = n.subscribe("/record", 1000, recordCallback);
    dir_sub     = n.subscribe("/directory", 1000, dirCallback);
    alt_sub     = n.subscribe("/mavros/altitude", 1000, alt_cb);
    gps_sub     = n.subscribe("/mavros/global_position/raw/fix", 1000, gps_cb);
    vel_sub     = n.subscribe("/mavros/global_position/raw/gps_vel", 1000, vel_cb); 
    mag_sub     = n.subscribe("/mavros/imu/mag", 1000, mag_cb);
    imu_sub     = n.subscribe("/mavros/imu/data", 1000, imu_cb);
    temp_sub    = n.subscribe("/mavros/imu/temperature_imu", 1000, temp_cb);
    
    image_pub_ = it_.advertise("gobi_image", 1);




}


void GobiBHG::set_img_name(string fname){
    self.imageFilename = fname;
}

void GobiBHG::open_camera(){
    // Open a connection to the first detected camera by using connection string cam://0
    ROS_INFO("*** Gobi ***: opening connection to cam://0");

    // Variables
    ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
    
    this->imageCount = 0;
    this->handle = XC_OpenCamera("cam://0");
    
    if(XC_IsInitialised(this->handle))
    {    
      // ... start capturing
      ROS_INFO("*** Gobi ***: Is initialized.");
      if (errorCode = XC_StartCapture(this->handle) != I_OK)
      {
        ROS_ERROR("*** Gobi ***: Could not start capturing, errorCode: %lu", errorCode);
        exit(1);
      }
      else if (XC_IsCapturing(this->handle)) // When the camera is capturing ...
      {
        ROS_INFO("*** Gobi ***: Successfully capturing.");

        /*       */  
        // Load the color profile delivered with this sample.
        if (errorCode = XC_LoadColourProfile(this->handle, "/home/user1/catkin_ws/src/usma_bhg/resources/ThermalBlue.png") != I_OK)
        {
            ROS_ERROR("*** Gobi ***: Problem while loading the desired colorprofile, errorCode: %lu", errorCode);
        }
        else
        {
            ROS_INFO("*** Gobi ***: Successfully loaded the desired colorprofile.");
        }
        
        // Set the colourmode so that the last loaded colorprofile is used.
        XC_SetColourMode(this->handle, ColourMode_Profile);

        // Determine framesize for a 32-bit buffer.
        this->frameSize = XC_GetWidth(this->handle) * XC_GetHeight(this->handle); // currently = 307200 which is 640x480
        ROS_INFO("*** Gobi ***: Frame size is: %d",this->frameSize);
      }
    }
    else
    {
        ROS_ERROR("*** Gobi ***: Failed to initialize. EXITING NOW!");
        exit(1);   
    }
}



string GobiBHG::get_homedir(){
    // Get home directory
    const char *tmpdir;
    std::string home;
    // check the $HOME environment variable, and if that does not exist, use getpwuid
    if ((tmpdir = getenv("HOME")) == NULL) {
        tmpdir = getpwuid(getuid())->pw_dir;
    }
    home.assign(tmpdir);
    return home;
}

string GobiBHG::char_array_to_string(char* char_array)
{
    string my_string(char_array);

    return my_string;
}

string GobiBHG::make_header()
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

string GobiBHG::make_logentry()
{
    string alt_str; 
    string gps_str; 
    string mag_str;    
    string imu_str;    
    string vel_str;  
    string temp_str;
    string output;
    
    alt_str = imageFilename + "," + char_array_to_string(dateTime) + "," 
              + to_string(rel_alt.monotonic) + "," + to_string(rel_alt.amsl) + "," + to_string(rel_alt.local) + "," + to_string(rel_alt.relative); 
    gps_str = to_string(gps_fix.status.status) + "," + to_string(gps_fix.status.service) + "," + to_string(gps_fix.latitude) + "," + to_string(gps_fix.longitude) + "," + to_string(gps_fix.altitude);; 
    mag_str = to_string(mag_data.magnetic_field.x) + "," + to_string(mag_data.magnetic_field.y) + "," + to_string(mag_data.magnetic_field.z);    
    imu_str = to_string(imu_data.orientation.x) + "," + to_string(imu_data.orientation.y) + "," + to_string(imu_data.orientation.z) + "," + to_string(imu_data.orientation.w) + ",";
    imu_str += to_string(imu_data.angular_velocity.x) + "," + to_string(imu_data.angular_velocity.y) + "," + to_string(imu_data.angular_velocity.z) + ",";
    imu_str += to_string(imu_data.linear_acceleration.x) + "," + to_string(imu_data.linear_acceleration.y) + "," + to_string(imu_data.linear_acceleration.z);    
    vel_str = to_string(vel_gps.twist.linear.x) + "," + to_string(vel_gps.twist.linear.y) + "," + to_string(vel_gps.twist.linear.z);
    vel_str += to_string(vel_gps.twist.angular.x) + "," + to_string(vel_gps.twist.angular.y) + "," + to_string(vel_gps.twist.angular.z);  
    temp_str = to_string(temp_imu.temperature);
    output = alt_str + "," + gps_str + "," + mag_str + "," + imu_str + "," + vel_str + "," + temp_str;
    return output;
}

void GobiBHG::mag_cb(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    mag_data = *msg;
}

void GobiBHG::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_data = *msg;
}

void GobiBHG::alt_cb(const mavros_msgs::Altitude::ConstPtr& msg)
{
    rel_alt = *msg;
}

void GobiBHG::gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    gps_fix = *msg;
}

void GobiBHG::vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    vel_gps = *msg;
}

void GobiBHG::temp_cb(const sensor_msgs::Temperature::ConstPtr& msg)
{
    temp_imu = *msg;
}

void GobiBHG::recordCallback(const std_msgs::Bool::ConstPtr& msg)
{
    record = *msg;
 
    //ROS_INFO("*** Gobi *** recordCallback: [%s]", record);
}

void GobiBHG::dirCallback(const std_msgs::String::ConstPtr& msg)
{
    this->data_dir = msg->data.c_str();
    std::vector<std::string> results; 
    boost::split(results, data_dir, [](char c){return c == '/';});
    //ROS_INFO("Subscibed dir is %s", data_dir);
    // /home/user1/Data/20200526_085616_798277
       
    string imgtmp = data_dir + "GOBI000088/";
    string csv_filename   = data_dir + results[4].c_str() + "_gobi.csv";
    
    if (mkdir(imgtmp.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1)
    {
        if( errno == 0 ) { // TODO fix this to return accurate feedback
            // TODO figure out why this check for zero? 
        }
        else if (errno == 17){
            // The directory already exists. Do nothing
        }
        else{
            ROS_INFO("*** Gobi ***: ERROR: Directory does not exist and MKDIR failed the errno is %i",errno );        
        }
    }    
    else
    {
        ROS_INFO("*** Gobi ***: Created Data Directory and establishing csv file" );  
        ROS_INFO("*** Gobi ***: Data directory is: [%s]", imgtmp.c_str());
        
        if (csv_outfile.is_open()){
            ROS_INFO("*** Gobi ***: GOBI CSV File is already open: [%s]", csv_filename.c_str());
        }
        else{        
            csv_outfile.open(csv_filename, std::ios_base::app); // DML is unrealable in creating the initial file
            if (!csv_outfile){
                ROS_INFO("*** Gobi ***: ERROR   FAILED TO OPEN GOBI CSV File: %s,  [%s]", strerror(errno), csv_filename.c_str()); 
                csv_outfile.open(csv_filename, std::ios_base::app); // DML is unrealable in creating the initial file           
                ROS_INFO("*** Gobi ***: ERROR   BLINDLY TRIED TO OPEN csv AGAIN: %s,  [%s]", strerror(errno), csv_filename.c_str());           
            } 
            ROS_INFO("*** Gobi ***: CSV file is: [%s]", csv_filename.c_str());                                  
        }
    }
    csv_outfile << make_header() << endl;  
}

int main(int argc, char **argv)
{
    //NBL: ROS Compliance{
    ros::init(argc, argv, "gobi_oop");  //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
    GobiBHG gobi_cam(&nh); // instantiate gobi class

    //Initialize ros time ONCE
    ros::Time::init();
    ros::Rate loop_rate(20); //TODO how fast should this be?
    while (ros::ok())
    {
        //Can be stopped with a Ctrl-C, but otherwise, loop forever.
        //NBL: record = true      

        //Before taking a picture, grab timestamp to record to filename  
        ros::Time rosTimeSinceEpoch = ros::Time::now();        
        std::time_t raw_time = static_cast<time_t>(rosTimeSinceEpoch.toSec());
        
        // take picture
        // save picture
        // publish picture  
       
        // DML: TODO stop using data_dir as global
        gobi_cam.set_img_name(data_dir + "/GOBI000088/GOBI000088" + "_" + dateTime + ".png");
        
        // Initialize the 32-bit buffer.
        frameBuffer = new dword[frameSize];
       
        if ((errorCode = XC_GetFrame(handle, FT_32_BPP_RGBA, XGF_Blocking, frameBuffer, frameSize * 4 /* bytes per pixel */)) != I_OK)
        {
            ROS_ERROR("*** Gobi ***: problem while fetching frame, errorCode %lu", errorCode);
        }
        else
        {         
            // TODO IF successful grab do a deep copy and write it to disk.
            cv::Mat cv_image(cv::Size(640, 480), CV_8UC4, frameBuffer);

            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
            cv_ptr->encoding = "rgba8";
            cv_ptr->header.stamp =  ros::Time::now();
            cv_ptr->header.frame_id = "/gobi";

            cv_ptr->image = cv_image;
            image_pub_.publish(cv_ptr->toImageMsg());

            if(record.data)    
            //if(true)
            {
                if((errorCode = XC_SaveData(handle, imageFilename.c_str(), XSD_SaveThermalInfo | XSD_RFU_1)) != I_OK)
                {
                    ROS_ERROR("*** Gobi ***: problem saving data, errorCode %lu", errorCode);
                }
                else
                {
                    imageCount += 1;
                    //NBL When you save your image, update your CSV file
                    /***  CSV FILE UPDATE ***/
                    csv_outfile << make_logentry() << endl;
                    
                    /***  CSV FILE UPDATE ***/
                    if (imageCount % 10 == 0){
                        ROS_INFO("*** Gobi ***: Image count: %d", imageCount);
                    }
                 }
            }// End of if record
        }// End if grame grabbed

        if (frameBuffer != 0)
        {
            delete [] frameBuffer;
            frameBuffer = 0;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    } // end of big while loop

    // Close the csv log file
    csv_outfile.close(); 
    
    // Below here needs to use printf as all Ros functionality ends with ctr-c
    printf("*** Gobi ***: Starting shutdown procedures.\n");
    if(XC_IsCapturing(handle))
    {
        // ... stop capturing.
        printf("*** Gobi ***: Stop capturing.\n");
        if ((errorCode = XC_StopCapture(handle)) != I_OK)
        {
            printf("*** GOBI ***: Could not stop capturing, errorCode: %lu\n", errorCode);
        }
    }

    // TODO add to deconstructor
    printf("*** Gobi ***: Clearing buffers.\n");
    if (frameBuffer != 0)
    {
       delete [] frameBuffer;
       frameBuffer = 0;
    }

    // When the handle to the camera is still initialised ...
    if (XC_IsInitialised(handle))
    {
        printf("*** Gobi ***: Closing connection to camera.\n");
        XC_CloseCamera(handle);
    }

    return 0;
}

// TODO Transition to OOP. Too many side effects are occuring and creatign unreliable performance. 
// Currently csv_outfile is causing problems with asynch code.
