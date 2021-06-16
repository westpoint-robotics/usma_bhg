#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavros_msgs/RTCM.h"
#include <sstream>

//#include <SerialPort.h>
//#include <SerialStream.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtk_corrections");

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

// Create a object instance. https://libserial.readthedocs.io/en/latest/tutorial.html
//SerialPort   my_serial_port;

//// Obtain the serial port name from user input.
//std::cout << "Please enter the name of the serial device, (e.g. /dev/ttyUSB0): " << std::flush;
//std::string serial_port_name = "/dev/ttyUSB0";

//// Open the serial port for communication.
//my_serial_port.Open( serial_port_name );
//my_serial_port.SetBaudRate( BAUD_115200 );





  ros::Rate loop_rate(10);
  int count = 0;  
  while (ros::ok())
  {

  
  
  
  
  
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
//    mavros_msgs::RTCM msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
