#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>

//NBL: kbhit() includes
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

//NBL: Look for a keyboard hit.  This will be put to use to start and stop video recording.
//Credit to: https://cboard.cprogramming.com/c-programming/63166-kbhit-linux.html
int kbhit(int &ch)
{
  struct termios oldt, newt;
  //int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  /*  
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }*/
 
  return 0;
}

//NBL: Check start button pressed.
void check_start_toggle(std_msgs::String &msg, int &key){
    int hit = -1;
    if(key != ' '){       
        hit = kbhit(key);
        //std::cout << "hit = " << hit << " key = " << key << std::endl;
        if(key == ' '){
            msg.data = "1";
            
        }
    }
}

//NBL: Check stop button pressed.
void check_stop_toggle(std_msgs::String &msg, int &key){
    int hit = -1;
    if(key != 'q'){       
        hit = kbhit(key);
        //std::cout << "hit = " << hit << " key = " << key << std::endl;
        if(key == 'q'){
            msg.data = "0";
            
        }
    }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "record_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("record", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  /**
   * This is a message object. You stuff it with data, and then publish it.
   */
  std_msgs::String msg;
  int key = '~';
  msg.data = "0";
  ROS_INFO("Starting publisher.  0 = do not record,  1 = record images.  Currently = %s", msg.data.c_str());
  while (ros::ok())
  {
    //ROS_INFO("%s", msg.data.c_str());
    //std::cout << "publishing... " << std::boolalpha << msg.data << std::endl;
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
    /*
    std::cout << "Press SPACE to start capture...." << std::endl;
    while(key != ' ' && msg.data == "0"){
        //puts("Press a key!");       
        kbhit(key);
        if(key == ' '){
            msg.data = "1";
        }
    }
    //std::cout << "Begin capturing... press 'q' to stop..." << std::endl;
    getchar();
    
    while(key != 'q' && msg.data == "1"){
        //puts("Press a key!");       
        kbhit(key);
        if(key == 'q'){
            msg.data = "0";
        }
    }
    //std::cout << "Press SPACE to start capture...." << std::endl;
    getchar();
    */
    if(msg.data == "0"){
        //std::cout << "call, check_start_toggle()..." << std::endl;
        check_start_toggle(msg, key);
    }else if(msg.data == "1"){
        //std::cout << "call, check_stop_toggle()... " << std::endl;
        check_stop_toggle(msg, key);
    }
    //std::cout << "msg: " << msg.data.c_str() << " key: " << key << std::endl;
    /*
    if(count % 3){
        msg.data = "0";//ss.str();
    }else{
        msg.data = "1";
    }*/
    
    
  }


  return 0;
}
