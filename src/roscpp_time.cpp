//NBL: ROS Compliance
#include "ros/ros.h"
#include <iostream>   // std::cout
#include <locale>
#include <string>     // std::string, std::to_string
#include <chrono>
#include <ctime>
#include <math.h> 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

using namespace std;
using namespace std::chrono;

int main(int argc, char **argv)
{
    time_t tt;
    tm local_tm;
    tm last_tm;

    ros::Time::init();
    ros::Time rosTimeSinceEpoch = ros::Time::now();
    std::time_t raw_time = static_cast<time_t>(rosTimeSinceEpoch.toSec());    
    local_tm = *localtime(&raw_time);

    char dateTime [50];
    int n;
    n=sprintf (dateTime, "%d%02d%02d_%02d%02d%02d_%03d", local_tm.tm_year + 1900, local_tm.tm_mon + 1, local_tm.tm_mday, local_tm.tm_hour, local_tm.tm_min, local_tm.tm_sec, rosTimeSinceEpoch.nsec);
    
}
