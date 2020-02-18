#include <iostream>   // std::cout
#include <string>     // std::string, std::to_string
#include <fstream>

using namespace std;

string csvFilename;
string csvData;

string make_header()
{
    string header = "";
    header = "rostime,rel_alt.monotonic,rel_alt.amsl,rel_alt.local,rel_alt.relative,";
    header += "gps_fix.status.status,gps_fix.status.service,gps_fix.latitude,gps_fix.longitude,gps_fix.altitude,";
    header += "imu_data.magnetic_field.x,imu_data.magnetic_field.y,imu_data.magnetic_field.z,";
    header += "imu_mag.orientation.x,imu_mag.orientation.y,imu_mag.orientation.z,imu_mag.orientation.w, imu_mag.angular_velocity.x,imu_mag.angular_velocity.y,imu_mag.angular_velocity.z,";
    header += "imu_mag.linear_acceleration:.x,imu_mag.linear_acceleration:.y,imu_mag.linear_acceleration:.z,";
    header += "vel_gps.twist.linear.x,vel_gps.twist.linear.y,vel_gps.twist.linear.z,";
    header += "vel_gps.twist.angular.x,vel_gps.twist.angular.y,vel_gps.twist.angular.z,";
    header += "temp_imu.temperature";
    return header;
}

int main(int argc, char **argv)
{
    csvFilename   = "/home/user1/Data/deleteme_gobi.csv";

    std::ofstream outfile;
    csvData = make_header();
    cout<<"anything"<<endl;    
    outfile.open(csvFilename, std::ios_base::app); // append instead of overwrite
    for(int i = 0; i < 10; i++)
    {
        
        outfile << csvData << endl;
        cout << csvData << endl;
    }
    
    return 0;
}
