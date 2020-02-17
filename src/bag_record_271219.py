#! /usr/bin/python

import csv
# os for changing directories for saving images, data, CSVs
import os
import subprocess
# rospy for the subscriber
import rospy
from datetime import datetime
from std_msgs.msg import String

dataDirectory = "/home/user1/Data/"
# Camera directories, csv file, and bag file all go here
now = datetime.now() # current date and time
missionName = now.strftime("%Y%m%d_%H%M%S_%f")[:-3] 
missionDirectory = dataDirectory + missionName + "/"
bag_directory = ''

def directory_setup():
    if not os.path.exists(missionDirectory):
        os.makedirs(missionDirectory)
#    if not os.path.exists(flirDirectory):
#        os.makedirs(flirDirectory)
    #if not os.path.exists(gobiDirectory):
    #    os.makedirs(gobiDirectory)    
    subprocess.call(['rosbag record -a -o ' + missionDirectory], shell=True)

def main():
    pub = rospy.Publisher('directory', String, queue_size=10)
    rospy.init_node('rosbag_recoder')
    directory_setup()
    
    pub.publish(missionDirectory)
    # Spin until ctrl + c
    rospy.sleep(3)
    rospy.spin()

if __name__ == '__main__':
    main()
    
    
'''
rostopic pub /record std_msgs/Bool True
'''    
