#! /usr/bin/python

import csv
# os for changing directories for saving images, data, CSVs
import os
import subprocess
# rospy for the subscriber
import rospy
from datetime import datetime
from std_msgs.msg import String

dataDirectory = "/home/user1/Data/"; 
# Camera directories, csv file, and bag file all go here
bag_directory = ''

def directory_callback(msg):
    global bag_directory
    bag_directory = msg.data
    subprocess.call(['rosbag record -a -o /home/user1/Data/'], shell=True)

def main():
    rospy.init_node('rosbag_recoder')
    # Set up your subscriber and define its callback
    rospy.Subscriber('directory', String, directory_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
    
    
'''
rostopic pub /record std_msgs/Bool True
'''    
