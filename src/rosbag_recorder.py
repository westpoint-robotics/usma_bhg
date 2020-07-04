#! /usr/bin/python

'''
This program starts recording rosbag files when record is true
and stops when it is false.
'''

import os
import subprocess, shlex
import signal, psutil

import rospy
import rosbag
from rosbag import rosbag_main
from std_msgs.msg import Bool
from std_msgs.msg import String  



is_recording = False
last_state = False
bag_filename = '/tmp/BHG_DATA/bhg.bag'
crnt_bagfile = 'crnt_bhg.bag'

def record_callback(msg):
    global is_recording
    is_recording = msg.data
    
def directory_callback( msg):
    global bag_filename
    global crnt_bagfile
    data_dir = msg.data
    dir_time = data_dir.split("/")[4] #   missionName  "20200615_145002_422"
    bag_filename = data_dir + "bhg_" + dir_time + ".bag"
    if (crnt_bagfile != bag_filename):
        if(not (os.path.isdir(data_dir))): # if data directory does not exist then create it
            os.mkdir(data_dir)
            rospy.loginfo("***** ROSBAG: Data directory Created: " + data_dir)
        rospy.loginfo("***** ROSBAG: Bagfile filename is: " + bag_filename)
    crnt_bagfile = bag_filename

def kill_child_processes(parent_pid, sig=signal.SIGINT):
    try:
      parent = psutil.Process(parent_pid)
    except psutil.NoSuchProcess:
      return
    children = parent.children(recursive=True)
    for process in children:
      process.send_signal(sig)

rosbag_proc = []
rospy.init_node('rosbagger')
rospy.Subscriber('/directory', String, directory_callback)
rospy.Subscriber("/record", Bool, record_callback)
r = rospy.Rate(10) 
while not rospy.is_shutdown():

    if last_state != True and is_recording == True:
        #start bagging
        #rosbag_main.record_cmd(['-O testrt.bag','-a'])
        command = "rosbag record -a -x '(.*)camera(.*)' -O " + bag_filename
        command = shlex.split(command)
        rosbag_proc = subprocess.Popen(command)

        print("Start Bagging",is_recording,last_state)
    
    elif last_state != False and is_recording == False: 
        kill_child_processes(rosbag_proc.pid)
        print("Stoped Bagging",is_recording,last_state,rosbag_proc.pid)

    last_state = is_recording
    r.sleep()
kill_child_processes(rosbag_proc.pid)    
    
    #rosbag_main.record_cmd(['-O testrt.bag','-a'])
