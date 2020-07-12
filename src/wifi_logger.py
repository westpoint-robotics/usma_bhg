#! /usr/bin/env python2
"""
This module logs wifi Link quality, noise level, signal level

"""
import rospy
import datetime
import subprocess
import time
import argparse

parser = argparse.ArgumentParser(description='Display WLAN signal strength.')
#parser.add_argument(dest='interface', nargs='?', default='wlx00c0caa8a144',
#                    help='wlan interface (default: wlan0)')
parser.add_argument(dest='interface', nargs='?', default='wlp1s0',
                    help='wlan interface (default: wlan0)')
args = parser.parse_args()

rospy.init_node('wifi_logger')

crnt_time = rospy.get_time()
outfile_name = '/home/user1/' + datetime.datetime.fromtimestamp(crnt_time).strftime('%Y%m%d_%H%M%S_wifi.log') 
r = rospy.Rate(2)
while not rospy.is_shutdown(): 
    crnt_time = rospy.get_time()
    datetimeData = datetime.datetime.fromtimestamp(crnt_time).strftime('%Y%m%d_%H%M%S_%f')
    #cmd = subprocess.Popen('iwconfig %s' % args.interface, shell=True,stdout=subprocess.PIPE)
    cmd = subprocess.Popen('iwconfig', shell=True,stdout=subprocess.PIPE)
    for line in cmd.stdout:
        if 'Link Quality' in line:
            outstring = datetimeData + ',' + line.lstrip(' ')
            with open(outfile_name, 'a+') as f:
                f.write(outstring)
            print(outstring)
        elif 'Not-Associated' in line:
            print('No signal')
    r.sleep()
