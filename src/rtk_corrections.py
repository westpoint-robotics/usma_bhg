#! /usr/bin/python
from __future__ import print_function

"""
This module manages the directories for BHG. It does the following:
    - Creates the data directory and mission directory for saving the images
    - Publishes the mission directory for the cameras to use for saving images
    - Publishes the number of files in the subdirectories
    - Cleans up (deletes) unused directories
    
It also does 

"""
import os
import rospy
from mavros_msgs.msg import RTCM

import serial
import time
import binascii

baud_rate = 115200  
com_port1 = '/dev/ttyUSB0'
read_timeout = 0.1 
ser = serial.Serial(port=com_port1, baudrate=baud_rate, bytesize=8, parity='N', stopbits=1, timeout=read_timeout)

#this will store the line
seq = []
count = 1

outfile='freewave_serial.bin'
while True:
    new_data = bytearray(ser.read(255))
    if (len(new_data)>1):
        with open(outfile, 'ab') as f:
            f.write(new_data)
        print("NEW DATA READ", type(new_data),len(new_data))
        print(binascii.hexlify(new_data))

#while True:
#    c = ser.read(1)
#    if(ord(c) == 0):
#        if (len(seq)>1):
#            for item in seq:
#                out_c = "{0:02x}".format(ord(item))
#                print(out_c, end=' ') 
#            print("")           
#        seq = []
#    else:
#        seq.append(c)        

#        seq.append(data) #convert from ASCI
#        joined_seq = ''.join(str(v) for v in seq) #Make a string from array

#        if chr(c) == '\n':
#            print("Line " + str(count) + ': ' + joined_seq)
#            seq = []
#            count += 1
#            break
    
ser.close()




#while True:
#    s = listener.read(10000)
##    print("-", s.decode('UTF-8'))
#    print('{:02x}',s)
#    time.sleep(.1)
    
#rcvd = ""
#while True:
#    c = listener.read()
#    if len(c) == 0:
#        break
#    rcvd += c
#    for ch in c:
#        print ord(ch), " ",  
#    print(" THE END ")
        
        
          
