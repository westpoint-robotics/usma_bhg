#! /usr/bin/env python3
"""
This module allows replaying videos side by side.

"""

import os
import time
import shutil
from os.path import expanduser
from datetime import datetime

import cv2
import numpy as np
import glob
 
def get_etime(name):
    '''
    Converts the date time in the filename into epoch time for math comparison. 
    '''
    
    fname_split=name.split('/')[-1:][0].split('.')[0].split('_')
    time_string = f'{fname_split[1]}-{fname_split[2]}.{fname_split[3]}'
    #print(outstring) # 2020-05-28 18-02-08.44
    utc_time = datetime.strptime(time_string, "%Y%m%d-%H%M%S.%f")
    epoch_time = (utc_time - datetime(1970, 1, 1)).total_seconds()
    return epoch_time

def combine_pictures(flir_img, gobi_img):
    img0 = cv2.imread(flir_img)    
    img1 = cv2.imread(gobi_img)
    dim = (640,480)
    img0 = cv2.resize(img0, dim)
    img_out = np.concatenate((img0, img1), axis=1)
    return img_out
 
img_array = []
img0_dir = '/home/user1/Data/20200528_154219_500904/FLIR18284612/*.ppm'
img1_dir = '/home/user1/Data/20200528_154219_500904/GOBI000088/*.png'
imgs0 = sorted(glob.glob(img0_dir))
imgs1 = sorted(glob.glob(img1_dir))
if (len(imgs0) > len(imgs1)):
    filecount = len(imgs0)
else:
    filecount = len(imgs1)
totalfiles =  len(imgs0) + len(imgs1)  
f = g = 0 # counter for flir and gobi file indexes   
print(f'len(imgs0) {len(imgs0)} -- len(imgs1) {len(imgs1)}')  
flir_etime = gobi_etime = 0
#for i in range(filecount):
while totalfiles > f + g + 2:
    flir_etime = get_etime(imgs0[f])
    gobi_etime = get_etime(imgs1[g])
    img_array.append(combine_pictures(imgs0[f],imgs1[g]))
    #print(f'0gobi_time {gobi_etime} flir_etime {flir_etime} -- {f} + {g} = {f+g} and totalfiles = {totalfiles}')
    # If Flir is behind Gobi iterate flir
    while flir_etime <= gobi_etime and len(imgs0)-1 > f:
        flir_etime = get_etime(imgs0[f])
        gobi_etime = get_etime(imgs1[g])
        img_array.append(combine_pictures(imgs0[f],imgs1[g]))
        #print(f'1gobi_time {gobi_etime} flir_etime {flir_etime} -- {f} + {g} = {f+g} and totalfiles = {totalfiles}')
        f+=1
    while flir_etime >= gobi_etime and len(imgs1)-1 > g:
        flir_etime = get_etime(imgs0[f])
        gobi_etime = get_etime(imgs1[g])
        img_array.append(combine_pictures(imgs0[f],imgs1[g]))
        #print(f'2gobi_time {gobi_etime} flir_etime {flir_etime} -- {f} + {g} = {f+g} and totalfiles = {totalfiles}')
        g+=1
#    f+=1
#    g+=1        
#    print(f'gobi_time {gobi_etime} flir_etime {flir_etime}')
        

#    img0 = cv2.imread(imgs0[f])    
#    img1 = cv2.imread(imgs1[g])
#    
#    img_array.append(combine_pictures(img0,img1)).
print(f'Done parsing and stitching {totalfiles} files. Now starting to create the video.')
height, width, layers = img_array[0].shape
size = (width,height)
out = cv2.VideoWriter('project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)
 
for i in range(len(img_array)):
    out.write(img_array[i])
out.release()


exit()
def set_datadir():
    home = expanduser("~")
    data_dir = home + "/Data/"  
    print("===== The data directory is: %s =====" % data_dir) 
    return data_dir    

def make_datetime(name):
    '''
    Converts the date time in the filename into 2 outputs. 
    Output 1 is the epoch time for mathmatical comparison
    Output 2 is a human readable string that matches the one used by ardupilot
    '''
    fname_split=name.split('_')    
    m_sec = round(int(fname_split[3][:3])/10.0)
    if m_sec < 10:
        m_sec = f'{m_sec}0'
    outstring=f'{fname_split[1][:4]}-{fname_split[1][4:6]}-{fname_split[1][6:8]} {fname_split[2][:2]}:{fname_split[2][2:4]}:{fname_split[2][4:6]}.{m_sec}'

    #print(outstring) # 2020-05-28 18-02-08.44
    utc_time = datetime.strptime(outstring, "%Y-%m-%d %H:%M:%S.%f")
    epoch_time = (utc_time - datetime(1970, 1, 1)).total_seconds()
    return (epoch_time,outstring)


if __name__ == '__main__':
    rootDir = set_datadir() + '20200528_180208_387820/'
    rootDir = '/home/user1/DATA_ARCHIVE/BHGTest'
    flr_dir = ''
    gob_dir = ''
    #convert_bin_2_csv()
    #convert_bin_2_csv(False)
    remove_empty_files(rootDir)
    delete_folders(find_dirs_to_delete(rootDir))

