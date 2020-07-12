#! /usr/bin/env python3
"""
This module allows replaying videos side by side.

"""
import sys
import os
import time
import shutil
from os.path import expanduser
from datetime import datetime
import cv2
import numpy as np
import glob
import shutil

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

def set_datadir():
    home = expanduser("~")
    data_dir = home + "/Data/"  
    print("===== The data directory is: %s =====" % data_dir) 
    return data_dir    

data_dir = set_datadir()
print('path',data_dir)
#path = '/home/user1/Data/latest'
dirs_to_search = []


# Delete empty directories also
# Delete empty files to avoid errors later
for root, dirs, files in os.walk(data_dir):
    for dirc in dirs:
        #print(',\n'+ dirc)
        fullpath = os.path.join(root, dirc)        
        num_files = sum([len(files) for r, d, files in os.walk(fullpath)])
        if (num_files < 5): 
            shutil.rmtree(fullpath)
    for file in files:
        fullpath = os.path.join(root, file)
        try:
            if os.path.getsize(fullpath) == 0:
                os.remove(fullpath)
                print(f'Deleting {fullpath} it is empty')
        except Exception as e: 
            print(f'ERROR deleting the empty file: {fullpath}')

# Next 3 lines currently not used. Use when doing entire data dir.
subdirs = os.listdir(data_dir)
subdirs = sorted(subdirs)
#print(f'subdirs {subdirs}')

# comment the below line to run for directories in data directory.
arguments = len(sys.argv) - 1
if (arguments > 0):
    subdirs=[data_dir + sys.argv[1]]
else:
    subdirs=['/home/user1/Data/latest']
    
print("Creating video for the dirs in this list: ",subdirs)

for item in subdirs:
    if not os.path.isfile(os.path.join(data_dir, item)):
        dir2proc = os.path.join(data_dir, item)
        #img0_dir = f'{dir2proc}/FLIR_SN_18285440/*.ppm'
        #img1_dir = f'{dir2proc}/GOBI_SN_5003/*.png'
        img0_dir = f'{dir2proc}/FLIR_SN_18284612/*.ppm'
        img1_dir = f'{dir2proc}/GOBI_SN_5270/*.png'
        print(f'img0_dir: {img0_dir}')
        print(f'img0_dir: {img1_dir}')

# For each directory make a video
        img_array = []
#        img0_dir = '/home/user1/Data/20200527_180928_748671/FLIR18284612/*.ppm'
#        img1_dir = '/home/user1/Data/20200527_180928_748671/GOBI000088/*.png'
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
        height, width, layers = combine_pictures(imgs0[f],imgs1[g]).shape
        size = (width,height)
        outdir = set_datadir()
        outfile = f'{outdir}{img0_dir.split("/")[4]}.mp4'
        out = cv2.VideoWriter(outfile,cv2.VideoWriter_fourcc(*'AVC1'), 15, size)         
        doCont = True
        while doCont:
            #print(f'f {f} of {len(imgs0)} g {g} of {len(imgs1)}')
            img_array.append(combine_pictures(imgs0[f],imgs1[g]))
            flir_etime = get_etime(imgs0[f])
            gobi_etime = get_etime(imgs1[g])
            #print(f'0gobi_time {gobi_etime} flir_etime {flir_etime} -- {f} + {g} = {f+g} and totalfiles = {totalfiles}')
            # If Flir is behind Gobi iterate flir
           
            while flir_etime <= gobi_etime and doCont:
                f+=1
                if f > len(imgs1):
                    doCont = False
                else:                
                    # img_array.append(combine_pictures(imgs0[f],imgs1[g]))
                    out.write(combine_pictures(imgs0[f],imgs1[g]))
                    flir_etime = get_etime(imgs0[f])
                    gobi_etime = get_etime(imgs1[g])
                    print(f'flir:  1gobi_time {gobi_etime} flir_etime {flir_etime} -- {f} + {g} = {f+g} and totalfiles = {totalfiles}')
            while flir_etime >= gobi_etime and doCont:
                g+=1
                if g > len(imgs1):
                    doCont = False
                else:
                    #img_array.append(combine_pictures(imgs0[f],imgs1[g]))
                    out.write(combine_pictures(imgs0[f],imgs1[g]))
                    flir_etime = get_etime(imgs0[f])
                    gobi_etime = get_etime(imgs1[g])
                    print(f'gobi:  2gobi_time {gobi_etime} flir_etime {flir_etime} -- {f} + {g} = {f+g} and totalfiles = {totalfiles}')
            if flir_etime == gobi_etime:
                f+=1
                if f > len(imgs1):
                    doCont = False
                else:                
                    #img_array.append(combine_pictures(imgs0[f],imgs1[g]))
                    out.write(combine_pictures(imgs0[f],imgs1[g]))
                    flir_etime = get_etime(imgs0[f])
                    gobi_etime = get_etime(imgs1[g])
                    print(f'equal:  1gobi_time {gobi_etime} flir_etime {flir_etime} -- {f} + {g} = {f+g} and totalfiles = {totalfiles}')               

        print(f'Done parsing and stitching {totalfiles} files. Now starting to create the video.')


        print(f'Writing video to file: {outfile}, {len(img_array)}') 
#        for i in range(len(img_array)):
#            out.write(img_array[i])
        out.release()



exit()
