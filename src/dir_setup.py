#! /usr/bin/python
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
import datetime
from std_msgs.msg import String
from std_msgs.msg import Int32
from os.path import expanduser
from std_msgs.msg import Bool
import dir_cleanup as dc

class DirSetup:

    def __init__(self):
        self.data_dir = self.setup_data_dir()
        self.mission_directory = self.setup_mission_dir()
        self.dir_pub = rospy.Publisher('directory', String, queue_size=10)
        self.gobi_fc_pub = rospy.Publisher("gobi_fc", Int32, queue_size=10)
        self.flir_fc_pub = rospy.Publisher("flir_fc", Int32, queue_size=10)
        rospy.Subscriber("/record", Bool, self.record_cb)
        self.last_rec = False

    # Responds to record boolean topic. If transition from not recording to recording
    # it creates a new directory structure and publishes it.                
    def record_cb(self, msg):
        if self.last_rec == False and msg.data == True:
            self.mission_directory = self.setup_mission_dir()
            self.dir_pub.publish(self.mission_directory)
            rospy.loginfo("===== DIR_SETUP:  A new mission directory was created at: %s" % self.mission_directory) 
        self.last_rec = msg.data
                
    # Creates a symlink clled 'latestl' that points to the current directory. This function was taken 
    # from roslogging.py, which is part of the core ROS software.  
    def renew_latest_logdir(self, logfile_dir):
        log_dir = os.path.dirname(logfile_dir)
        latest_dir = os.path.join(log_dir, '..', 'latest')
        if os.path.lexists(latest_dir):
            if not os.path.islink(latest_dir):
                return False
            os.remove(latest_dir)
        os.symlink(logfile_dir, latest_dir)
        return True
    
    # Creates a string representing the base data directory.        
    def setup_data_dir(self):
        home = expanduser("~")
        data_dir = home + "/Data/"  
        rospy.loginfo("===== DIR_SETUP:  The data directory is: %s" % data_dir) 
        return data_dir

    def setup_mission_dir(self):        
        tNow = rospy.get_rostime() # current date and time
        millisec = int(round(tNow.nsecs/1000000.0))
        rospy.loginfo("millis is: %d",millisec)
        if millisec < 10:
            millisec = '00' + str(millisec)
        elif millisec < 100:
            millisec = '0' + str(millisec)
        else:
            millisec = str(millisec)            
        mission_name = datetime.datetime.fromtimestamp(tNow.secs).strftime('%Y%m%d_%H%M%S_') 
        mission_name += millisec   
        mission_dir = self.data_dir + mission_name + "/"
        rospy.loginfo("===== DIR_SETUP:  The mission directory is: %s  % mission_dir)         
        if not os.path.exists(mission_dir):
            os.makedirs(mission_dir)
            rospy.loginfo("===== DIR_SETUP:  Created Mission Directory  )         
            try:
                success = self.renew_latest_logdir(mission_dir)
                if not success:
                    sys.stderr.write("===== DIR_SETUP:  Cannot create a symlink to latest data directory\n")
            except OSError as e:
                sys.stderr.write("===== DIR_SETUP:  Cannot create a symlink to latest data directory: %s\n" % e)
        return mission_dir

    def count_files(self):
        path = self.mission_directory
        for root, dirs, files in os.walk(path):
            for dir2 in dirs:
                #print(os.path.join(root, dir2), len(os.listdir(os.path.join(root, dir2))))
                file_count = len(os.listdir(os.path.join(root, dir2)))  
                if dir2.startswith("GOBI"):
                    self.gobi_fc_pub.publish(file_count)
                elif dir2.startswith("FLIR"):
                    self.flir_fc_pub.publish(file_count)
                    
    def pub_mission_dir(self):
        self.dir_pub.publish(self.mission_directory)
        
    def dir_cleanup(self):
        # Below here is code to cleanup any unused directories in the Data directory
        # Move the latest bagfile into the latest directory
        disk_cleaner = dc.DirCleanup() 
        disk_cleaner.move_bagfile(self.data_dir)   
        disk_cleaner.find_dirs_to_delete(self.data_dir)
        # Next line must use print as ROSCORE is shutdown at this point.
        print("===== DIR_SETUP:  The following directories were unused and thus deleted:\n",disk_cleaner.get_dirs_to_delete())
        disk_cleaner.delete_folders()       

if __name__ == '__main__':
    rospy.init_node('directory_setup')  
    ds = DirSetup()
    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        ds.pub_mission_dir()
        ds.count_files()
        r.sleep()
    print("===== DIR_SETUP:  DATA DIR IS",ds.data_dir,ds.mission_directory)
    ds.dir_cleanup()

