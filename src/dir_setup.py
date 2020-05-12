#! /usr/bin/python

# os for changing directories for saving images, data, CSVs
import os
# rospy for the subscriber
import rospy
import datetime
#from datetime import datetime
from std_msgs.msg import String
from os.path import expanduser
import dir_cleanup as dc

# This function was taken from roslogging.py to create a symlink to the latest directory
def renew_latest_logdir(logfile_dir):
    log_dir = os.path.dirname(logfile_dir)
    latest_dir = os.path.join(log_dir, '..', 'latest')
    if os.path.lexists(latest_dir):
        if not os.path.islink(latest_dir):
            return False
        os.remove(latest_dir)
    os.symlink(logfile_dir, latest_dir)
    return True

def main():
    directory_topic = "directory"
    home = expanduser("~")
    dataDirectory = home + "/Data/"
    rospy.init_node('directory_setup')  
    
    tNow = rospy.get_time() # current date and time
    #rospy.loginfo("tNow = %.3f\n", tNow)
    missionName = datetime.datetime.fromtimestamp(tNow).strftime('%Y%m%d_%H%M%S_%f')
    #missionName = time.strftime('%Y%m%d_%H%M%S_%03d', time.localtime(tNow))
    missionDirectory = dataDirectory + missionName + "/"
    pub = rospy.Publisher(directory_topic, String, queue_size=10)
    
    if not os.path.exists(missionDirectory):
        os.makedirs(missionDirectory)
        try:
            success = renew_latest_logdir(missionDirectory)
            if not success:
                sys.stderr.write("INFO: cannot create a symlink to latest data directory\n")
        except OSError as e:
            sys.stderr.write("INFO: cannot create a symlink to latest data directory: %s\n" % e)

    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        pub.publish(missionDirectory) #TODO change system to handle publish of missionDirectory
        r.sleep()
        
    # Below here is code to cleanup any unused directories in the Data directory
    # Move the latest bagfile into the latest directory
    disk_cleaner = dc.DirCleanup() 
    disk_cleaner.move_bagfile(dataDirectory)   
    disk_cleaner.find_dirs_to_delete(dataDirectory)
    print("The following directories were unused and thus deleted:\n",disk_cleaner.get_dirs_to_delete())
    disk_cleaner.delete_folders()
    

if __name__ == '__main__':
    main()


