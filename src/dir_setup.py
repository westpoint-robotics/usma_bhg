#! /usr/bin/python

# os for changing directories for saving images, data, CSVs
import os
# rospy for the subscriber
import rospy
from datetime import datetime
from std_msgs.msg import String

def main():
    directory_topic = "directory"
    dataDirectory = "/home/user1/Data/"
    now = datetime.now() # current date and time
    missionName = now.strftime("%Y%m%d_%H%M%S_%f")[:-3]
    missionDirectory = dataDirectory + missionName + "/"
    pub = rospy.Publisher(directory_topic, String, queue_size=10)
    rospy.init_node('directory_setup')
    if not os.path.exists(missionDirectory):
        os.makedirs(missionDirectory)

    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        pub.publish(missionName) #TODO change system to handle publish of missionDirectory
        r.sleep()

if __name__ == '__main__':
    main()


