#! /usr/bin/python
# license removed for brevity
import rospy
import datetime
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        tnow = rospy.get_time()
        rospy.loginfo("now = %.3f\n", tnow)
        #mytime = time.strftime('%Y-%m-%d %H:%M:%S.%f', time.localtime(tnow)) 
        mytime = datetime.datetime.fromtimestamp(tnow).strftime('%Y-%m-%d %H:%M:%S.%f')

        #rospy.loginfo("humantime = %s\n", mytime)



        #hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(mytime)
        pub.publish(mytime)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



'''


import rospy
import time
#from time import localtime, gmtime, strftime

def main():
    rospy.init_node('rospy_datetime')


    now = rospy.get_time()
    rospy.loginfo("now = %.3f\n", now)
    mytime = time.strftime('%Y%m%d_%H%M%S_%03d', time.localtime(now))    
    rospy.loginfo("humantime = %s\n", mytime)
if __name__ == '__main__':
    main()

'''
