#! /usr/bin/python

# os for changing directories for saving images, data, CSVs
import os
# rospy for the subscriber
import rospy
import datetime
#from datetime import datetime
from std_msgs.msg import String
from os.path import expanduser
from std_msgs.msg import UInt16MultiArray
from diagnostic_msgs.msg import DiagnosticArray

class HealthMonitor:
    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher("/number_count", UInt16MultiArray, queue_size=10)
        self.diagnostic = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnostic_cb)
        self.temperature_case = 0
        self.temperature_cores = []
        self.temperature_status = ''
        self.diag = {}

    def diagnostic_cb(self, msg):
        del self.temperature_cores[:]
        for status in msg.status:
            if "libsensors_monitor" in status.name:
                if "Package" in status.name:
                    self.temperature_case = int(status.values[0].value)  #get the case temperature
                    #rospy.loginfo(status.values[0].value)  #get the case temperature
                elif "Core" in status.name:
                    self.temperature_cores.append(int(status.values[0].value))
            elif "System" in status.name:
                for entry in status.values:
                    if "CPU Load" in entry.key:
                        rospy.loginfo(entry.value) #['CPU Load (%)'])

        if self.temperature_case > 90:
            self.temperature_status = "Red"
        elif self.temperature_case > 70:
            self.temperature_status = "Amber"
        else:
            self.temperature_status = "Green"

    def show_health(self):
        rospy.loginfo(self.diag)

if __name__ == '__main__':
    rospy.init_node('mav_monitor')
    hm = HealthMonitor()
    r = rospy.Rate(2) # 5hz
    while not rospy.is_shutdown():
        #hm.show_health()
        #rospy.loginfo("NUC Temperature status is %s" % hm.temperature_status)
        r.sleep()

'''
hardware_id: "/dev/ttyUSB0:921600"


[INFO] [1588288458.675274]: mavros: GPS
[INFO] [1588288458.675944]: mavros: Heartbeat # all zero
[INFO] [1588288458.676528]: mavros: System    # all zero
[INFO] [1588288458.677075]: mavros: Battery
[INFO] [1588288458.677610]: mavros: Time Sync
[INFO] [1588288459.174404]: mavros: GCS bridge
[INFO] [1588288460.174617]: mavros: FCU connection

'''
