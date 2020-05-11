#! /usr/bin/python

# os for changing directories for saving images, data, CSVs
import os
# rospy for the subscriber
import rospy
import datetime
#from datetime import datetime
from std_msgs.msg import String
from os.path import expanduser
from collections import namedtuple # used for disk analsis
from std_msgs.msg import UInt16MultiArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Vector3Stamped

class HealthMonitor:
    def __init__(self):
        self.counter = 0
        self.disk_pub = rospy.Publisher("/disk_usage", Vector3Stamped, queue_size=10)
        self.diagnostic = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnostic_cb)
        self.temperature_case = 0
        self.temperature_cores = []
        self.temperature_status = ''
        self.diag = {}
        self.fcu_usage = 0
        
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
                        self.fcu_usage = float(entry.value) # percentage of the fcu cpu usage     
            
    def show_health(self):
        rospy.loginfo(self.diag)
        
    def publish_disk_usage(self, path):
        """publish disk usage statistics about the given path.
           unit of measure is GB. 
        """
        msg = Vector3Stamped()
        st = os.statvfs(path)
        free = st.f_bavail * st.f_frsize/1000000000.0
        total = st.f_blocks * st.f_frsize/1000000000.0
        used = (st.f_blocks - st.f_bfree) * st.f_frsize/1000000000.0
        msg.vector.x = total
        msg.vector.y = used
        msg.vector.z = free
        self.disk_pub.publish(msg) 

if __name__ == '__main__':
    rospy.init_node('health_monitor')
    hm = HealthMonitor()
    
    r = rospy.Rate(2) # 5hz
    while not rospy.is_shutdown():
        #hm.show_health()
        #rospy.loginfo("NUC Temperature status is %s" % hm.temperature_status)
        
        if hm.temperature_case > 90:
            hm.temperature_status = "Red"
            rospy.logwarn("HHHHHHHM The NUC Temperature exceeds 90 Degrees HHHHHHHM")
        elif hm.temperature_case > 70:
            hm.temperature_status = "Amber"
        else:
            hm.temperature_status = "Green"
        if hm.fcu_usage > 90:
            rospy.logwarn("HHHHHHHM The FCU CPU usage is above 90% HHHHHHHM")
        
        hm.publish_disk_usage("/home")
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
