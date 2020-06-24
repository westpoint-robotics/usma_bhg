#!/usr/bin/env python

from __future__ import print_function

import rospy
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import StreamRate
        
def set_stream_rate():
    # sets the pulse frequency in hertz
    # Minimum work value 16 tested up to 50
    rospy.wait_for_service('/mavros/set_stream_rate')
    try:
        set_rate = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
        value = StreamRate()
        resp1 = set_rate(1, 10 ,1)
    except rospy.ServiceException as e:
        rospy.logerr("+++++ TRIGGER:  Service call failed: %s"%e)
        
def set_servo_rate(rate):
    # sets the pulse frequency in hertz
    # Minimum work value 16 tested up to 50
    rospy.wait_for_service('/mavros/param/set')
    try:
        set_rate = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        value = ParamValue()
        value.integer = rate
        resp1 = set_rate('SERVO_RATE', value)
    except rospy.ServiceException as e:
        rospy.logerr("+++++ TRIGGER:  Service call failed: %s"%e)
        
def set_servo9_max(rate):
    # TODO Figure out exceptable range for this
    # maximum PWM pulse width in microseconds must be less than 32767.
    rospy.wait_for_service('/mavros/param/set')
    try:
        set_rate = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        value = ParamValue()
        value.integer = rate
        resp1 = set_rate('SERVO9_MAX', value)
    except rospy.ServiceException as e:
        rospy.logerr("+++++ TRIGGER: Service call failed: %s"%e)        
        
def set_servo9_pw(rate):
    # sets the pulse width in microseconds
    # TODO Figure out exceptable range for this
    rospy.wait_for_service('/mavros/cmd/command')
    try:
        send_cmd = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        # format for below is: ['broadcast', 'command', 'confirmation', 'param1', 'param2', 'param3', 'param4', 'param5', 'param6', 'param7'] 
        # 183 is the command for CMD_DO_SET_SERVO
        resp1 = send_cmd(0, 183, 1, 9, rate, 0, 0, 0, 0, 0)                
    except rospy.ServiceException as e:
        rospy.logerr("+++++ TRIGGER:  Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('trigger_setup')     
    img_hz = 20
    img_hz = int(rospy.get_param('/img_freq', 20))
    rospy.loginfo("+++++ TRIGGER:  Beginnig to setup trigger at %d hz." % img_hz)       
    pulse_width = int((1.0/img_hz * 1000000)/2) # The cameras prefer 50% duty cycle   
    
    rospy.sleep(30.) # TODO This is a guess at the delay needed to make sure the services are running and ready for the below commands
    set_servo_rate(img_hz)
    set_servo9_max(pulse_width+100) # TODO for now set max to slightly higher than required
    set_servo9_pw(pulse_width)
    rospy.loginfo("+++++ TRIGGER:  Completed the setup of trigger at %d hz." % img_hz)

