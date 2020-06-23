# NOTES for PWM Control of AUX ports on Pixhawk

## Mavproxy CLI
- To connect to mavproxy (will not work if the mavproxy ROS node is also running)  
`mavproxy.py --master=/dev/ttyACM1`  
- Commands to set parameters:  
`param set SERVO_RATE 25`  
`param set SERVO9_MAX 30000`  
- Command to set pulse width:
`servo set 9 1500`
`param show`

## Mavros commands using ROS services:  
- Get the servo rate in HZ  
`rosservice call /mavros/param/get SERVO_RATE`  
- Set the servo rate in Hz  
`rosservice call /mavros/param/set SERVO_RATE '[25, 0.0]'`  
- Get the servo max pulse width in micro seconds  
`rosservice call /mavros/param/get SERVO9_MAX`  
- Set the servo max pulse width in micro seconds  
`rosservice call /mavros/param/set SERVO9_MAX '[30000, 0.0]'`  
- Set the pulse width  
`rosservice call /mavros/cmd/command 183 9 25000 0 0 0 0 0`

### Other ROS Service cli commands:
`rosservice list`  
`rosservice info /mavros/param/set `  
`rossrv show mavros_msgs/ParamSet`  
`rosservice list`

## Mavros commands: 
`rosrun mavros checkid`  
`rosrun mavros mavparam set SERVO_RATE 20`  
`rosrun mavros mavparam set SERVO9_MAX 30000`  
`rosrun mavros mavcmd long 183 9 25000 0 0 0 0 0`  

## Notes:
- https://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html  
Messages that don’t have a target <sysid,compid> are processed by the vehicle and then forwarded to each known system/component  
- https://ardupilot.org/dev/docs/mavlink-basics.html  
The sender always fills in the System ID and Component ID fields so that the receiver knows where the packet came from. The System ID is a unique ID for each vehicle or ground station. Ground stations normally use a high system id like “255” and vehicles default to use “1” (this can be changed by setting the SYSID_THISMAV parameter). The Component ID for the ground station or flight controller is normally “1”. Other MAVLink capable device on the vehicle (i.e. companion computer, gimbal) should use the same System ID as the flight controller but use a different Component ID

## References:
- https://mavlink.io/en/messages/common.html  
- https://ardupilot.org/plane/docs/parameters.html  
- https://ardupilot.org/dev/docs/mavlink-basics.html  

## Lines used in the launch file:
- These worked except the Param set commands would fail often as the service was not fully running and ready for them when they were sent.
```html
    <node pkg="rosservice" type="rosservice" name="mavros_set_stream_service" args="call -wait /mavros/set_stream_rate 0 10 1" />
    <node pkg="rosservice" type="rosservice" name="set_servo_rate" args="call -wait /mavros/param/set SERVO_RATE '[25, 0.0]'" />
    <node pkg="rosservice" type="rosservice" name="set_servo9_max" args="call -wait /mavros/param/set SERVO9_MAX '[50000, 0.0]'" />
    <node pkg="mavros" type="mavcmd" name="set_pixhwk_trigger_width" args="-wait long 183 9 25000 0 0 0 0 0"/>
```
- The ultimate fix was writing python to do this as seen in: src/mavlink_trigger.py   
