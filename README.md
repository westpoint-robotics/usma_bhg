# usma_bhg
ROS nodes, launch files, etc. for Bloodhound Gang project (USMA)

This document will show you how to 
1. Access the companion computer, "brix012" on the USMA BHG T650 platform from a groundstation computer running Ubuntu 16.04 or 18.04.
2. Launch ROS nodes, "bool_pub" and "camera_subscriber", which will allow
3. The toggling of the ROS topic, "record" which indicates whether to start recording images from a FLIR Blackfly camera, which can be mounted to the UAV.
4. Mount the FLIR camera and connect it to the Brix.

1. Ground Station to Companion Computer Connection
a. Ascertaining the Brix IP Address
   If you will be operating with access to a WREN connection on post at West Point, the Brix IP Address will be 10.212.149.243. 
   Otherwise, if you are operating out of the Robotics Trailer for the first time at a new location do the following.
   - Connect a USB hub to a free USB port on the Brix.
   - Connect a USB keyboard and mouse to this hub.
   - Connect one of the trailer monitors with an HDMI cable.
   - Unplug the power cable running from the Brix to the power distribution board on the UAV, and plug in the Brix adapter        instead (plug the other end into a wall outlet).
   - Turn on the Brix using the power button on the underside of the computer at the aft, port corner with respect to the          front of the UAV (blue motors).  Log in using standard RRC credentials.
   - Open a terminal and use the command, "ip a".  You should see a line similar to, "link/ether 00:c0:ca:97:df:c1 brd            ff:ff:ff:ff:ff:ff".  Below that, should read "inet" and then a number, for example starting with 10, or 198, or similar.      This is the start of the IP address you want.  Record this in a project/mission notebook.
b. Connecting from the Ground Station
   Your ground station computer should be a laptop or desktop running Ubuntu 16.04, or 18.04.  Turn on and log into this        machine, and open a terminal.  Enter, "ssh <UAV IP>", where "<UAV IP>" will be the address from 1a.  You will be prompted    for a password, which will be the RRC standard.  Once logged in, your terminal prompt should read, "user1@brix012", or        similar (whatever the name of your companion computer is to the right of the "@".
2. Launching image capture nodes
   - On your ground station computer, in the terminal from 1b., run the command, "cd catkin_ws".
   - Now run, "roslaunch beginner_tutorials bhg_pub_sub.launch"; you should be able to tab complete the package and launch        file.  If successful, you should see output about the names of the nodes you lauched, and ROS_INFO messages about            "Starting Publisher", and then "Number of cameras detected: 1" (if you see 0 instead of 1 here, you may have to restart      the launch).  You will then see streaming by, "recordData: 0", which is a message from the subscriber indicating that        the topic, "record" has the value "0" right now.  This means no images are being recorded.  
  
  NOTE: before recording images, or flying, you will want to ensure removal of the FLIR lens cap!
