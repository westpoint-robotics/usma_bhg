This document will show you how to:
1. Connect your ground station computer to your UAV companion computer.
2. Launch image capturing ROS nodes.
3. Start and stop recording from FLIR and Xeneth Gobi cameras.
4. Mount the FLIR camera (Gobi follows suit).
------------------------------------------------------
# 1. Ground Station to Companion Computer Connection
   
a. Ascertaining the Companion IP Address
   
   If you will be operating with access to a WREN connection on post at West Point, the Companion IP Address will be
   10.212.149.243. 
   Otherwise, if you are operating for the first time at a new location do the following.
   - Connect a USB hub to a free USB port on the Companion.
   - Connect a USB keyboard and mouse to this hub.
   - Connect one of the trailer monitors with an HDMI cable.
   - Unplug the power cable running from the Companion to the power distribution board on the UAV, and plug in the Companion adapter
     instead (plug the other end into a wall outlet).
   - Turn on the Companion using the power button on the underside of the computer at the aft, port corner with respect to the
     front of the UAV (blue motors).  Log in using standard RRC credentials.
   - Open a terminal and use the command:  
   
      `ip a`  
   
     You should see a line similar to, "link/ether 00:c0:ca:97:df:c1 brd
     ff:ff:ff:ff:ff:ff".  Below that, should read "inet" and then a number, for example starting with 10, or 198, or 
     similar.  This is the start of the IP address you want.  Record this in a project/mission notebook.
   
b. Connecting from the Ground Station
   
   - Your ground station computer should be a laptop or desktop running Ubuntu 16.04, or 18.04.  Turn on and log into this machine, and open a terminal.  Enter:  
   
     `ssh [UAV IP]`  
   
     where [UAV IP] will be the address from 1a.  You will be prompted for a password, which will be the RRC standard.  Once 
    logged in, your terminal prompt should read, "user1@brix012", or similar (whatever the name of your companion computer is 
    to the right of the "@".
------------------------------------------------------   
# 2. Launching image capture nodes
   - On your ground station computer, in the terminal from 1b., run the command:  
                            
     `roslaunch usma_bhg master.launch`   
     
     you should be able to tab complete the package and launch file.  If successful, you should see output about the names of 
     the nodes you lauched, and ROS_INFO messages about "Starting Publisher", and then "Number of cameras detected: 1" (if 
     you see 0 instead of 1 here, you may have to restart the launch).  You will then see streaming by, "recordData: 0", 
     which is a message from the subscriber indicating that the topic, "record" has the value "0" right now.  This means no 
     images are being recorded.
  
## NOTE: before recording images, or flying, you will want to ensure removal of the FLIR lens cap!
------------------------------------------------------
# 3. Starting and Stopping Image Recording
   - To start recording images, bring up a new terminal.  Here you will use ROS to publish a message, which will tell any subscribing camera scripts to start acquiring and saving images.  Enter: `rostopic pub /record /std_msgs/Bool True`  Use Ctrl-C to get a new prompt; this will not stop the publishing of the "record" topic.
   - To stop recording, Enter: `rostopic pub /record /std_msgs/Bool False`.  You can toggle back and 
     forth using these two buttons as much as you want.
------------------------------------------------------   
# 4. Mounting the FLIR
   - Locate the mounting bracket, screws, and washers for mounting.
     
     <img src="pictures/Bracket_to_Battery-Cage.jpg"
     alt="screws and washers"
     class="center"
     width="300px"/>
     <img src="pictures/FLIR_in_Bracket.jpg"
     alt="bracket"
     class="center"
     width="300px"/>
     
   - Insert the camera into the bracket with the "FLIR" logo on the side upsidedown, and screw into place as shown.
     
     <img src="pictures/FLIR_in_Bracket2.jpg"
     alt="bracket insall"
     class="center"
     width="300px"/>
     
   - Plug in USB3 cable into the back of the FLIR, and plug the other end into a spare port (not a hub!) on the companion 
   computer. 
   - Place the bracket between the rails on the battery cage at the bottom of the UAV.  Position the bracket such that the 
     center of the FLIR's lens is as close to centered as possible.  The camera should point 45 degrees downward and toward 
     the front of the craft (blue motors).
   - Screw the bracket into place on the rails.  Beware that the nuts in the top of the bracket may fall out if too much 
   force is applied during installation.
     
     <img src="pictures/FLIR_Screws_Washers.jpg"
     alt="screws and washers install"
     class="center"
     width="300px"/>
     <img src="pictures/FLIR_Installed.jpg"
     alt="fully installed"
     class = "center"
     width="600px"/>
     
------------------------------------------------------   
# 4. New Additions May2020
- Cameras are now fully operational with running "master.launch". Previously Gobi would not take pictures until record was 'true'. Now both Gobi and Flir publish pictures upon starting master.luanch. They both listen for the topic "/record" to be true to start saving images or false to stop saving images.
- master.launch and img_capture.launch have a set of bool args at the top of the file. These allow disabling or enabling the running of different components fo the system. This eliminates the copy and paste of comments to achieve this.
- A new node call record.mux is created. This node listens for any change in th "F" switch on the DX9. If the switch changes and into any position but position "0" it publishes record = true, if it is changed to the "0" position it publishes record = false. This node only publishes once upon a switch change.
- It is recommended to use the below commands to start and stop record from command line. They will puclish one time only. This is all that is required to make the change.  
`rostopic pub -1 /record std_msgs/Bool True`  
`rostopic pub -1 /record std_msgs/Bool False`   

# 5. Disk cleanup and maintenance tasks added:
## The file dir_setup.py uses the file dir_cleanup.py to do the following Data directory tasks:
- It removes any folders with less then 5 files in it. It is assumed these folders are unused data directories
- If the symlink is broken by this removal it recreates to point to the last used directory.
- It moves tha Bagfile to the last used folder.
- All this is done after rosshutdown is triggered.

