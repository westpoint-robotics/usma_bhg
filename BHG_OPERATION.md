# This document will show you how to:
1. Connect your ground station computer to your UAV companion computer network.
2. Launch image capturing ROS nodes.
3. Start and stop recording from FLIR and Xeneth Gobi cameras.
4. Mount the FLIR camera (Gobi follows suit).
------------------------------------------------------
## 1. Ground Station to Companion Computer Network Connection
   
- The companion computer will by default boot and log into the OS.
- After the OS loads it will automatically create a wireless access point with an SSID of 'nuc##' where ## is the NUC number.
- Once the access point becomes visible connect to it and provide the password. 
- The IP address for the companion will always be 10.42.0.1.
- To find the IP address of the ground station use the below command:  
    `ip a`  
- To open a remote shell on the companion computer use SSH:  
    `ssh user1@10.42.0.1`
  
------------------------------------------------------   
## 2. Launching bhg software

- On your ground station computer open a remote shell on the companion computer:  
    `ssh user1@10.42.0.1`   
- In the remote shell start the software:  
     `roslaunch usma_bhg master.launch`   
    or use the alias:  
    `bhg_launch`  
- If successful, you should see output about the names of the nodes you lauched, and ROS_INFO messages about "Starting Publisher"
- Whatch for any red error text to stream by. If it does investigate it. 
- If everything works as expected there should be no red text, however is common for the Pixhawk to generate red text if it is failing PreArm checks. This is not a failure of nodes to start and no action in the terminal is required. This message look like this:
<pre>
[ERROR] [1593401054.831960641]: FCU: PreArm: fence requires position
[ERROR] [1593401054.832934034]: FCU: PreArm: Throttle below Failsafe
[ERROR] [1593401054.832957058]: FCU: PreArm: Waiting for 3D fix
</pre>
### Check if both cameras started completed
- If both cameras started completly you will see a message for the Gobi and a message for the FLIR appear about every 30 seconds stating the number of pictures taken and the number saved. They look this:
<pre>
[ INFO] [1593401045.704584516]: ***** GOBI:  Grabbed Image 183, and saved 0
[INFO] [1593401061.708994]:     ***** FLIR:  Grabbed Image 602, and saved 0
</pre>
- Typically the FLIR camera starts operating before the GOBI and will have a higher image count.
### Trigger Signal
- Both cameras recieve an external trigger signal to synchronize the pictures. Tests have shown that aproximately 70% of the images are taken within 1ms of each other and greater than 90% of the images are taken within 2ms of each other.
- The trigger signal is sent at 20hz with a 50% duty cycle using an Arduino Trinket. 
- The frequency can be changed by modifying the 'hz' variable in the Arduino source code. 
- The Arduino source code can be found in this repo in the 'resoruces' sub-directory. 
------------------------------------------------------
## 3. Starting and Stopping Image Recording

- All commands in this section are ran in a remote shell to the companion computer.
- Both cameras take pictures upon the start of the software and publish them as rostopics in the camera namespace.
    - FLIR Topic is: `/camera/image_color`
    - GOBI Topic is: `/camera/gobi_image`
- The images are saved only after the recieving a `/record` topic set to true and stopped when the topic is published with a value of false.
- The `/record` topic is published as true automatically when the pixhawk enters 'AUTO' mode. It stops when the pixhawk leaves 'AUTO' mode.
- The `/record` topic is published as true when the switch on channel 9 is toggled back toward the user and false when toggled away from the user.
- The `/record` topic can also be published by command line using the alias or typing out the command.
    - Alias:
        - `bhg_start`
        - `bhg_stop`
    - Command:
        - `rostopic pub -1 /record std_msgs/Bool True`
        - `rostopic pub -1 /record std_msgs/Bool False` 
        
## NOTE: Before recording images, or flying: REMOVE THE LENS CAPS!
------------------------------------------------------   
## 4. Mounting the FLIR
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
# BELOW HERE NEEDS UPDATING
- Cameras are now fully operational with running "master.launch". Previously Gobi would not take pictures until record was 'true'. Now both Gobi and Flir publish pictures upon starting master.luanch. They both listen for the topic "/record" to be true to start saving images or false to stop saving images.
- master.launch and img_capture.launch have a set of bool args at the top of the file. These allow disabling or enabling the running of different components fo the system. This eliminates the copy and paste of comments to achieve this.
- A new node call record.mux is created. This node listens for any change in th "F" switch on the DX9. If the switch changes and into any position but position "0" it publishes record = true, if it is changed to the "0" position it publishes record = false. This node only publishes once upon a switch change.
- It is recommended to use the below commands to start and stop record from command line. They will puclish one time only. This is all that is required to make the change.  
`rostopic pub -1 /record std_msgs/Bool True`  
`rostopic pub -1 /record std_msgs/Bool False`   

## 5. Disk cleanup and maintenance tasks added:
### The file dir_setup.py uses the file dir_cleanup.py to do the following Data directory tasks:
- It removes any folders with less then 5 files in it. It is assumed these folders are unused data directories
- If the symlink is broken by this removal it recreates to point to the last used directory.
- It moves tha Bagfile to the last used folder.
- All this is done after rosshutdown is triggered.

# TODO
- Automatically copy the flight log into the data directory.
- Use Gui to create log entries for flight, things like direction of camera, angle camera, alti, flight plan

