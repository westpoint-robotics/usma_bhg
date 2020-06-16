# usma_bhg
### TODOs
- Migrate FLIR img_save to a nodelet
- Develop scripts to inform us if system is running as expected:
1. Did the Flir driver start successfully
2. Is the CSV being filled out properly
3. Did the Gobi driver start correctly
4. Are pictures being saved to the correct director
5. Did we leave a lense cap on
6. Warn if disk drive is filling up
7. Develop a script that makes a symlink from the newest data folder to a folder called latest, much like ros logs. Done 17FEB
8. Develop CSV building capability for Gobi

### ROS nodes, launch files, etc. for Bloodhound Gang project (USMA)
This document will show you how to:  
1. Set up a companion computer (NUC, Brix, or similar) to run Ubuntu 16.04 or 18.04.
2. Install ROS for one of the above versions of Ubuntu.
3. Install the Xenics SDK for use with Xeneth Gobi IR cameras.
4. Install the Spinnaker SDK for use with FLIR cameras.
5. Install MAVROS.
6. Install usma_bhg, which includes Python and C++ code necessary for operating the FLIR and Xeneth Gobi cameras respectively.
7. Edit bashrc.

### Documentation on how to operate the BHG system can be found in "BHG_OPERATION.md".
------------------------------------------------------
# Ubuntu: Image Restore or Installation
### a. Option 1: Full Sysem Image
- Use Clonezilla to image the development Companion Computer. You will be taking the steps to [Restore](https://clonezilla.org/clonezilla-live-doc.php) an image, but it will be device-device, rather than device-image. 
   
### b. Option 2: From a clean install of Ubuntu
- Follow the instructions at: https://github.com/westpoint-robotics/os-setup/blob/master/ubuntu18_bhg.md
- Follow the instructions in part "c. Option3" below. 

### c. Option 3: From a computer with ROS already installed
- Use Clonezilla to burn an image with Ubuntu 18.04.  You will be taking the steps to [Restore](https://clonezilla.org/clonezilla-live-doc.php) an image, but it will be device-device, rather than device-image.  

### d. Manual Install
#1. ROS Install 
- For Ubuntu 16.04, follow [these steps](http://wiki.ros.org/kinetic/Installation/Ubuntu).
- For Ubuntu 18.04, follow [these steps](http://wiki.ros.org/melodic/Installation/Ubuntu).

#2. Install MavROS  
- Install the ROS package:  
`sudo apt-get install ros-melodic-mavros`  
- Use this script to download Geoid Model datasets for Mavros:  
`sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh`  

#3. Xenics SDK Install
- Install [Xenics SDK](http://support.xenics.com/Support/Linux_SDK_27.zip) and dependencies.  Follow directions for Ubuntu 18.04 for AMD 64 architecture.
- Extract the files and cd into the created directory and run the below commands.
&nbsp;&nbsp;&nbsp;`sudo apt-get install libusb-0.1-4`
&nbsp;&nbsp;&nbsp;`sudo dpkg -i xeneth_2.7.0-181_amd64.deb`
- Add ip address into your local network
	- Go to Settings > Network > Wired, and click (+) button to add Gobi camera
	- In `Identity` tab, put the camera name (ex. `gobi`)
	- In `IPv4` tab, select `Manual` and enter the following information
	- Address: `169.254.107.22`
	- Netmask: `255.255.0.0`

#4. Install Spinnaker SDK and dependancies:  
- Download the SDK from https://flir.app.boxcn.net/v/SpinnakerSDK/folder/69083919457
- Uncompress the folders to get the folder spinnaker-2.0.0.109-Ubuntu18.04-amd64-pkg/spinnaker-2.0.0.146-amd64  
`sudo apt-get install libavcodec57 libavformat57 libswscale4 libswresample2 libavutil55 libusb-1.0-0 libgtkmm-2.4-dev`  
`sudo sh install_spinnaker.sh`  
- During install process, add a new member to `flirimaging`
<pre>
Would you like to add a udev entry to allow access to USB hardware? If a udev entry is not added, your cameras may only be accessible by running Spinnaker as sudo.  
[Y/n] $ <b>y</b>  
Adding new members to usergroup flirimaging… To add a new member please enter username (or hit Enter to continue):  
<b>Type your computer name (ex. $ user1)</b>  
Writing the udev rules file… Do you want to restart the udev daemon?  
[Y/n] $ <b>y</b>  
Would you like to set USB-FS memory size to 1000 MB at startup (via /etc/rc.local)?  
[Y/n] $ <b>y</b>  
Would you like to make a difference by participating in the Spinnaker feedback program?  
[Y/n] $ <b>n</b>  
</pre>

- If you do not already have a catkin_ws then create one  
`mkdir -p ~/catkin_ws/src`  
`cd ~/catkin_ws/`  
`catkin_make`  
`echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`  
`source ~/.bashrc`  

- Clone dependencies and ROS FLIR camera drivers into catkin_ws:  
`sudo apt-get install ros-melodic-camera-info-manager ros-melodic-dynamic-reconfigure`  
`cd ~/catkin_ws/src/`  
`git clone https://github.com/westpoint-robotics/flir_camera_driver.git`  
`cd ..`  
`catkin_make`  

#5. Install PySpin
- Download the package (spinnaker_python-2.0.0.146-cp27-cp27mu-linux_x86_64.tar.gz) that corresponds to the python version (2.7) and Spinnaker version (spinnaker-2.0.0.146) from https://flir.app.boxcn.net/v/SpinnakerSDK/folder/74728781416
- Uncompress the folders and run  
`sudo python -m pip install --upgrade numpy matplotlib`  
`python2.7 -m pip install enum34`  
- Move to the location in which the whl file is located, and run  
`sudo python -m pip install spinnaker_python-2.0.0.146-cp27-cp27mu-linux_x86_64.whl`  
- The examples are located in the Examples folder of the extracted tarball. Run with:  
ex. `python Examples/Python3/DeviceEvents.py`  

#6. Install usma_bhg package  
&nbsp;&nbsp;&nbsp;`cd ~/catkin_ws/src/`  
&nbsp;&nbsp;&nbsp;`git clone https://github.com/westpoint-robotics/usma_bhg.git`  

#7. Create a hotspot access piont
- Open the network manager connection editor:  
`nm-connection-editor`
- Choose the plus sign to add a connections
- Choose Wi-Fi for "Connection Type" and click "Create..."
- On the Wi-Fi tab:
    - Set "Mode:" to Hotspot
    - Set "SSID:" to NUC42
- On the Wi-Fi Security Tab
    - Set "Security:" to WPA & WPA2 Personal
    - Set the password: nuc42access
- On the General Tab
    - Make sure 'Automatically connect ...' is checked

#8 Edit bashrc:  
&nbsp;&nbsp;&nbsp;a. `cd /home/user1/.bashrc`  
&nbsp;&nbsp;&nbsp;b. `gedit .bashrc`  
&nbsp;&nbsp;&nbsp;c. At the bottom of the .bashrc file, insert the lines (if not already added):  
<pre>
source /opt/ros/melodic/setup.bash  
source /home/user1/catkin_ws/devel/setup.bash  
# export ROS_MASTER_URI=http://[NUC_IP]:11311  
# export ROS_IP=[NUC_IP]  
<\pre>

&nbsp;&nbsp;&nbsp;d. Save these changes.  

# Additonal installs for MAVProxy to work:
`sudo apt-get update`    #Update the list of packages in the software center  
`sudo apt-get install python3-dev python3-opencv python3-wxgtk3.0 libxml2-dev python3-pip python3-matplotlib python3-lxml python-pip python3-pip`  
`sudo pip3 install future`  
`sudo pip3 install pymavlink`  
`sudo pip3 install mavproxy`  

# FTDI wiring.  
- The FTDI adapter did not work as wired. I had to switch rx and tx wires for this to work with ardupilot. Now the rx pin on the pixhawk is wired to the rx pin on the FT232 and the same with the tx pin.

# Install the latest QGroundControl on Ubuntu 18.04
- Install dependencies:
`sudo apt-get install libqt5serialport5 qml-module-qtquick2 qtdeclarative5-qtquick2-plugin gstreamer1.0-plugins-bad gstreamer1.0-libav`
- Remove modemmanager:
`sudo apt-get remove modemmanager -y`
- Download: https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
- `chmod +x ./QGroundControl.AppImage`
- Download: https://firmware.ardupilot.org/Tools/APMPlanner/apm_planner_2.0.26_xenial64.deb
- Install it with:
`sudo dpkg -i apm_planner_2.0.26_xenial64.deb`

# Mavros not working Totally
- Mavros was publishing diagnostits but no other topics. Running the below command fixed this:
`rosservice call /mavros/set_stream_rate 0 10 1`
- This is now incorporated in the ardupilot.launch node and appears to work from there.

# Web server and ROS    
- `sudo apt-get install python-tornado python-bson ros-melodic-rosbridge-suite  ros-melodic-roswww`
- `roslaunch rosbridge_server rosbridge_websocket.launch`

## Experimental below here:
- curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
- sudo apt-get install -y nodejs
- node --version
- npm --version
solves problem of eventemitter2 missing on local computer

### Solve the nagging error of: Failed to load module "canberra-gtk-module" 
- `sudo apt-get install libcanberra-gtk-module`

### Missing dependencies
- `sudo apt-get install ros-melodic-libsensors-monitor`
- `sudo -H pip3 install --upgrae pip`
- `sudo -H pip3 install matplotlib`
- `sudo -H pip3 install jupyter`
- `sudo -H pip3 install pymavlink`
- `sudo -H pip3 install pandas`
`



