#!/bin/bash
# BHG install script. Start from a clean install of Ubuntu 18.04.

# Add the google chrome repository for chrome install.
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
echo 'deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main' | sudo tee /etc/apt/sources.list.d/google-chrome.list

# Update the package list and os
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y dist-upgrade
sudo timedatectl set-timezone America/New_York

# Install google chrome
sudo apt-get -y install google-chrome-stable

echo "========= INSTALLING ROS =========="
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get -y install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential meld minicom ant git gitk openssh-server terminator gparted git-core python-argparse python-wstool python-vcstools build-essential gedit-plugins dkms python-rosdep gedit-plugins libcanberra-gtk-module
sudo rosdep init
rosdep update

sudo adduser user1 dialout
git config --global user.email "user1@nucXX.com"
git config --global user.name "User1 NucXX"
git config --global push.default simple

cd
mkdir -p catkin_ws/src
source ~/.bashrc
cd catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
rospack profile

cd src
git clone https://github.com/westpoint-robotics/usma_bhg.git
echo "source ~/catkin_ws/src/usma_bhg/resources/.bhg_aliases.sh" >> ~/.bashrc

sudo apt-get -y install ros-melodic-mavros
sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh

# Xenics install 
cd ~/Downloads
sudo apt-get -y install libusb-0.1-4

echo "===== Installing Xenics SDK ====="
wget http://support.xenics.com/Support/Linux_SDK_27.zip
mkdir xenethSDK
cd xenethSDK
unzip ../Linux_SDK_27.zip
sudo dpkg -i xeneth_2.7.0-181_amd64.deb

echo "===== Installing Wired connection to Gobi camera ====="
cd 
sudo cp ~/catkin_ws/src/usma_bhg/resources/Gobi /etc/NetworkManager/system-connections/Gobi
sudo chown root:root /etc/NetworkManager/system-connections/Gobi

echo "===== Installing Mavproxy dependencies ====="
sudo apt-get -y autoremove
sudo apt-get -y install python3-dev python3-opencv python3-wxgtk4.0 libxml2-dev python3-pip python3-matplotlib python3-lxml python-pip python3-pip
sudo pip3 install future
sudo pip3 install pymavlink
sudo pip3 install mavproxy

echo "===== Installing Spinnaker Delendencies ====="
sudo apt-get -y install libavcodec57 libavformat57 libswscale4 libswresample2 libavutil55 libusb-1.0-0 libgtkmm-2.4-dev
sudo apt-get -y install ros-melodic-camera-info-manager ros-melodic-dynamic-reconfigure
sudo apt-get -y install python-pip python3-pip
sudo apt-get install python-pip python3-pip  
sudo python -m pip install --upgrade numpy matplotlib  
python2 -m pip install enum34  

## 12. Install Arduino IDE
cd ~/Downloads  
wget https://downloads.arduino.cc/arduino-1.8.13-linux64.tar.xz
tar -xf arduino-1.8.13-linux64.tar.xz  
cd arduino-1.8.13/  
sudo ./install.sh  
sudo apt-get -y install ros-melodic-rosserial-arduino ros-melodic-rosserial
cd ~/Arduino/libraries/
rm -rf ros_lib
rospack profile
rosrun rosserial_arduino make_libraries.py .
chmod +x ~/Desktop/arduino-arduinoide.desktop 
gio set ~/Desktop/arduino-arduinoide.desktop "metadata::trusted" yes


echo '========================================================================='
echo '========================================================================='
echo '========================================================================='
echo '-'
echo 'From os-setup/ubuntu18_bhg.md, do the following steps manually 4, 5, 6, 7, 8, 10'
echo 'A reboot is required to replace the running dbus-daemon.'
echo '-'
echo '========================================================================='
echo '========================================================================='
echo '========================================================================='
echo "!! Successfully Ran to completion !!"
