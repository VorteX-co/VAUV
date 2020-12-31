#!bin/bash

echo "#########################################################"
echo "#      Prepare machine to install VAUV packages         #"
echo "#                                                       #" 
echo "#########################################################"

#Create a workspace and clone all repos:

mkdir -p ~/ros2_eloquent/src
cd ~/ros2_eloquent
wget https://raw.githubusercontent.com/ros2/ros2/eloquent/ros2.repos
vcs import src < ros2.repos

#Install dependencies using rosdep

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro eloquent -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"

