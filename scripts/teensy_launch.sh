#!/bin/bash
# path from gui.py to erc24_ws confirm later for jetson
cd erc24_ws/ && 
export ROS_MASTER_URI=http://192.168.1.50:11311 &&
export ROS_HOSTNAME=192.168.1.50 &&
source devel/setup.bash &&
chmod 666 /dev/ttyACM* &&
roslaunch general_controls teensy.launch 