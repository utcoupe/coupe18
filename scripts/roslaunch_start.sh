#!/bin/bash
# DO NOT DELETE : USED FOR INIT.D SERVICE
source /home/utcoupe/.bashrc
source /opt/ros/kinetic/setup.sh
source /home/utcoupe/coupe18/ros_ws/devel/setup.sh

/usr/bin/python /opt/ros/kinetic/bin/roslaunch memory_definitions general.launch robot:=$1 &
sleep 2

