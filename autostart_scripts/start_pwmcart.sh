#!/bin/bash

sleep 5

export DISPLAY=:0.0
export LOGFILE=/home/nvidia/chestnut_picker_ros/autostart_scripts/pwmcart_node.log

source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
source /home/nvidia/chestnut_picker_ros/ROBOT_CONFIG.txt

cd /home/nvidia/catkin_ws/src/jmoab-ros/

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting jmoab-ros jmoab-ros-pwmcart.py" >> $LOGFILE
		
		rosrun jmoab-ros jmoab-ros-pwmcart.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done