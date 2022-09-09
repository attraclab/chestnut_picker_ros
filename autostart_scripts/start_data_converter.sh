#!/bin/bash

sleep 5

export DISPLAY=:0.0
export LOGFILE=/home/nvidia/chestnut_picker_ros/autostart_scripts/data_conveter_node.log

source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
source /home/nvidia/chestnut_picker_ros/ROBOT_CONFIG.txt

cd /home/nvidia/chestnut_picker_ros

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting recvUdp_to_rostopic.py" >> $LOGFILE
		
		python -u recvUdp_to_rostopic.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done