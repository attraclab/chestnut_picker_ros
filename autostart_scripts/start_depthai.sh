#!/bin/bash

sleep 5

export DISPLAY=:0.0
export LOGFILE=/home/nvidia/chestnut_picker_ros/autostart_scripts/depthai_node.log
export OPENBLAS_CORETYPE=ARMV8

#source /opt/ros/melodic/setup.bash
#source /home/nvidia/catkin_ws/devel/setup.bash
#source /home/nvidia/chestnut_picker_ros/ROBOT_CONFIG.txt

cd /home/nvidia/depthai_test

while true
do
	echo >>$LOGFILE
	echo "----------------------------------------------" >> $LOGFILE
	date >> $LOGFILE

	echo "Starting yolo_rgb_depth_aligned_UDP_socket.py" >> $LOGFILE
	
	python3 yolo_rgb_depth_aligned_UDP_socket.py >> $LOGFILE

	echo "program seems to have stopped" >> $LOGFILE

	date >> $LOGFILE
	sleep 1

done