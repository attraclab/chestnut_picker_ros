#!/bin/bash

# sleep 5

export DISPLAY=:0.0
export LOGFILE=/home/nvidia/chestnut_picker_ros/autostart_scripts/tensorrt_node.log

source /opt/ros/melodic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
source /home/nvidia/chestnut_picker_ros/ROBOT_CONFIG.txt

cd /home/nvidia/catkin_ws/src/tensorrt_demos_ros/src

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting trt_yolo_ros.py" >> $LOGFILE
		
		python -u trt_yolo_ros.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done