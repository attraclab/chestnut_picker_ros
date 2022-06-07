#!/bin/bash

cd /home/nvidia/chestnut_picker_ros/autostart_scripts

bash start_roscore.sh &\
bash start_depthai.sh & bash start_data_converter.sh &\
bash start_pwmcart.sh &\
bash start_chestnut_picker_demo.sh & 