#!/bin/bash

cd /home/nvidia/chestnut_picker_ros/autostart_scripts

bash start_jmoab_launch.sh & bash start_chestnut_picker_demo.sh & bash start_tensorrt.sh & 