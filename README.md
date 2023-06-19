# Chestnut-Picker with OAK-D camera

This project has three main components,

1. cart control; this is to move or to stop the cart according to AI of chestnut detection. Basically, the cart will be moving forward and will stop when chestnut is detected.

2. chestnut detection; this is an object detection trained on Darket YOLO-tiny v4. The trained model is converted to blob file which will be used with OAK-D camera. The output from this are location of chestnuts in the frame and depth from camera to the object.

3. robot manipulator; this is the robot arm which will be moving to the grab the chestnut when the detection gives target location and depth. The cart will stop at this moment and the arm will try grabbing all of the chestnuts in the frame until finish, and the cart starts to move again.

## Hardware

- Jetson Nano
- U2D2 (Dynamixel's RS485-USB converter)
- Robot Manipulator
  - XM540-W270-R 3pcs for linkage
  - XM430-W350-R 1pc for gripper
  - aluminum + carbon parts
- OAK-D (lite) camera
- GL-iNet WiFi router
- 15V DC-DC regulator
- 5V DC-DC regulator
- Futaba T6K radio transmitter and receiver
- AT_JMOAB06
- Crawler wheels
- Makita battery

## System

The software is running on ROS (Robot-Operating-System), because it's easier to develop the robot when there are many data has to pass from one program to another. Please check on below diagram.

![](image/software_diagram.jpg)

## AT_JMOAB

AT_JMOAB or JMOAB for short is an interface board for Jetson Nano to control the AT-Cart motor or PWM-Cart. In this case we are using PWM-cart which means the motors have ESC which needs PWM signal to drive forward-backward. For more detail about this board please check on [this repo](https://github.com/rasheeddo/jmoab-ros). I made a ROS package for this to get data or control things on JMOAB by ROS topics.

The `pwmcart` node is running to subscribe on `cmd_vel` topic, so it's waiting for `chestnut_picker_demo` node to send command to it.

## Robot Manipulator

The robot manipulator of this configuration is known as 3DOF parallel linkage or "Delta Robot", it's mostly used in manufacturing process for speed-picking. I have made another repo to explain about the working space and dimension of this robot, please check on [here](https://github.com/rasheeddo/The-Delta-Robot-MK2-Cpp). The code in that repo is C++, but I adopted that code to Python as in this repo, you can check it on `DeltaRobot.py` .

There is a possibility that we want to change speed of the cart, the chestnut bucket position got changed or the speed of the arm has to be changed, then we use `DeltaRobotParams.yaml` file as parameters file. When we run `chestnut_picker_demo`, we need to pass the argument as `python chestnut_picker_demo.py --params_file DeltaRobotParams.yaml`.

## OAK-D chestnut detection

As explained above, we've trained the yolov4-tiny to detect only chestnut. Currently, the model consider consider all of the chestnut with and without shell as same object. For example, the image below is a sample data of training.

![](image/sample_data.jpg)

Jetson Nano is running with Ubuntu 18.04, so we can only use ROS melodic distro (python2). But the SDK of Depthai for OAK-D needs python3, so I made some trick as `yolo_rgb_depth_aligned_UDP_socket` will be running by python3 and it will send data out to UDP port 8888 to `recvUdp_to_rostopic`, and this `recvUdp_to_rostopic` which will be running by python2 will convert the JSON data to ROS topic as `oakd/detection`.

## Operation

We can drive the cart manually by using Futaba transmitter and set the Ch5 to `Manual` mode, the left stick up-down is to drive forward-backward direction, and the right stick left-right is to turn the cart left-right direction. If we put the Ch5 to `AUTO` mode, the cart will start to move by itself with constant speed. And on Ch6, we can enable or disable the robot arm movement. In case we want to drive the cart manually, we may need to disable the arm not to go pick up something during we drive the cart.

![](image/futaba_tx_rx.png)