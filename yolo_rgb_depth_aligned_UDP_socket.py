#!/usr/bin/env python3

import cv2
import numpy as np
import depthai as dai
import time
import argparse
import socket
import pickle
import zmq
import base64

parser = argparse.ArgumentParser(description='DepthAI object detection with depth showing')
parser.add_argument('--ip',
                help="This is ip address of the camera in LAN")


args = parser.parse_args()
_ip = args.ip

labelMap = [
    "chestnut",
]

##################
### UDP Socket ###
##################
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
data_obj = {
            'nboxes': 0,
            'xc': np.array([]),
            'yc': np.array([]),
            'closest_depth': np.array([])
}

def send_udp_socket(obj):
    dump_packets = pickle.dumps(obj, protocol=2)
    sock.sendto(dump_packets,("127.0.0.1", 8888))


#####################
### ZMQ streaming ###
#####################
PORT_DISPLAY = '5555'
context = zmq.Context()
footage_socket = context.socket(zmq.PUB)
footage_socket.bind('tcp://*:' + PORT_DISPLAY)
print("Video streaming on PORT: " +PORT_DISPLAY)


#######################
### Create pipeline ###
#######################
pipeline = dai.Pipeline()

#################################
## Define sources and outputs ###
#################################
camRgb = pipeline.create(dai.node.ColorCamera)
left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
detection_nn = pipeline.createYoloDetectionNetwork()


#######################
### Setup each pipe ###
#######################

#### RGB ####
camRgb.setPreviewKeepAspectRatio(False) ## this will squeeze the frame similar to original darknet yolo
camRgb.setPreviewSize(416, 416) ## For NN input
# camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setFps(30)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setIspScale(2, 3)
# For now, RGB needs fixed focus to properly align with depth.
# This value was used during calibration
# camRgb.initialControl.setManualFocus(0) # 130

#### left/right ####
left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P) # THE_720_P
left.setBoardSocket(dai.CameraBoardSocket.LEFT)
left.setFps(30)
right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)  # THE_720_P
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
right.setFps(30)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY) # HIGH_DENSITY
stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
# LR-check is required for depth alignment
stereo.setLeftRightCheck(True)
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

camRgb.initialControl.setAutoFocusMode(dai.RawCameraControl.AutoFocusMode.AUTO)
# camRgb.initialControl.setManualFocus(0)

####################
### detection NN ###
####################
# detection_nn.setBlobPath("/home/rasheed/depthai_test/models/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob") ## yolov3-tiny seems to work better than v4-tiny
# detection_nn.setBlobPath("/home/rasheed/depthai_test/models/yolov4-tiny_KNT_openvino_2021.4_6shave.blob")
detection_nn.setBlobPath("/home/nvidia/chestnut_picker_ros/models/yolov4-tiny-chestnut_openvino_2021.4_6shave.blob")
# detection_nn.setBlobPath(str(blobconverter.from_zoo(name='mobilenet-ssd', shaves=6)))
detection_nn.setConfidenceThreshold(0.5)
detection_nn.setNumClasses(1)
detection_nn.setCoordinateSize(4)
detection_nn.setAnchors(np.array([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319]))
# detection_nn.setAnchorMasks({"side26": np.array([1, 2, 3]), "side13": np.array([3, 4, 5])})
detection_nn.setAnchorMasks({"side26": np.array([0, 1, 2]), "side13": np.array([3, 4, 5])})
detection_nn.setIouThreshold(0.5) # 0.5
detection_nn.setNumInferenceThreads(2)
detection_nn.input.setBlocking(False)


###############
### Linking ###
###############

#### XLinkOut ####
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutNn = pipeline.create(dai.node.XLinkOut)
xoutDisplay = pipeline.create(dai.node.XLinkOut)


#### Stream name #####
xoutRgb.setStreamName("rgb")
xoutDepth.setStreamName("depth")
xoutNn.setStreamName("nn")
xoutDisplay.setStreamName("display")
# xoutSpatialData.setStreamName("spatialData")
# xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

camRgb.preview.link(detection_nn.input)
# camRgb.isp.link(xoutRgb.input)
detection_nn.passthrough.link(xoutRgb.input) # camRgb.preview.link(xoutRgb.input)
camRgb.video.link(xoutDisplay.input)
left.out.link(stereo.left)
right.out.link(stereo.right)
stereo.depth.link(xoutDepth.input) # stereo.disparity.link(xoutDepth.input)


detection_nn.out.link(xoutNn.input)



############################################
### Connect to device and start pipeline ###
############################################
if _ip is None:
    dai_statement = dai.Device(pipeline, usb2Mode=True)
    print("Not specified ip, checking on other device")
else:
    found, device_info = dai.Device.getDeviceByMxId(_ip)
    if not found:
        raise RuntimeError("Device not found")
    dai_statement = dai.Device(pipeline, device_info)
    print("Connecting with OAK-D-POE: {:}".format(device_info.desc.name))


with dai_statement as device:

    frame = None
    displayFrame = None

    queueRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    queueDisplay = device.getOutputQueue(name="display", maxSize=4, blocking=False)
    queueNn = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
    queueDepth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    startTime = time.monotonic()
    counter = 0
    fps = 0
    prev_dist = 0.0

    while True:
        inRgb = queueRgb.get()
        inDisplay = queueDisplay.get()
        inNn = queueNn.get()
        inDepth = queueDepth.get()

        if inRgb is not None:
            frame = inRgb.getCvFrame()

        if inDisplay is not None:
            displayFrame = inDisplay.getCvFrame()

        if inNn is not None:
            detections = inNn.detections
            # print(detections)

        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time


        depthFrame = inDepth.getFrame()

        # Optional, extend range 0..95 -> 0..255, for a better visualisation
        depthFrameShow = (depthFrame * 255. / 65535).astype(np.uint8)
        # Optional, apply false colorization
        # depthFrameShow = cv2.applyColorMap(depthFrameShow, cv2.COLORMAP_HOT)
        # depthFrame = np.ascontiguousarray(depthFrame)
        bbox = [None]*4
        if len(detections) != 0:
            nboxes_counter = 0
            # clear_obj()
            data_obj = {
                        'nboxes': 0,
                        'xc': np.array([]),
                        'yc': np.array([]),
                        'closest_depth': np.array([])
            }

            for detection in detections:
                if detection.label == 0:
                    # print("rgb", detection.xmin, detection.ymin, detection.xmax, detection.ymax)
                    # print("rgb_frame", frame.shape)
                    # bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                    depthFrame_height = depthFrame.shape[0]
                    depthFrame_width = depthFrame.shape[1]

                    height = displayFrame.shape[0]
                    width = displayFrame.shape[1]

                    if np.isinf(detection.xmin) or np.isinf(detection.ymin) or np.isinf(detection.xmax) or np.isinf(detection.ymax):
                        print("There is infinity here!")
                        print(detection.xmin, detection.ymin, detection.xmax, detection.ymax)
                        continue

                    xmin = int(detection.xmin*width)
                    ymin = int(detection.ymin*height)
                    xmax = int(detection.xmax*width)
                    ymax = int(detection.ymax*height)

                    xc = int((xmax+xmin)/2.0)
                    yc = int((ymax+ymin)/2.0)
                    bbox = [xmin, ymin, xmax, ymax]

                    data_obj['xc'] = np.append(data_obj['xc'], xc)
                    data_obj['yc'] = np.append(data_obj['yc'], yc)

                    box_scanline_array = depthFrame[yc-1:yc, xmin:xmax]
                    box_scanline_array = box_scanline_array.reshape(box_scanline_array.shape[1])
                    box_scanline_array = np.ma.masked_equal(box_scanline_array, 0.0, copy=False)
                    if len(box_scanline_array) != 0:
                        box_closest_dist = box_scanline_array.min()
                        box_closest_dist = box_closest_dist/1000.0 # in meter
                        box_closest_index = np.argmin(box_scanline_array) + xmin
                        ## to avoid numpy.ma.core.MaskedConstant 
                        if type(box_closest_dist) != np.float64:
                            box_closest_dist = 0.0
                    else:
                        box_closest_dist = 0.0 #prev_dist
                        # print("here")

                    data_obj['closest_depth'] = np.append(data_obj['closest_depth'], box_closest_dist)
                    # print(data_obj['closest_depth'])
                    prev_dist = box_closest_dist
                    ## Put bounding box
                    cv2.putText(displayFrame, labelMap[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 0, 0))
                    cv2.putText(displayFrame, f"{int(detection.confidence*100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 0, 0))
                    cv2.rectangle(displayFrame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0,0,255), 2)
                    ## Put closeset depth
                    # print(box_closest_dist, type(box_closest_dist))
                    cv2.putText(displayFrame, f"{box_closest_dist:.2f}m", (xc,yc), cv2.FONT_HERSHEY_DUPLEX, 1, (0,0,255), 2)
                    cv2.circle(displayFrame, (box_closest_index,yc), 5, (0,255,0), 2)
                    
                    # cv2.rectangle(depthFrameShow, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)
                    nboxes_counter += 1

            if nboxes_counter == 0:
                data_obj = {
                            'nboxes': 0,
                            'xc': np.array([]),
                            'yc': np.array([]),
                            'closest_depth': np.array([])
                }
            else:
                data_obj['nboxes'] = int(nboxes_counter)
        else:
            data_obj = {
                        'nboxes': 0,
                        'xc': np.array([]),
                        'yc': np.array([]),
                        'closest_depth': np.array([])
            }

        send_udp_socket(data_obj)
        data_obj = {
                    'nboxes': 0,
                    'xc': np.array([]),
                    'yc': np.array([]),
                    'closest_depth': np.array([])
        }

        # cv2.imshow("display", displayFrame)

        encoded, buffer = cv2.imencode('.jpg', cv2.resize(displayFrame, (640,360), interpolation=cv2.INTER_AREA))
        jpg_as_text = base64.b64encode(buffer)
        footage_socket.send(jpg_as_text)

        # if cv2.waitKey(1) == ord('q'):
        #     break

        # print(fps) ## around ~10fps
