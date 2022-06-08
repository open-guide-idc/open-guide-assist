#!/usr/bin/env python3
import json
import os
import tempfile
import platform
from pathlib import Path
import ransac
import open3d as o3d
# import RPi.GPIO as GPIO


import cv2
import depthai as dai
import numpy

# import socket

# HOST = '155.41.122.253'
# PORT = 2000
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((HOST,PORT))

#setup for new depthai version
# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended = False
out_depth = False
out_rectified = True
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = False
# Better handling for occlusions:
lr_check = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)
xoutRight = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("disparity")
xoutRight.setStreamName("right")


# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth.initialConfig.setConfidenceThreshold(245)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(lr_check)
depth.setExtendedDisparity(extended)
depth.setSubpixel(subpixel)

# Linking
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.depth.link(xout.input)
monoRight.out.link(xoutRight.input)


try:
    from projector_3d import PointCloudVisualizer
except ImportError as e:
    raise ImportError(f"\033[1;5;31mError occured when importing PCL projector: {e} \033[0m ")


#setup PI
# GPIO.setmode(GPIO.BOARD)
# #motor1
# GPIO.setup(8,GPIO.OUT)
# pwm2 = GPIO.PWM(8, 100)
# pwm2.start(0)

right = None
pcl_converter = None
vis = o3d.visualization.Visualizer()
vis.create_window()
isstarted = False
with dai.Device(pipeline) as device:
    qDepth = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)
    qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)

    try:
        from projector_3d import PointCloudVisualizer
    except ImportError as e:
        raise ImportError(f"\033[1;5;31mError occured when importing PCL projector: {e}. Try disabling the point cloud \033[0m ")
    calibData = device.readCalibration()
    right_intrinsic = numpy.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT, 640, 400))
    pcl_converter = PointCloudVisualizer(right_intrinsic, 640, 400)

    while True:
        
        #to get prism
        # was at the border work 0.7 to 2.2 with 1/4 z rotation 
        corners = numpy.asarray([[-0.5,-1.0,0.35],[0.5,-1.0,0.35],[0.5,1.0,0.35],[-0.5,1.0,0.35],[-0.5,-1.0,1.7],[0.5,-1.0,1.7],[0.5,1.0,1.7],[-0.5,1.0,1.7]])

        
        
        bounds = corners.astype("float64")
        bounds = o3d.utility.Vector3dVector(bounds)
        oriented_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(bounds)
        
        

        inRight = qRight.get()
        right = inRight.getFrame()

        # cv2.imshow("right", right)

        inDepth = qDepth.get()
        frame = inDepth.getFrame()
        median = cv2.medianBlur(frame, 5)
        median2 = cv2.medianBlur(median,5)
        # cv2.imshow("depth",median2)
     
        
        pcd = pcl_converter.rgbd_to_projection(median, right,False)
            
        #to get points within bounding box
        num_pts = oriented_bounding_box.get_point_indices_within_bounding_box(pcd.points)


        if not isstarted:
            vis.add_geometry(pcd)
            vis.add_geometry(oriented_bounding_box)
            isstarted = True       
                        
        else:
            vis.update_geometry(pcd)
            vis.update_geometry(oriented_bounding_box)
            vis.poll_events()
            vis.update_renderer()
        if len(num_pts)>5000:
            print("Obstacle")
            # s.send(bytes('1','utf-8'))
        else:
            print("Nothing")
            # s.send(bytes('0','utf-8'))

        if cv2.waitKey(1) == ord("q"):
            break

    if pcl_converter is not None:
        pcl_converter.close_window()
