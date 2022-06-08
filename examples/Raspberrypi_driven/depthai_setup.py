import cv2
import depthai as dai
import numpy as np
import time
from projector_3d import PointCloudVisualizer
import open3d as o3d

class DepthAi:
    def create_pipeline(self, nnBlobPath):
        yolo = 0
        if yolo:
            print("creating YOLO pipeline")
        else:
            print("creating MobileNet pipeline")

        syncNN = True

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
        camRgb = pipeline.create(dai.node.ColorCamera)
        if yolo:
            spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        else:
            spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)

        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutNN = pipeline.create(dai.node.XLinkOut)
        xoutBoundingBoxDepthMapping = pipeline.create(dai.node.XLinkOut)
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutRight = pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")
        xoutNN.setStreamName("detections")
        xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
        xoutDepth.setStreamName("depth")
        xoutRight.setStreamName("right")

        # Properties
        if yolo:
            camRgb.setPreviewSize(416, 416)
        else:
            camRgb.setPreviewSize(300, 300)

        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # setting node configs
        stereo.initialConfig.setConfidenceThreshold(255)

        spatialDetectionNetwork.setBlobPath(nnBlobPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.5)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        # Yolo specific parameters
        if yolo:
            spatialDetectionNetwork.setNumClasses(80)
            spatialDetectionNetwork.setCoordinateSize(4)
            spatialDetectionNetwork.setAnchors(np.array([10,14, 23,27, 37,58, 81,82, 135,169, 344,319]))
            spatialDetectionNetwork.setAnchorMasks({ "side26": np.array([1,2,3]), "side13": np.array([3,4,5]) })
            spatialDetectionNetwork.setIouThreshold(0.5)

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        monoRight.out.link(xoutRight.input)

        camRgb.preview.link(spatialDetectionNetwork.input)
        if syncNN:
            spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        else:
            camRgb.preview.link(xoutRgb.input)

        spatialDetectionNetwork.out.link(xoutNN.input)
        spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

        # ###################PC
        # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
        stereo.initialConfig.setConfidenceThreshold(245)
        # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(lr_check)
        stereo.setExtendedDisparity(extended)
        stereo.setSubpixel(subpixel)
        print("done creating")
        return pipeline
    
    def __init__(self, nnBlobPath):
        self.pipeline = self.create_pipeline(nnBlobPath)
        self.detections = []
        self.roiDatas = []
        self.right_intrinsic =[]
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window()

    def yolo_det(self):
        with dai.Device() as device:
            device.startPipeline(self.pipeline)
            # Output queues will be used to get the rgb frames and nn data from the outputs defined above
            previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
            xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
            depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

            # #################PC
            qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)
            calibData = device.readCalibration()
            self.right_intrinsic = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT, 640, 400))
            
            # #################PC

            startTime = time.monotonic()
            counter = 0
            fps = 0
            color = (255, 255, 255)
            detcount = 0

            while True:
                inPreview = previewQueue.get()
                inDet = detectionNNQueue.get()
                depth = depthQueue.get()
                right = qRight.get()
                pcFrame = right.getFrame()

                frame = inPreview.getCvFrame()
                depthFrame = depth.getFrame()
                depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                depthFrameColor = cv2.equalizeHist(depthFrameColor)
                depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

                self.detections = inDet.detections

                if len(self.detections) != 0:
                    boundingBoxMapping = xoutBoundingBoxDepthMappingQueue.get()
                    self.roiDatas = boundingBoxMapping.getConfigData()

                    
                counter+=1
                current_time = time.monotonic()
                if (current_time - startTime) > 1 :
                    fps = counter / (current_time - startTime)
                    counter = 0
                    startTime = current_time

                
                yield frame, depthFrameColor, fps, depthFrame, pcFrame

    def pc(self):
        with dai.Device(self.pipeline, usb2Mode=True) as device:
            # device.startPipeline(self.pipeline)
            depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

            qRight = device.getOutputQueue(name="right", maxSize=4, blocking=False)
            calibData = device.readCalibration()
            self.right_intrinsic = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT, 640, 400))

            while True:
                depth = depthQueue.get()
                right = qRight.get()
                pcFrame = right.getFrame()
                depthFrame = depth.getFrame()                
                yield depthFrame, pcFrame


    def __del__(self):
        del self.pipeline