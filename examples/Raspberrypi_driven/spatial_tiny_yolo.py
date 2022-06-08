from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
from datetime import datetime
import open3d as o3d

from subprocess import Popen
from depthai_setup import DepthAi
from projector_3d import PointCloudVisualizer
from collections import deque
import speech_recognition as sr

#from gtts import *
import gtts 
from playsound import playsound
import os
from google.cloud import texttospeech



rpi = 0
start=datetime.now()

cmd_start='gtts-cli '
cmd_mid=' --output '
cmd_end='message.mp3'
scan_end ='scan.mp3'
opensound = ""
modeswitchpin=1
mode = 1

if (rpi==1):
    # setup socket
    import socket
    import RPi.GPIO as GPIO

    HOST = '155.41.122.253'
    PORT = 2000
    #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #s.connect((HOST,PORT))

    #setup PI
    GPIO.setmode(GPIO.BOARD)
    modeswitchpin=3
    GPIO.setup(modeswitchpin, GPIO.IN)
  


class Main:
    depthai_class = DepthAi

    def __init__(self):
        self.nnBlobPath = str((Path(__file__).parent / Path('../models/2classes_model.blob')).resolve().absolute())
        self.depthai = self.depthai_class(self.nnBlobPath)
        self.labelMap = ["", "door", "handle"]
        # [
        #     "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
        #     "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
        #     "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
        #     "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
        #     "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
        #     "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
        #     "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
        #     "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
        #     "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
        #     "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
        #     "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
        #     "teddy bear",     "hair drier", "toothbrush"
        # ]
        self.isstarted = False
        # self.pcl_converter = None
        self.target = "handle"
        self.confq = deque(maxlen=30)
        self.lastsaid = [0,0,0]
        self.epsDist = 3


    def run_yolo_pc(self):
        color = (255, 255, 255)
        speed=' -s' + '160'
        pcl_converter = None
        #vis = o3d.visualization.Visualizer()
        #vis.create_window()
        for frame, depthFrameColor, fps, depthFrame, pcFrame in self.depthai.yolo_det():
            for roiData in self.depthai.roiDatas:
                roi = roiData.roi
                roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                topLeft = roi.topLeft()
                bottomRight = roi.bottomRight()
                xmin = int(topLeft.x)
                ymin = int(topLeft.y)
                xmax = int(bottomRight.x)
                ymax = int(bottomRight.y)

                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)


            # If the frame is available, draw bounding boxes on it and show the frame
            height = frame.shape[0]
            width  = frame.shape[1]
            maxconf = 0
            maxconfdepth = 0
            maxconfx = 0
            medvals = [0,0,0]
            label=""
            for detection in self.depthai.detections:
 
                x1 = int(detection.xmin * width)
                x2 = int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)
                try:
                    label = self.labelMap[detection.label]

                    #check if a handle is detected
                    if (label==self.target):

                        #save highest confidence value and corresponding depth
                        if detection.confidence>maxconf:
                            maxconf = detection.confidence
                            maxconfdepth = detection.spatialCoordinates.z
                            maxconfx = detection.spatialCoordinates.x

                        tempq = list(self.confq)
                        medvals = np.median(tempq, axis=0)
                    
                except:
                    label = detection.label
                cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

            #push highest confidence & corresponding depth to queue
            self.confq.append([maxconf, maxconfdepth, maxconfx])
            # try:
            # print(self.lastsaid, medvals)
            distdiff = abs(round(self.lastsaid[1]/1000*3.28,1)-round(medvals[1]/1000*3.28,1))
            # print(round(self.lastsaid[1]/1000*3.28,1),round(medvals[1]/1000*3.28,1))
            if(label==self.target and distdiff>self.epsDist and medvals[1]>0):
                self.lastsaid = medvals
                heading = self.calc_direction(medvals[1],medvals[2])
                print("Notified User")
                vdistance = str(round(self.lastsaid[1]/1000*3.28,1))
                message=self.target+vdistance+"feetat"+heading+"o'clock"
                #Popen(cmd_start+'"'+message+'"'+cmd_mid+cmd_end, shell=True)
                
                #Popen('message.mp3', shell=True)
                #gtts.setPitch(1)
                                
                # Instantiates a client
                client = texttospeech.TextToSpeechClient()

                # Set the text input to be synthesized
                synthesis_input = texttospeech.SynthesisInput(text=message)

                # Build the voice request, select the language code ("en-US") and the ssml
                # voice gender ("neutral")
                voice = texttospeech.VoiceSelectionParams(
                    language_code="en-GB", ssml_gender=texttospeech.SsmlVoiceGender.NEUTRAL, name="en-GB-Wavenet-B"
                )

                # Select the type of audio file you want returned
                audio_config = texttospeech.AudioConfig(
                    audio_encoding=texttospeech.AudioEncoding.MP3
                )

                # Perform the text-to-speech request on the text input with the selected
                # voice parameters and audio file type
                response = client.synthesize_speech(
                    input=synthesis_input, voice=voice, audio_config=audio_config
                )
                try:
                    #t1 = gtts.gTTS(message,tld='co.uk')
                    #t1.save("message.mp3")
                    #os.system('mpg123 -f -22768 message.mp3')
                    with open("output.mp3", "wb") as out:
                        # Write the response to the output file.
                        out.write(response.audio_content)
                        print('Audio content written to file "output.mp3"')
                        os.system('mpg123 -f -22768 output.mp3')

                except:
                    continue
              
          
            #cmd_start+self.target+vdistance+"feetat"+heading+"o'clock"+speed,shell=True
            cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
            #cv2.imshow("depth", depthFrameColor)
            #cv2.imshow("rgb", frame)
            if cv2.waitKey(1) == ord('q'):
                break

            
            corners = np.asarray([[-0.5,-1.0,0.35],[0.5,-1.0,0.35],[0.5,1.0,0.35],[-0.5,1.0,0.35],[-0.5,-1.0,1.7],[0.5,-1.0,1.7],[0.5,1.0,1.7],[-0.5,1.0,1.7]])

            bounds = corners.astype("float64")
            bounds = o3d.utility.Vector3dVector(bounds)
            oriented_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(bounds)

            # inRight = qRight.get()
            right = pcFrame
 
            frame = depthFrame
            median = cv2.medianBlur(frame, 5)
            # median2 = cv2.medianBlur(median,5)

            
            pcl_converter = PointCloudVisualizer(self.depthai.right_intrinsic, 640, 400)

            pcd = pcl_converter.rgbd_to_projection(median, right,False)

            #to get points within bounding box
            num_pts = oriented_bounding_box.get_point_indices_within_bounding_box(pcd.points)

            # vis = o3d.visualization.Visualizer()
            
            # vis.create_window()
            # if not self.isstarted:
            #     vis.add_geometry(pcd)
            #     vis.add_geometry(oriented_bounding_box)
            #     self.isstarted = True       
                        
            # else:
            #     vis.update_geometry(pcd)
            #     vis.update_geometry(oriented_bounding_box)
            #     vis.poll_events()
            #     vis.update_renderer()

            if len(num_pts)>5000:
                print("Obstacle")
                #if (rpi==1):
                    #s.send(bytes('1','utf-8'))
            else:
                print("Nothing")
                #if (rpi==1):
                    #s.send(bytes('0','utf-8'))


        if self.pcl_converter is not None:
            self.pcl_converter.close_window()
         
    
    def calc_direction(self,z, x):
        z = round(z/1000*3.28,1)
        x = round(x/1000*3.28,1)
        angle = round(np.arctan(x/z),1)*180/3.14
        print(z,x)
        print(angle)
        if (15<angle<=45):
            heading = 1
        elif (45<angle<75):
            heading = 2
        elif (-15<angle<=15):
            heading = 12
        elif (-45<angle<=-15):
            heading = 11
        elif (-75<angle<=-45):
            heading = 11
        return(str(heading))
    
    def get_target(self):
        while True:
            saidtext=''
            if (modeswitchpin == 1 and mode == 1):
                r = sr.Recognizer()
                with sr.Microphone(device_index=6) as source:
                    print("You have entered the scanning mode:")
                    prompt='Say'+'object'
                    #Popen([s_cmd_start+prompt+speed+s_cmd_end],shell=True)
                    Popen(opensound+'sayobject.mp3', shell=True)
                    audio=r.adjust_for_ambient_noise(source)
                    audio=r.listen(source)
                try:
                    text = "handle"
                    # r.recognize_google(audio)

                    print("You said: " + text)
                    if (text not in self.labelMap):
                        errormessage='Try'+'again'
                        #Popen([s_cmd_start+errormessage+speed+s_cmd_end],shell=True)
                        Popen(opensound+'tryagain.mp3', shell=True)
                        break
                    else:
                        saidtext=text
                        confirm='Scanning'+'for'
                        #Popen([s_cmd_start+confirm+saidtext+speed+s_cmd_end],shell=True)
                        scanmessage = 'Scanning '+'for '+text
                        #print(cmd_start+'"'+scanmessage+'"'+' '+cmd_mid+scan_end)
                        Popen(cmd_start+'"'+scanmessage+'"'+' '+cmd_mid+scan_end, shell=True)
                        Popen(opensound+'scan.mp3', shell=True)

                except sr.UnknownValueError:
                    print('Sorry could not recognize voice')
                    errormessage='Try'+'again'
                    #Popen([s_cmd_start+errormessage+speed+s_cmd_end],shell=True)
                    Popen(opensound+'tryagain.mp3', shell=True)
                    break
                except sr.RequestError as e:
                    print("error 2")
        print("out")

    def run_pointcloud(self): 

        for depthFrame, pcFrame in self.depthai.pc():
        
            corners = np.asarray([[-0.5,-1.0,0.35],[0.5,-1.0,0.35],[0.5,1.0,0.35],[-0.5,1.0,0.35],[-0.5,-1.0,1.7],[0.5,-1.0,1.7],[0.5,1.0,1.7],[-0.5,1.0,1.7]])

            bounds = corners.astype("float64")
            bounds = o3d.utility.Vector3dVector(bounds)
            oriented_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(bounds)

            right = pcFrame
 
            frame = depthFrame
            median = cv2.medianBlur(frame, 5)
            
            self.pcl_converter = PointCloudVisualizer(self.depthai.right_intrinsic, 640, 400)

            pcd = self.pcl_converter.rgbd_to_projection(median, right,False)

            #to get points within bounding box
            num_pts = oriented_bounding_box.get_point_indices_within_bounding_box(pcd.points)


            # if not self.isstarted:
            #     self.depthai.vis.add_geometry(pcd)
            #     self.depthai.vis.add_geometry(oriented_bounding_box)
            #     self.isstarted = True       
                        
            # else:
            #     self.depthai.vis.update_geometry(pcd)
            #     self.depthai.vis.update_geometry(oriented_bounding_box)
            #     self.depthai.vis.poll_events()
            #     self.depthai.vis.update_renderer()
            if len(num_pts)>5000:
                print("Obstacle")
                if (rpi==1):
                    s.send(bytes('1','utf-8'))
            else:
                print("Nothing")
                if (rpi==1):
                    s.send(bytes('0','utf-8'))

        if self.pcl_converter is not None:
            self.pcl_converter.close_window()

if __name__ == '__main__':

    Main().run_yolo_pc()
