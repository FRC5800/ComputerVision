#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

import time
import sys
import cv2
import numpy as np

from cscore import CameraServer, VideoSource
from networktables import NetworkTablesInstance

if __name__ == "__main__":
    #Start network tables to conect to dashboard
    ntinst = NetworkTablesInstance.getDefault()
    print("Setting up NetworkTables client for team {}".format(5800))
    ntinst.initialize(server='10.58.0.2')
    sd = ntinst.getTable("SmartDashboard")
    sd.putNumber("SomeNumber", 1234)
    # Get CameraServer and initialize camera(s)
    print("Setting up cameras")
    cs = CameraServer.getInstance()
    # Start camera 0
    cam0 = cs.startAutomaticCapture(name="cam0", path='/dev/video0')
    cam0.setResolution(160, 120)
    # Get a CvSink. This will capture images from the camera
    cvSink = cs.getVideo(name="cam0")
    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = cs.putVideo(name="p_cam0", width=160, height=120)
    # sending cam to dashboard
    cameras = []
    cameras.append(cam0)
    cameras.append(outputStream)
    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(160, 120, 3), dtype=np.uint8)
    hsv = np.zeros(shape=(160, 120, 3), dtype=np.uint8)
    mask = np.zeros(shape=(160, 120, 3), dtype=np.uint8)
    out = np.zeros(shape=(160, 120, 3), dtype=np.uint8)
    blur = np.zeros(shape=(160, 120, 3), dtype=np.uint8)
    closing = np.zeros(shape=(160, 120, 3), dtype=np.uint8)
    opening = np.zeros(shape=(160, 120, 3), dtype=np.uint8)
    dilation = np.zeros(shape=(160, 120, 3), dtype=np.uint8)
    erosion = np.zeros(shape=(160, 120, 3), dtype=np.uint8)
    edges = np.zeros(shape=(160, 120, 3), dtype=np.uint8)
    contours = np.zeros(shape=(160, 120, 3), dtype=np.uint8)
    kernel3 = np.ones((3,3), np.uint8)
    kernel9 = np.ones((11,11), np.float32)/225
    # Range setting
    lower_led = np.array([0, 150, 60])
    upper_led = np.array([13, 240, 170])
    # loop forever
    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, img = cvSink.grabFrame(img)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError());
            # skip the rest of the current iteration
            continue
        # Put your openCv logic here
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Mask Applying
        mask = cv2.inRange(hsv, lower_led, upper_led)
        #Image filtering
        blur = cv2.medianBlur(mask, 15)
        closing = cv2.morphologyEx(blur, cv2.MORPH_CLOSE, kernel9)
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel9)
        dilation = cv2.dilate(opening, kernel9, iterations = 1)
        erosion = cv2.erode(dilation, kernel3, iterations = 1)
        # Edge detection
        edges = cv2.Canny(erosion, 100, 150)
        erosion, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #Center of the object
        if len(contours) != 0:
        
            sorted(contours, key=cv2.contourArea, reverse = True)
            
            c = contours[0]
            x, y, w, z = cv2.boundingRect(c)
            cy1 = int(y+(z/2))
            cx1 = int(x+(w/2))
            
            '''if len(contours) > 2:
                v = contours[2]
                f, g ,h, i = cv2.boundingRect(v)
                
                cy2 = int(g+(i/2))
                cx2 = int(f+(h/2))

                cx = int((cx1+cx2)/2)
                cy = int((cy1+cy2)/2)

                cv2.circle(erosion, (cx, cy), 5, (150, 0, 255), -1)
                sd.putNumber("Coordinate X", cx)
                sd.putNumber("Coordinate Y", cy)
                
            elif len(contours) != 1:
                v = contours[1]
                f, g ,h, i = cv2.boundingRect(v)
                
                cy2 = int(g+(i/2))
                cx2 = int(f+(h/2))

                cx = int((cx1+cx2)/2)
                cy = int((cy1+cy2)/2)
                #print (cx)
                #print (cy)

                cv2.circle(erosion, (cx, cy), 5, (150, 0, 255), -1)
                sd.putNumber("Coordinate X", cx)
                sd.putNumber("Coordinate Y", cy)
            else:'''
            cx = cx1
            cy = cy1
            #coordinates = ('X' + str(cx) + 'Y' + str(cy) + 'N' + str(n)) #String form
            #print(coordinates)
            cv2.circle(erosion, (cx, cy), 5, (150, 0, 255), -1)
            #sd.putNumber("Coordinate X", cx)
            #sd.putNumber("Coordinate Y", cy)


        outputStream.putFrame(erosion)
