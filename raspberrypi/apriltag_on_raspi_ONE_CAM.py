#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import time
import sys
import cv2
from dt_apriltags import Detector
import numpy as np
import math

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags

configFile = "/boot/frc.json"

class CameraConfig: pass

team = 2052
server = False
cameraConfigs = []
cameras = []

# Server config and stuff

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    camera = UsbCamera(config.name, config.path)
    server = CameraServer.startAutomaticCapture(camera=camera)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

####################################################################################

class Tag():

    def __init__(self, tag_size, family):
        self.family = family
        self.size = tag_size
        self.locations = {}
        self.orientations = {}
        self.found_tags = []
        corr = np.eye(3)
        corr[0, 0] = -1
        self.tag_corr = corr
        
    def addTag(self,id,x,y,z,theta_x,theta_y,theta_z):
        self.locations[id]=self.inchesToTranslationVector(x,y,z)
        self.orientations[id]=self.eulerAnglesToRotationMatrix(theta_x,theta_y,theta_z)
        
    # Calculates Rotation Matrix given euler angles.
    def eulerAnglesToRotationMatrix(self, theta_x,theta_y,theta_z):
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(theta_x), -np.sin(theta_x)],
                        [0, np.sin(theta_x), np.cos(theta_x)]
                        ])

        R_y = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                        [0, 1, 0],
                        [-np.sin(theta_y), 0, np.cos(theta_y)]
                        ])

        R_z = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                        [np.sin(theta_z), np.cos(theta_z), 0],
                        [0, 0, 1]
                        ])

        R = np.matmul(R_z, np.matmul(R_y, R_x))

        return np.transpose(R)
        
    def inchesToTranslationVector(self,x,y,z):
        #inches to meters
        return np.array([[x],[y],[z]])*0.0254
    
    def addFoundTags(self, tags):
        self.found_tags = []
        for tag in tags:
            if tag.hamming < 9:
                self.found_tags.append(tag)

    def getFilteredTags(self):
        return self.found_tags

    def estimateTagPose(self, tag_id, R,t):
        localTranslation = self.tag_corr @ np.transpose(R) @ t
        #localTranslation = self.tag_corr @ t
        #localRotation = self.tag_corr @ np.transpose(R)
        #print(self.orientations[tag_id])
        #return (localRotation+ self.orientations[tag_id], localTranslation + self.locations[tag_id])
        tagYaw = math.degrees(math.atan2(-R[2, 0], math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])))
        return (localTranslation + self.locations[tag_id], tagYaw)

####################################################################################

# Visualize input frame with detected tags

def visualize_frame(img, tags):
    color_img = img

    for tag in tags:
        # Add bounding rectangle
        for idx in range(len(tag.corners)):
            cv2.line(color_img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)),
                    (0, 255, 0), thickness=3)
        # Add Tag ID text
        cv2.putText(color_img, str(tag.tag_id),
                    org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8,
                    color=(0, 0, 255),
                    thickness=3)
        # Add Tag Corner
        cv2.circle(color_img, tuple(tag.corners[0].astype(int)), 2, color=(255, 0, 255), thickness=3)
    
    return color_img

####################################################################################

# Pose Estimation

class PoseEstimator():

    def estimatePoseMeters(self):
        
        estimated_poses_list = []

        for tag in tags.getFilteredTags():
            estimated_poses_list.append(tags.estimateTagPose(tag.tag_id, tag.pose_R, tag.pose_t))
        
        if not estimated_poses_list:
            print("no estimated poses list")
            # If we have no samples, report none
            return (None, estimated_poses_list)
                
        total = np.array([0.0, 0.0, 0.0])
        totalYaw = 0.0

        for pose in estimated_poses_list:
            total = ([(total[0] + pose[0][0]), (total[1] + pose[0][1]), (total[2] + pose[0][2])])
            totalYaw = totalYaw + pose[1]
        avg = np.divide(total, len(estimated_poses_list))
        avgYaw = totalYaw / len(estimated_poses_list)

        return (avg, avgYaw)
        
####################################################################################

# Constants

# Camera Parameters:
camera_info = {}
camera_info["cameraName"] = "PiCamera1"
# [fx, fy, cx, cy] f is focal length in pixels x and y, c is optical center in pixels x and y.
# focal_pixel = (image_width_in_pixels * 0.5) / tan(FOV * 0.5 * PI/180)

# 1280,960 res: **UNTESTED**
#camera_info["params"] = [1338.26691, 1338.26691, 639.266524, 486.552512] 
#RES = (1280,960)
# 640,480 res: 
camera_info["params"] = [669.13345619, 669.13345619, 319.63326201, 243.27625621]
RES = (640,480)
# 320,240 res: 
#camera_info["params"] = [334.566728095, 334.566728095, 159.816631005, 121.638128105]
#RES = (320,240)

camera_info["res"] = RES

inchesInAMeter = 39.37

TAG_SIZE = (6.5)/39.37
FAMILIES = "tag36h11"

tags = Tag(TAG_SIZE, FAMILIES)
poseEstimator = PoseEstimator()

####################################################################################

# INITIALIZE VARIABLES #

# add tags:
# takes in id,x,y,z,theta_x,theta_y,theta_z
# theta_x:roll, theta_y:pitch, theta_z:yaw in radians

tags.addTag(1, 593.68, 9.68, 53.38, 0, 0, 2.0944)
tags.addTag(2, 637.21, 34.79, 53.38, 0, 0, 2.0944)
tags.addTag(3, 652.73, 196.17, 57.13, 0, 0, 3.14159)
tags.addTag(4, 652.73, 218.42, 57.13, 0, 0, 3.14159)
tags.addTag(5, 578.77, 323.00, 53.38, 0, 0, 4.71239)
tags.addTag(6, 72.5, 323.00, 53.38, 0, 0, 4.71239)
tags.addTag(7, -1.50, 218.42, 57.13, 0, 0, 0)
tags.addTag(8, -1.50, 196.17, 57.13, 0, 0, 0)
tags.addTag(9, 14.02, 34.79, 53.38, 0, 0, 1.0472)
tags.addTag(10, 57.54, 9.68, 53.38, 0, 0, 1.0472)
tags.addTag(11, 468.69, 146.19, 52.00, 0, 0, 5.23599)
tags.addTag(12, 468.69, 177.10, 52.00, 0, 0, 1.0472)
tags.addTag(13, 441.74, 161.62, 52.00, 0, 0, 3.14159)
tags.addTag(14, 209.48, 161.62, 52.00, 0, 0, 0)
tags.addTag(15, 182.73, 177.10, 52.00, 0, 0, 2.0944)
tags.addTag(16, 182.73, 146.19, 52.00, 0, 0, 4.18879)

####################################################################################

if __name__ == "__main__":

    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()

    raspberryPiTable = ntinst.getTable(camera_info["cameraName"])

    CameraServer.setSize(CameraServer.kSize640x480)
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    detector = Detector(families='tag36h11',
                        nthreads=4,
                        quad_decimate=2,
                        quad_sigma=0,
                        refine_edges=1,
                        decode_sharpening=0,
                        debug=0
                        )
    

    cvSink = CameraServer.getVideo(camera_info["cameraName"])

    processedOutput = CameraServer.putVideo(camera_info["cameraName"] + " Processed Output", RES[0], RES[1])

    frame = np.zeros(shape=(RES[1], RES[0], 3), dtype=np.uint8)

    while True:
        frameTime, frame = cvSink.grabFrame(frame)
        if frameTime == 0:
            processedOutput.notifyError(cvSink.getError())

            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detected_tags = detector.detect(gray, estimate_tag_pose=True, camera_params=camera_info["params"], tag_size=TAG_SIZE)

        tags.addFoundTags(detected_tags)
        
        processedOutput.putFrame(visualize_frame(frame, tags.getFilteredTags()))
            
        # pose detector gives x (left and right), y (up and down),z (forward backward)
        # field relative: x (points away away from driverstation aka forward backward) y (perpendicular to x aka left and right)
        pose = poseEstimator.estimatePoseMeters()
        if np.all(pose[0]):
            raspberryPiTable.putNumberArray(camera_info["cameraName"], [pose[0][2], pose[0][0], pose[0][1], pose[1]])
            raspberryPiTable.putBoolean(camera_info["cameraName"]+ "tagFound", True)
        else:
            raspberryPiTable.putBoolean(camera_info["cameraName"]+ "tagFound", False)
