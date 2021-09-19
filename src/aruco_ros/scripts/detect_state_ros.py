#!/usr/bin/env python

import rospy
import time
import sys
import numpy as np
import argparse
import cv2
import imutils
from imutils.video import VideoStream
# from utils import ARUCO_DICT 
from geometry_msgs.msg import Pose, Twist
import copy
import math
# Replace aruco_ros by whatever package name you have given.
from aruco_ros.msg import BotPose, BotTwist
from aruco_ros.srv import SrvPose, SrvPoseRequest, SrvPoseResponse, SrvTwist, SrvTwistRequest, SrvTwistResponse



class bot_state():
    def __init__(self):
        self.prev_pose = Pose()
        self.curr_pose = Pose()
        self.curr_twist = Twist()

class detect_aruco():
    def __init__(self):
        rospy.init_node('detect_aruco', anonymous = True)
        self.rate = rospy.Rate(10)
        print("Detect Aruco class initialised!")
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.matrix_coefficients = np.load('calibration_matrix.npy')
        self.distortion_coefficients = np.load('distortion_coefficients.npy')
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.bot1 = bot_state()
        self.bot2 = bot_state()
        self.bot3 = bot_state()
        self.bot4 = bot_state()

        self.poses = BotPose()
        self.twists = BotTwist()

        self.time = rospy.Time.now()
        self.flag = 0 
        self.delta_t = 0
        self.t_prev = rospy.Time.now() 

        self.pub_pose = rospy.Publisher('/bot_poses', BotPose, queue_size=10)
        self.pub_twist = rospy.Publisher('/bot_twists', BotTwist, queue_size=10)

        self.service_pose = rospy.Service("srv_bot_pose", SrvPose, self.service_pose)
        self.service_twist = rospy.Service("srv_bot_twist", SrvTwist, self.service_twist)
        self.pose_response = SrvPoseResponse()
        self.twist_response = SrvTwistResponse()


        # for image 
        # self.image=cv2.imread('./tags/test.png')
        # cv2.imshow("etst",image)
        # cv2.waitKey(0)

        # for webcam video
        self.vs = VideoStream(src=0).start()
        time.sleep(3)

    def isclose(self, x, y, rtol=1.e-5, atol=1.e-8):
        return abs(x-y) <= atol + rtol * abs(y)

    def euler_angles_from_rotation_matrix(self, R):
        '''
        From a paper by Gregory G. Slabaugh (undated),
        "Computing Euler angles from a rotation matrix
        '''
        phi = 0.0
        if self.isclose(R[2,0],-1.0):
            theta = math.pi/2.0
            psi = math.atan2(R[0,1],R[0,2])
        elif self.isclose(R[2,0],1.0):
            theta = -math.pi/2.0
            psi = math.atan2(-R[0,1],-R[0,2])
        else:
            theta = -math.asin(R[2,0])
            cos_theta = math.cos(theta)
            psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
            phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
        # return psi, theta, phi      #roll, pitch, yaw
        return phi

    def pose_esitmation(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get(self.arucoDict)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
            cameraMatrix = self.matrix_coefficients,
            distCoeff = self.distortion_coefficients)

        # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.matrix_coefficients,
                                                                           self.distortion_coefficients)
                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(frame, corners) 
                # print(tvec[0][0][0], tvec[0][0][1], tvec[0][0][2]) #x,y,z 
                R_mat, _ = cv2.Rodrigues(rvec)
                yaw = self.euler_angles_from_rotation_matrix(R_mat) 

                # Draw Axis
                cv2.aruco.drawAxis(frame, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)  

        return frame , tvec[0][0][0], yaw 

    def service_pose(self, request):
        self.pose_response.header.stamp = self.time
        self.pose_response.pose = [self.bot1.curr_pose, self.bot2.curr_pose, self.bot3.curr_pose, self.bot4.curr_pose]
        self.pose_response.uid = ['0', '1', '2', '3']

        return self.pose_response
    
    def service_twist(self, request):
        self.twist_response.header.stamp = self.time
        self.twist_response.twist = [self.bot1.curr_twist, self.bot2.curr_twist, self.bot3.curr_twist, self.bot4.curr_twist]
        self.twist_response.uid = ['0', '1', '2', '3']

        return self.twist_response


    def detect_function(self):
        # while not rospy.is_shutdown():
        # grab the frame from the threaded video stream and resize it to maximum 1000 px width
        frame = self.vs.read()
        frame = imutils.resize(frame, width=1000)
        image=frame
        
        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image,self.arucoDict,parameters=self.arucoParams)
        _ , x_pos, yaw = self.euler_angles_from_rotation_matrix(frame, self.aruco_dict_type, self.k, self.d)

        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()

            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):

                # extract the marker corners order: (top-left, top-right, bottom-right, and bottom-left)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = map(tuple,corners.astype(np.int))
                # print(corners)

                # draw the bounding box of the ArUCo detection
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                
                # compute and draw the center (x, y) coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                
                # draw the ArUco marker ID on the image
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 5), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                # print("[INFO] ArUco marker ID: {}".format(markerID))
            
                self.time = rospy.Time.now()
                self.delta_t = (self.time - self.t_prev).to_sec()      ## delta_t = time - t_prev


                if self.flag == 0:
                    self.delta_t = 0
                    self.flag +=1

                else:
                    self.flag = self.flag + 1 
                    if markerID == 0:
                        self.bot1.curr_pose.position.x = x_pos
                        self.bot1.curr_pose.position.y = yaw    # didn't know for angular so used this :p
                        # if (self.delta_t > 2):
                        self.bot1.curr_twist.linear.x = (self.bot1.curr_pose.position.x - self.bot1.prev_pose.position.x)/self.delta_t
                        self.bot1.curr_twist.angular.z = (self.bot1.curr_pose.position.y - self.bot1.prev_pose.position.y)/self.delta_t
                        self.bot1.prev_pose = copy.deepcopy(self.bot1.curr_pose)

                    elif markerID == 1:
                        self.bot2.curr_pose.position.x = x_pos
                        self.bot2.curr_pose.position.y = yaw
                        # if (self.delta_t > 2):
                        self.bot2.curr_twist.linear.x = (self.bot2.curr_pose.position.x - self.bot2.prev_pose.position.x)/self.delta_t
                        self.bot2.curr_twist.angular.z = (self.bot2.curr_pose.position.y - self.bot2.prev_pose.position.y)/self.delta_t
                        self.bot2.prev_pose = copy.deepcopy(self.bot2.curr_pose)

                    elif markerID == 2:
                        self.bot3.curr_pose.position.x = x_pos
                        self.bot3.curr_pose.position.y = yaw
                        # if (self.delta_t > 2):
                        self.bot3.curr_twist.linear.x = (self.bot3.curr_pose.position.x - self.bot3.prev_pose.position.x)/self.delta_t
                        self.bot3.curr_twist.angular.z = (self.bot3.curr_pose.position.y - self.bot3.prev_pose.position.y)/self.delta_t
                        self.bot3.prev_pose = copy.deepcopy(self.bot3.curr_pose)

                    elif markerID == 3:
                        self.bot4.curr_pose.position.x = x_pos
                        self.bot4.curr_pose.position.y = yaw
                        # if (self.delta_t > 2):
                        self.bot4.curr_twist.linear.x = (self.bot4.curr_pose.position.x - self.bot4.prev_pose.position.x)/self.delta_t
                        self.bot4.curr_twist.angular.z = (self.bot4.curr_pose.position.y - self.bot4.prev_pose.position.y)/self.delta_t
                        self.bot4.prev_pose = copy.deepcopy(self.bot4.curr_pose)

            if self.delta_t > 2:
                print("Resetting delta t to 0")
                self.t_prev = self.time
                self.delta_t = 0

            self.poses.header.stamp = self.time
            self.poses.pose = [self.bot1.curr_pose, self.bot2.curr_pose, self.bot3.curr_pose, self.bot4.curr_pose]
            self.poses.uid = ['0', '1', '2', '3']

            self.twists.header.stamp = self.time
            self.twists.twist = [self.bot1.curr_twist, self.bot2.curr_twist, self.bot3.curr_twist, self.bot4.curr_twist]
            self.twists.uid = ['0', '1', '2', '3']

            self.pub_pose.publish(self.poses)
            self.pub_twist.publish(self.twists)

        # show the output image
        cv2.imshow("Image", image)
        key=cv2.waitKey(1)

        # break from the loop
        # if key==ord("q"):
        #     break
        # self.rate.sleep()

    # Exit            
    def exit_detect_aruco(self):
        cv2.destroyAllWindows()
        self.vs.stop()
   
if __name__ == "__main__":
    print("helllo")
    o = detect_aruco()   
    while not rospy.is_shutdown(): 
        o.detect_function()
        o.rate.sleep()
    o.exit_detect_aruco()