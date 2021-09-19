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
                        self.bot1.curr_pose.position.x = cX
                        self.bot1.curr_pose.position.y = cY
                        # if (self.delta_t > 2):
                        self.bot1.curr_twist.linear.x = (self.bot1.curr_pose.position.x - self.bot1.prev_pose.position.x)/self.delta_t
                        self.bot1.curr_twist.linear.y = (self.bot1.curr_pose.position.y - self.bot1.prev_pose.position.y)/self.delta_t
                        self.bot1.prev_pose = copy.deepcopy(self.bot1.curr_pose)

                    elif markerID == 1:
                        self.bot2.curr_pose.position.x = cX
                        self.bot2.curr_pose.position.y = cY
                        # if (self.delta_t > 2):
                        self.bot2.curr_twist.linear.x = (self.bot2.curr_pose.position.x - self.bot2.prev_pose.position.x)/self.delta_t
                        self.bot2.curr_twist.linear.y = (self.bot2.curr_pose.position.y - self.bot2.prev_pose.position.y)/self.delta_t
                        self.bot2.prev_pose = copy.deepcopy(self.bot2.curr_pose)

                    elif markerID == 2:
                        self.bot3.curr_pose.position.x = cX
                        self.bot3.curr_pose.position.y = cY
                        # if (self.delta_t > 2):
                        self.bot3.curr_twist.linear.x = (self.bot3.curr_pose.position.x - self.bot3.prev_pose.position.x)/self.delta_t
                        self.bot3.curr_twist.linear.y = (self.bot3.curr_pose.position.y - self.bot3.prev_pose.position.y)/self.delta_t
                        self.bot3.prev_pose = copy.deepcopy(self.bot3.curr_pose)

                    elif markerID == 3:
                        self.bot4.curr_pose.position.x = cX
                        self.bot4.curr_pose.position.y = cY
                        # if (self.delta_t > 2):
                        self.bot4.curr_twist.linear.x = (self.bot4.curr_pose.position.x - self.bot4.prev_pose.position.x)/self.delta_t
                        self.bot4.curr_twist.linear.y = (self.bot4.curr_pose.position.y - self.bot4.prev_pose.position.y)/self.delta_t
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