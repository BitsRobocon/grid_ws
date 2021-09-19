import time
import sys
import numpy as np
import argparse
import cv2
import imutils
from imutils.video import VideoStream
from utils import ARUCO_DICT 


arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
arucoParams = cv2.aruco.DetectorParameters_create()

# for image 
# image=cv2.imread('./tags/test.png')
# cv2.imshow("etst",image)
# cv2.waitKey(0)

# for webcam video
vs = VideoStream(src=0).start()
time.sleep(3)


while True:
    # grab the frame from the threaded video stream and resize it to maximum 1000 px width
    frame = vs.read()
    frame = imutils.resize(frame, width=1000)
    image=frame
    
    # detect ArUco markers in the input frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image,arucoDict,parameters=arucoParams)

    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()

        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):

            # extract the marker corners order: (top-left, top-right, bottom-right, and bottom-left)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = map(tuple,corners.astype(np.int))
            print(corners)

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
            print("[INFO] ArUco marker ID: {}".format(markerID))

    # show the output image
    cv2.imshow("Image", image)
    key=cv2.waitKey(1)

    # break from the loop
    if key==ord("q"):
	    break

# exit
cv2.destroyAllWindows()
vs.stop()