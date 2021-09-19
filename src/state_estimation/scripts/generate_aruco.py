import numpy as np
import argparse
import cv2
import sys
from utils import ARUCO_DICT 

# argument parser and parsing the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", default=None, help="path to output image containing ArUCo tag")
ap.add_argument("-i", "--id", type=int, default=-1, help="ID of ArUCo tag to generate")
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to generate")
ap.add_argument("-n", "--number", type=int, default=1, help="number of ArUCo tags to generate")
args = vars(ap.parse_args())
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])

if args["id"]>0:

    print("[INFO] generating ArUCo tag type '{}' with ID '{}'".format(args["type"], args["id"]))
    tag = np.zeros((225, 225, 1), dtype="uint8")
    cv2.aruco.drawMarker(arucoDict, args["id"], 225, tag, 1)

    # write the generated ArUCo tag to disk and then display it to our screen
    if args["output"]:
        cv2.imwrite(f'{args["output"]}', tag)
    else:
        cv2.imwrite(f'./tags/{args["type"]}_id_{args["id"]}.png', tag)
    cv2.imshow("ArUCo Tag", tag)
    cv2.waitKey(0)

else:
    for i in range(args["number"]):

        # allocate memory for the output ArUCo tag and then draw the ArUCo tag on the output image
        print("[INFO] generating ArUCo tag type '{}' with ID '{}'".format(args["type"], i))
        tag = np.zeros((225, 225, 1), dtype="uint8")
        cv2.aruco.drawMarker(arucoDict, i, 225, tag, 1)

        # write the generated ArUCo tag to disk and then display it to our screen
        cv2.imwrite(f'./tags/{args["type"]}_id_{i}.png', tag)
        cv2.imshow("ArUCo Tag", tag)