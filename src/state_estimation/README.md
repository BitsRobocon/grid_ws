## Example generating ArUco tags:

For generating 4 tags of DICT_4X4_100 type (and generated tag ids defaulting to 0, 1, 2, 3 )  

```python3  generate_aruco.py --type DICT_4X4_100 --number 4 --output ./tags```

## For detecting ArUco tags:

```python3  detect_aruco.py```

## Using ROS with aruco detection:

1) add the detect_aruco_ROSnode.py in the /scripts of the workspace. 
2) change ```bot_description``` to _package name_ (on lines 15, 16)
3) add msg and srv folders to the same package
4) refer the CMakeLists.txt and package.xml files and do the needed changes in the corresponding files of your workspace.
5) build the workspace, source it
6) TODO: Velocity isnt getting calculated, not sure if its due to the slow movement of my aruco tag or the logic in the code. Laptop FOV is less so could not conclude. Logic BT is more likely tho.
