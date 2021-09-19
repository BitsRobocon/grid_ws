#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

def callback(msg):  # Define a function called "callback" that receives a parameter named "msg"
	print msg.data  # Print the value "data" inside the "msg" parameter
	print "map was read\n"

print "-----Subscriber started-----\n"

rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('/costmap_node/costmap/costmap', OccupancyGrid, callback)  # Create a Subscriber object that will listen to the "phrases" topic and will call the "callback" function each time it reads something from the topic

rospy.spin()  # Create a loop that will keep the program in execution