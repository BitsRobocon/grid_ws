#! /usr/bin/env python

# References -
# https://stackoverflow.com/questions/57271100/how-to-feed-the-data-obtained-from-rospy-subscriber-data-into-a-variable

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import tf

import numpy as np
import pandas as pd
import math
import time

from Astar import Astar
from Astar_left_biased import Astar_left_biased
from Astar_right_biased import Astar_right_biased
from plot_anim import plot_anim
from vectorize import vectorize

import esp_ip_publisher_mp

M = None
height = None
width = None

bot1_goal_flag = 0
bot1_return_flag = 0
bot2_goal_flag = 0
bot2_return_flag = 0
bot3_goal_flag = 0
bot3_return_flag = 0
bot4_goal_flag = 0
# bot4_return_flag = 0 # not required

def callback(msg):  # Define a function called "callback" that receives a parameter named "msg"
	global M, height, width
	M = msg.data  # Print the value "data" inside the "msg" parameter
	height = msg.info.height
	width = msg.info.width
	print("map was read by mp node\n")
	main_motion_planner()
	
def bot1_goal_feedback(msg):
	global bot1_goal_flag
	bot1_goal_flag = msg.data

def bot1_return_feedback(msg):
	global bot1_return_flag
	bot1_return_flag = msg.data

def bot2_goal_feedback(msg):
	global bot2_goal_flag
	bot2_goal_flag = msg.data

def bot2_return_feedback(msg):
	global bot2_return_flag
	bot2_return_flag = msg.data

def bot3_goal_feedback(msg):
	global bot3_goal_flag
	bot3_goal_flag = msg.data

def bot3_return_feedback(msg):
	global bot3_return_flag
	bot3_return_flag = msg.data

def bot4_goal_feedback(msg):
	global bot4_goal_flag
	bot4_goal_flag = msg.data

def main_motion_planner():
	print("-----Motion planning will start now-----")


	# print('reading image data ...')
	# data= pd.read_csv("eroded.csv")

	global M, height, width
	
	M = np.array(M)
	M = np.reshape(M, (height,width))
	# df = pd.DataFrame(M)
	# df.to_csv('raw_data.csv', index=False)
	# print('Data loaded into matrix')
	(nrows,ncols) = M.shape


	skip = 10
	offset = 2


	small_rows = int(math.floor(nrows/skip))
	small_cols = int(math.floor(ncols/skip))


	smallerM = np.zeros((small_rows,small_cols))

	for i in range(small_rows):
		for j in range(small_cols):
			smallerM[i][j] = M[skip*(i-1)+offset][skip*(j-1)+offset]



	#print(smallerM)
	R1_obstacles = np.zeros((small_rows,small_cols))
	R1_obstacles_x = []
	R1_obstacles_y = []
	for i in range(small_rows):
		for j in range(small_cols):
			if smallerM[i][j] == 100:
				R1_obstacles[i][j] = 1
				R1_obstacles_x.append(i)
				R1_obstacles_y.append(j)

	#print(R1_obstacles_x)

	R1_obstacles_1 = np.zeros((nrows,ncols))
	R1_obstacles_x_1 = []
	R1_obstacles_y_1 = []
	for i in range(nrows):
		for j in range(ncols):
			if M[i][j] == 100:
				R1_obstacles_1[i][j] = 1
				R1_obstacles_x_1.append(i)
				R1_obstacles_y_1.append(j)



	#print(R1_obstacles)
	#Plan route

	start =[[58, 56],
			[58, 62],
			[58, 66],
			[58, 72]]
	goal = [[11, 16],
			[7, 18],
			[7, 112],
			[11, 112]]



	offset_x = 0#(start[0][0]%5)/2
	offset_y = 0#(start[0][1]%5)/2

	planned_paths = []
	path_ls= []
	return_planned_paths = []
	return_path_ls= []

	grid_size = 2

	rate = rospy.Rate(10) #Hz

	# bot 1 ------------------------------------------------------
	path = Astar(R1_obstacles_x,R1_obstacles_y,grid_size,2.25)
	rx, ry = path.planning(start[0][0], start[0][1], goal[0][0], goal[0][1])

	# RETURN JOURNEY PATH HERE
	for i in range(len(rx)-1):
			a = ((rx[i]+offset_x-1)*skip+offset,(ry[i] +offset_y-1)*skip+offset)
			return_path_ls.append(a)
	return_path_ls.append(((start[0][0]-1)*skip+offset, (start[0][1]-1)*skip+offset))
	return_planned_paths.append(return_path_ls)

	# DEPARTURE JOURNEY PATH HERE
	le = len(return_path_ls)
	path_ls= []
	for i in range(le):
		path_ls.append(return_path_ls[le-i-1])
	planned_paths.append(path_ls)
	
	vec_x_1,vec_y_1 = vectorize(return_planned_paths[0]) # vectors
	re_vec_x_1,re_vec_y_1 = vectorize(planned_paths[0]) # return vectors


	bot1_path = Path()
	bot1_pose = PoseStamped()
	bot1_path.header.frame_id = "base_footprint"
	for i in range(len(rx)):
		# bot1_pose.header.stamp = rospy.Time.now()
		bot1_pose.header.frame_id = "base_footprint"
		bot1_pose.header.seq = i
		bot1_pose.pose.position.x = path_ls[i][0]
		bot1_pose.pose.position.y = path_ls[i][1]
		bot1_pose.pose.position.z = 0

		yaw = math.atan2(vec_y_1[i], vec_x_1[i])
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		bot1_pose.pose.orientation.x = quaternion[0]
		bot1_pose.pose.orientation.y = quaternion[1]
		bot1_pose.pose.orientation.z = quaternion[2]
		bot1_pose.pose.orientation.w = quaternion[3]

		bot1_path.poses.append(bot1_pose)
	
	print("bot1 should move from start to goal now\n")
	bot1_traj_pub.publish(bot1_path)
	# rate.sleep()
	bot1_goal_pub.publish(bot1_pose)

	print("Waiting for bot1 to reach goal...")
	while bot1_goal_flag == 0:
		rate.sleep()
	print("Confirmation received. Package to be dropped now! Waiting...")
	setIP("192.168.137.158") # <---------------bot1 IP address here
	sendCommand(0,0,1)
	time.sleep(3)

	bot1_path = Path()
	bot1_pose = PoseStamped()
	bot1_path.header.frame_id = "base_footprint"
	for i in range(len(rx)):
		# bot1_pose.header.stamp = rospy.Time.now()
		bot1_pose.header.frame_id = "base_footprint"
		bot1_pose.header.seq = i
		bot1_pose.pose.position.x = return_path_ls[i][0]
		bot1_pose.pose.position.y = return_path_ls[i][1]
		bot1_pose.pose.position.z = 0

		yaw = math.atan2(re_vec_y_1[i], re_vec_x_1[i])
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		bot1_pose.pose.orientation.x = quaternion[0]
		bot1_pose.pose.orientation.y = quaternion[1]
		bot1_pose.pose.orientation.z = quaternion[2]
		bot1_pose.pose.orientation.w = quaternion[3]

		bot1_path.poses.append(bot1_pose)
	print("bot1 should move from goal to start now\n")
	bot1_traj_pub.publish(bot1_path)
	bot1_goal_pub.publish(bot1_pose)

	print("Waiting for bot1 to reach start...")
	while bot1_return_flag == 0:
		rate.sleep()
	print("Confirmation received. Next bot GO!\n")

	# bot2 ------------------------------------------------------
	return_path_ls= []
	path = Astar_right_biased(R1_obstacles_x,R1_obstacles_y,grid_size,2.25)
	rx, ry = path.planning(start[1][0], start[1][1], goal[1][0], goal[1][1])

	# RETURN JOURNEY PATH HERE
	for i in range(len(rx)-1):
			a = ((rx[i]+offset_x-1)*skip+offset,(ry[i] +offset_y-1)*skip+offset)
			return_path_ls.append(a)
	return_path_ls.append(((start[1][0]-1)*skip+offset, (start[1][1]-1)*skip+offset))
	return_planned_paths.append(return_path_ls)

	# DEPARTURE JOURNEY PATH HERE
	le = len(return_path_ls)
	path_ls= []
	for i in range(le):
		path_ls.append(return_path_ls[le-i-1])
	planned_paths.append(path_ls)

	vec_x_2,vec_y_2 = vectorize(return_planned_paths[1]) # vectors
	re_vec_x_2,re_vec_y_2 = vectorize(planned_paths[1]) # return vectors


	bot2_path = Path()
	bot2_pose = PoseStamped()
	bot2_path.header.frame_id = "base_footprint"
	for i in range(len(rx)):
		# bot2_pose.header.stamp = rospy.Time.now()
		bot2_pose.header.frame_id = "base_footprint"
		bot2_pose.header.seq = i
		bot2_pose.pose.position.x = path_ls[i][0]
		bot2_pose.pose.position.y = path_ls[i][1]
		bot2_pose.pose.position.z = 0

		yaw = math.atan2(vec_y_2[i], vec_x_2[i])
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		bot2_pose.pose.orientation.x = quaternion[0]
		bot2_pose.pose.orientation.y = quaternion[1]
		bot2_pose.pose.orientation.z = quaternion[2]
		bot2_pose.pose.orientation.w = quaternion[3]

		bot2_path.poses.append(bot2_pose)

	print("bot2 should move from start to goal now\n")
	bot2_traj_pub.publish(bot2_path)
	# rate.sleep()
	bot2_goal_pub.publish(bot2_pose)

	print("Waiting for bot2 to reach goal...")
	while bot2_goal_flag == 0:
		rate.sleep()
	print("Confirmation received. Package to be dropped now! Waiting...")
	setIP("192.168.137.158") # <---------------bot2 IP address here
	sendCommand(0,0,1)
	time.sleep(3)
	
	bot2_path = Path()
	bot2_pose = PoseStamped()
	bot2_path.header.frame_id = "base_footprint"
	for i in range(len(rx)):
		# bot2_pose.header.stamp = rospy.Time.now()
		bot2_pose.header.frame_id = "base_footprint"
		bot2_pose.header.seq = i
		bot2_pose.pose.position.x = return_path_ls[i][0]
		bot2_pose.pose.position.y = return_path_ls[i][1]
		bot2_pose.pose.position.z = 0

		yaw = math.atan2(re_vec_y_2[i], re_vec_x_2[i])
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		bot2_pose.pose.orientation.x = quaternion[0]
		bot2_pose.pose.orientation.y = quaternion[1]
		bot2_pose.pose.orientation.z = quaternion[2]
		bot2_pose.pose.orientation.w = quaternion[3]

		bot2_path.poses.append(bot2_pose)
	
	print("bot2 should move from goal to start now\n")
	bot2_traj_pub.publish(bot2_path)
	bot2_goal_pub.publish(bot2_pose)

	print("Waiting for bot2 to reach start...")
	while bot2_return_flag == 0:
		rate.sleep()
	print("Confirmation received. Next bot GO!\n")

	# bot3 ------------------------------------------------------
	return_path_ls= []
	path = Astar_left_biased(R1_obstacles_x,R1_obstacles_y,grid_size,2.4)
	rx, ry = path.planning(start[2][0], start[2][1], goal[2][0], goal[2][1])

	# RETURN JOURNEY PATH HERE
	for i in range(len(rx)-1):
			a = ((rx[i]+offset_x-1)*skip+offset,(ry[i] +offset_y-1)*skip+offset)
			return_path_ls.append(a)
	return_path_ls.append(((start[2][0]-1)*skip+offset, (start[2][1]-1)*skip+offset))
	return_planned_paths.append(return_path_ls)

	# DEPARTURE JOURNEY PATH HERE
	le = len(return_path_ls)
	path_ls= []
	for i in range(le):
		path_ls.append(return_path_ls[le-i-1])
	planned_paths.append(path_ls)

	vec_x_3,vec_y_3 = vectorize(return_planned_paths[2]) # vectors
	re_vec_x_3,re_vec_y_3 = vectorize(planned_paths[2]) # return vectors


	bot3_path = Path()
	bot3_pose = PoseStamped()
	bot3_path.header.frame_id = "base_footprint"
	for i in range(len(rx)):
		# bot3_pose.header.stamp = rospy.Time.now()
		bot3_pose.header.frame_id = "base_footprint"
		bot3_pose.header.seq = i
		bot3_pose.pose.position.x = path_ls[i][0]
		bot3_pose.pose.position.y = path_ls[i][1]
		bot3_pose.pose.position.z = 0

		yaw = math.atan2(vec_y_3[i], vec_x_3[i])
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		bot3_pose.pose.orientation.x = quaternion[0]
		bot3_pose.pose.orientation.y = quaternion[1]
		bot3_pose.pose.orientation.z = quaternion[2]
		bot3_pose.pose.orientation.w = quaternion[3]

		bot3_path.poses.append(bot3_pose)

	print("bot3 should move from start to goal now\n")
	bot3_traj_pub.publish(bot3_path)
	# rate.sleep()
	bot3_goal_pub.publish(bot3_pose)

	print("Waiting for bot3 to reach goal...")
	while bot3_goal_flag == 0:
		rate.sleep()
	print("Confirmation received. Package to be dropped now! Waiting...")
	setIP("192.168.137.158") # <---------------bot3 IP address here
	sendCommand(0,0,1)
	time.sleep(3)

	bot3_path = Path()
	bot3_pose = PoseStamped()
	bot3_path.header.frame_id = "base_footprint"
	for i in range(len(rx)):
		# bot3_pose.header.stamp = rospy.Time.now()
		bot3_pose.header.frame_id = "base_footprint"
		bot3_pose.header.seq = i
		bot3_pose.pose.position.x = return_path_ls[i][0]
		bot3_pose.pose.position.y = return_path_ls[i][1]
		bot3_pose.pose.position.z = 0

		yaw = math.atan2(re_vec_y_3[i], re_vec_x_3[i])
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		bot3_pose.pose.orientation.x = quaternion[0]
		bot3_pose.pose.orientation.y = quaternion[1]
		bot3_pose.pose.orientation.z = quaternion[2]
		bot3_pose.pose.orientation.w = quaternion[3]

		bot3_path.poses.append(bot3_pose)

	print("bot3 should move from goal to start now\n")
	bot3_traj_pub.publish(bot3_path)
	bot3_goal_pub.publish(bot3_pose)

	print("Waiting for bot3 to reach start...")
	while bot3_return_flag == 0:
		rate.sleep()
	print("Confirmation received. Next bot GO!\n")

	# bot 4 ------------------------------------------------------
	return_path_ls =[]
	path = Astar(R1_obstacles_x,R1_obstacles_y,grid_size,2.4)
	rx, ry = path.planning(start[3][0], start[3][1], goal[3][0], goal[3][1])

	# RETURN JOURNEY PATH HERE
	for i in range(len(rx)-1):
			a = ((rx[i]-offset_x-1)*skip+offset,(ry[i] +offset_y-1)*skip+offset)
			return_path_ls.append(a)
	return_path_ls.append( ((start[3][0]-1)*skip+offset, (start[3][1]-1)*skip+offset))
	return_planned_paths.append(return_path_ls)

	# DEPARTURE JOURNEY PATH HERE
	le = len(return_path_ls)
	path_ls= []
	for i in range(le):
		path_ls.append(return_path_ls[le-i-1])
	planned_paths.append(path_ls)

	vec_x_4,vec_y_4 = vectorize(return_planned_paths[3]) # vectors
	re_vec_x_4,re_vec_y_4 = vectorize(planned_paths[3]) # return vectors


	bot4_path = Path()
	bot4_pose = PoseStamped()
	bot4_path.header.frame_id = "base_footprint"
	for i in range(len(rx)):
		# bot4_pose.header.stamp = rospy.Time.now()
		bot4_pose.header.frame_id = "base_footprint"
		bot4_pose.header.seq = i
		bot4_pose.pose.position.x = path_ls[i][0]
		bot4_pose.pose.position.y = path_ls[i][1]
		bot4_pose.pose.position.z = 0

		yaw = math.atan2(vec_y_4[i], vec_x_4[i])
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		bot4_pose.pose.orientation.x = quaternion[0]
		bot4_pose.pose.orientation.y = quaternion[1]
		bot4_pose.pose.orientation.z = quaternion[2]
		bot4_pose.pose.orientation.w = quaternion[3]

		bot4_path.poses.append(bot4_pose)

	print("bot4 should move from start to goal now\n")
	bot4_traj_pub.publish(bot4_path)
	# rate.sleep()
	bot4_goal_pub.publish(bot4_pose)

	print("Waiting for bot4 to reach goal...")
	while bot4_goal_flag == 0:
		rate.sleep()
	print("Confirmation received. Package to be dropped now! Waiting...")
	setIP("192.168.137.158") # <---------------bot4 IP address here
	sendCommand(0,0,1)
	time.sleep(3)

	bot4_path = Path()
	bot4_pose = PoseStamped()
	bot4_path.header.frame_id = "base_footprint"
	for i in range(len(rx)):
		# bot4_pose.header.stamp = rospy.Time.now()
		bot4_pose.header.frame_id = "base_footprint"
		bot4_pose.header.seq = i
		bot4_pose.pose.position.x = return_path_ls[i][0]
		bot4_pose.pose.position.y = return_path_ls[i][1]
		bot4_pose.pose.position.z = 0

		yaw = math.atan2(re_vec_y_4[i], re_vec_x_4[i])
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
		bot4_pose.pose.orientation.x = quaternion[0]
		bot4_pose.pose.orientation.y = quaternion[1]
		bot4_pose.pose.orientation.z = quaternion[2]
		bot4_pose.pose.orientation.w = quaternion[3]

		bot4_path.poses.append(bot4_pose)

	print("bot4 should move from goal to start now\n")
	bot4_traj_pub.publish(bot4_path)
	bot4_goal_pub.publish(bot4_pose)

	# print("Waiting for bot4 to reach start...")
	# while bot4_return_flag == 0:
	# 	rate.sleep()
	# print("Confirmation received. OVER")

	print("Motion planning execution complete. Now spinning.")

	# plot_anim(R1_obstacles_x_1, R1_obstacles_y_1, start, goal, return_planned_paths,offset,skip)
	#print(planned_paths[0])



if __name__=='__main__' :
	# global M
	print("-----Motion Planner node started-----\n")
	rospy.init_node('mp_map_subscriber')
	mp_map_sub = rospy.Subscriber('/costmap_node/costmap/costmap', OccupancyGrid, callback)  # Create a Subscriber object that will listen to the given topic and will call the "callback" function each time it reads something from the topic
	
	bot1_goal_sub = rospy.Subscriber('/bot1/goal_feedback', Bool, bot1_goal_feedback)
	bot1_return_sub = rospy.Subscriber('/bot1/return_feedback', Bool, bot1_return_feedback)
	bot2_goal_sub = rospy.Subscriber('/bot2/goal_feedback', Bool, bot2_goal_feedback)
	bot2_return_sub = rospy.Subscriber('/bot2/return_feedback', Bool, bot2_return_feedback)
	bot3_goal_sub = rospy.Subscriber('/bot3/goal_feedback', Bool, bot3_goal_feedback)
	bot3_return_sub = rospy.Subscriber('/bot3/return_feedback', Bool, bot3_return_feedback)
	bot4_goal_sub = rospy.Subscriber('/bot4/goal_feedback', Bool, bot4_goal_feedback)

	bot1_traj_pub = rospy.Publisher('/bot1/trajectory_publisher', Path, queue_size=10)
	bot2_traj_pub = rospy.Publisher('/bot2/trajectory_publisher', Path, queue_size=10)
	bot3_traj_pub = rospy.Publisher('/bot3/trajectory_publisher', Path, queue_size=10)
	bot4_traj_pub = rospy.Publisher('/bot4/trajectory_publisher', Path, queue_size=10)

	bot1_goal_pub = rospy.Publisher('/bot1/cmd_goal', PoseStamped, queue_size=2)
	bot2_goal_pub = rospy.Publisher('/bot2/cmd_goal', PoseStamped, queue_size=2)
	bot3_goal_pub = rospy.Publisher('/bot3/cmd_goal', PoseStamped, queue_size=2)
	bot4_goal_pub = rospy.Publisher('/bot4/cmd_goal', PoseStamped, queue_size=2)

	rospy.spin()  # Create a loop that will keep the program in execution
	