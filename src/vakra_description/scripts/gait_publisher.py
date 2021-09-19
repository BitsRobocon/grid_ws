#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

def callback(data):
	msg = data.points[0].positions
	#print((msg[4]))
	pub0 = rospy.Publisher('/vakra/lfs_position_controller/command', Float64, queue_size=10)
	pub1 = rospy.Publisher('/vakra/lft_position_controller/command', Float64, queue_size=10)
	pub2 = rospy.Publisher('/vakra/lff_position_controller/command', Float64, queue_size=10)
	pub3 = rospy.Publisher('/vakra/rfs_position_controller/command', Float64, queue_size=10)
	pub4 = rospy.Publisher('/vakra/rft_position_controller/command', Float64, queue_size=10)
	pub5 = rospy.Publisher('/vakra/rff_position_controller/command', Float64, queue_size=10)
	pub6 = rospy.Publisher('/vakra/lbs_position_controller/command', Float64, queue_size=10)
	pub7 = rospy.Publisher('/vakra/lbt_position_controller/command', Float64, queue_size=10)
	pub8 = rospy.Publisher('/vakra/lbf_position_controller/command', Float64, queue_size=10)
	pub9 = rospy.Publisher('/vakra/rbs_position_controller/command', Float64, queue_size=10)
	pub10 = rospy.Publisher('/vakra/rbt_position_controller/command', Float64, queue_size=10)
	pub11 = rospy.Publisher('/vakra/rbf_position_controller/command', Float64, queue_size=10)

	rate = rospy.Rate(1) # 100hz
	pub0.publish(-msg[0]-0.272)
	pub1.publish(msg[1]-1.5708)
	pub2.publish(-msg[2]-1.047)
	pub3.publish(-msg[3]+0.272)
	pub4.publish(msg[4]-1.5708)
	pub5.publish(msg[5]+1.047)
	pub6.publish(-msg[6]-0.272)
	pub7.publish(msg[7]-1.5708)
	pub8.publish(-msg[8]-1.047)
	pub9.publish(-msg[9]+0.272)
	pub10.publish(msg[10]-1.5708)
	pub11.publish(msg[11]+1.047)
	#rate.sleep()


def vakra_listner():
	rospy.init_node('vakra_sub', anonymous=True)
	rospy.Subscriber("/joint_group_position_controller/command", JointTrajectory, callback)
	rospy.spin()
		

if __name__ == '__main__':
	try:
		while True:
			vakra_listner()
	except KeyboardInterrupt:
			print("exiting")