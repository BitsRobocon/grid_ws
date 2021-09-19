#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
def vakra_publisher():
	rospy.init_node('vakra_pub', anonymous=True)
	pub = rospy.Publisher('/vakra/rft_position_controller/command', Float64, queue_size=10)
	rate = rospy.Rate(1) # 10hz
	while not rospy.is_shutdown():
		# hello_str = "hello world"
		# rospy.get_time()
		# rospy.loginfo(hello_str)
		for i in range(10):
			pub.publish(0.5)
			rate.sleep()
			pub.publish(-0.5)
			rate.sleep()
		
if __name__ == '__main__':
	try:
		vakra_publisher()
	except rospy.ROSInterruptException:
		pass
