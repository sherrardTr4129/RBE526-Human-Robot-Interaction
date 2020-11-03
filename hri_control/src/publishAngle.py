#!/usr/bin/env python

import rospy
from kortex_driver.msg import *
from std_msgs.msg import Float32

class Pub:

	def __init__(self):
		rospy.init_node('pub',anonymous=True)
		self.angle = rospy.Publisher('/joint_angle',Float32 ,queue_size=10)
		self.rate =rospy.Rate(10)

	
	def publishAngle(self,angle):
		self.angle.publish(Float32(angle))


	def run(self):
		print "Node running"
		rospy.spin()


if __name__ == '__main__':
	anglePub = Pub()
	comm_rate = 10
	rate = rospy.Rate(comm_rate)
	angle = 10
	while not rospy.is_shutdown():
		angle+=0.2
		anglePub.publishAngle(angle)
		rate.sleep()
