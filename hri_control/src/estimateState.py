#!/usr/bin/env python

import rospy
import math as m
from hri_control.msg import joint_angle

from gazebo_msgs.srv import GetModelState, GetLinkState

class estPos:

	def __init__(self):
		rospy.init_node('estPos',anonymous=True)
		#self.angle = rospy.Publisher('/desired_angle_right',joint_angle ,queue_size=1)
		self.can_name = ['coke_can','coke_can_clone','coke_can_clone_0','coke_can_clone_1','coke_can_clone_2',
						 'coke_can_clone_3']
		self.can_pos = []
		self.left_ee_post = []


	def get_left_ee_pos(self):
		ee_pos = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		result = ee_pos('trina2_1/left_arm_left_inner_finger','')
		ee_x = result.link_state.pose.position.x
		ee_y = result.link_state.pose.position.y
		ee_z = result.link_state.pose.position.z
		# global_x = 0.4 - ee_y
		# global_y = -0.4 + ee_x
		# global_z = ee_z
		#rospy.loginfo("ex: "+str(ee_x)+"  ey: "+str(ee_y)+"  ez: "+str(ee_z))
		#rospy.loginfo("x: "+str(global_x)+"  y: "+str(global_y)+"  z: "+str(global_z))
		self.left_ee_post = [ee_x,ee_y,ee_z]

	def cal_distance(self):
		distance = 100
		for i in range(0,6):
			d_x = self.left_ee_post[0] - self.can_pos[i][0]
			d_y = self.left_ee_post[1] - self.can_pos[i][1]
			d_z = self.left_ee_post[2] - self.can_pos[i][2]
			distance = min(distance,m.sqrt(d_x*d_x+d_y*d_y+d_z*d_z))
		rospy.loginfo("Shortest distance = " + str(distance))


	
	def print_all_pos(self):
		try:
			model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			self.can_pos = []
			for i in self.can_name:
				result = model_coordinates(i,'')
				self.can_pos.append([result.pose.position.x,result.pose.position.y,result.pose.position.z])

		except rospy.ServiceException as e:
			rospy.loginfo("Get Model State service call failed:  {0}".format(e))

	def run(self):
		print "Node running"
		rospy.spin()


if __name__ == '__main__':
	estPos = estPos()
	comm_rate = 10
	rate = rospy.Rate(comm_rate)
	while not rospy.is_shutdown():
		estPos.print_all_pos()
		estPos.get_left_ee_pos()
		estPos.cal_distance()
		rate.sleep()
