#!/usr/bin/env python

import rospy
from hri_control.msg import joint_angle
from gazebo_msgs.srv import GetModelState

class estPos:

	def __init__(self):
		rospy.init_node('estPos',anonymous=True)
		self.angle = rospy.Publisher('/desired_angle_right',joint_angle ,queue_size=1)
		self.can_name = ['coke_can','coke_can_clone','coke_can_clone_0','coke_can_clone_1','coke_can_clone_2',
						 'coke_can_clone_3']

	
	def print_all_pos(self):
		try:
			model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			for i in self.can_name:
				result = model_coordinates(i,'')
				print(i+" x:"+str(result.pose.position.x)+" y:"+str(result.pose.position.y)+" z:"+str(result.pose.position.z))

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
		rate.sleep()
