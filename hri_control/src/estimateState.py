#!/usr/bin/env python

import rospy
import math as m
from hri_control.msg import joint_angle
from geometry_msgs.msg import Pose
import geometry_msgs
from gazebo_msgs.srv import GetModelState, GetLinkState
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class estPos:

    def __init__(self):
        rospy.init_node('estPos', anonymous=True)
        # self.angle = rospy.Publisher('/desired_angle_right',joint_angle ,queue_size=1)
        self.can_name = ['coke_can', 'coke_can_clone', 'coke_can_clone_0', 'coke_can_clone_1', 'coke_can_clone_2',
                         'coke_can_clone_3']
        self.can_pos = []
        self.left_ee_post = []
        self.right_ee_post = []
        self.state = 'find'
        self.last_state = 'find'
        self.pub_cam = rospy.Publisher('/camArmPoseGoal', Pose, queue_size=1)


    def get_left_ee_pos(self):
        ee_pos = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        left_finger = ee_pos('trina2_1/left_arm_left_inner_finger', '')
        left_finger_x = left_finger.link_state.pose.position.x
        left_finger_y = left_finger.link_state.pose.position.y
        left_finger_z = left_finger.link_state.pose.position.z

        right_finger = ee_pos('trina2_1/left_arm_right_inner_finger', '')
        right_finger_x = right_finger.link_state.pose.position.x
        right_finger_y = right_finger.link_state.pose.position.y
        right_finger_z = right_finger.link_state.pose.position.z

        self.left_ee_post = [left_finger_x, left_finger_y, left_finger_z]
        self.right_ee_post = [right_finger_x, right_finger_y, right_finger_z]

    def cal_distance(self):
        distance = 100
        ee_mid_x = (self.left_ee_post[0] + self.right_ee_post[0]) / 2.
        ee_mid_y = (self.left_ee_post[1] + self.right_ee_post[1]) / 2.
        ee_mid_z = (self.left_ee_post[2] + self.right_ee_post[2]) / 2.
        for i in range(0, 6):
            d_x = ee_mid_x - self.can_pos[i][0]
            d_y = ee_mid_y - self.can_pos[i][1]
            d_z = ee_mid_z - self.can_pos[i][2]
            distance = min(distance, m.sqrt(d_x * d_x + d_y * d_y + d_z * d_z))

        d_f = abs(self.left_ee_post[1] - self.right_ee_post[1])
        if distance < 0.1 and d_f <= 0.08 and ee_mid_z > 0.9:
            self.state = 'stack'
        elif distance < 0.1 and d_f > 0.08:
            self.state = 'grab'
        else:
            self.state = 'find'

    def print_all_pos(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            self.can_pos = []
            for i in self.can_name:
                result = model_coordinates(i, '')
                self.can_pos.append([result.pose.position.x, result.pose.position.y, result.pose.position.z])

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

    def run(self):

        rospy.spin()

    def pub_cam_pose(self):
        newPose = Pose()
        self.state = 'grab'
        if self.state == 'find':
            x = -0.05
            y = 0
            z = 1.37
            r = 2.1
            p = 0
            yaw = 1.57
        elif self.state == 'grab':
            x = 0.5
            y = 0
            z = 1.36
            r = 3
            p = 0.1
            yaw = 1.57
        else:
            x = 0.1
            y = -0.35
            z = 1.25
            r = 2.1
            p = 0
            yaw = 2.1


        arr = quaternion_from_euler(r, p, yaw)
        ox = arr[0]
        oy = arr[1]
        oz = arr[2]
        ow = arr[3]

        newPose.orientation.w = ow
        newPose.orientation.x = ox
        newPose.orientation.y = oy
        newPose.orientation.z = oz


        newPose.position.x = x
        newPose.position.y = y
        newPose.position.z = z
        self.pub_cam.publish(newPose)
        rospy.loginfo("pub: " + self.state)


if __name__ == '__main__':
    estPos = estPos()
    comm_rate = 10
    rate = rospy.Rate(comm_rate)

    while not rospy.is_shutdown():
        estPos.pub_cam_pose()
        estPos.print_all_pos()
        estPos.get_left_ee_pos()
        estPos.cal_distance()
        if estPos.state != estPos.last_state:
            estPos.pub_cam_pose()
            estPos.last_state = estPos.state
        rate.sleep()
