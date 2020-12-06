#!/usr/bin/env python

import rospy
import math as m
from hri_control.msg import joint_angle
from geometry_msgs.msg import Pose
import geometry_msgs
from gazebo_msgs.srv import GetModelState, GetLinkState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class algo:

    def __init__(self):
        rospy.init_node('estPos', anonymous=True)
        # self.angle = rospy.Publisher('/desired_angle_right',joint_angle ,queue_size=1)
        self.can_name = ['coke_can', 'coke_can_clone_1']
        self.can_pos = []
        self.state = 'find'
        self.last_state = 'find'
        self.pub_cam = rospy.Publisher('/camArmPoseGoal', Pose, queue_size=1)
        self.pm_x = 0
        self.pm_y = 0
        self.d_r = 0
        self.beta = 0
        self.L = 0
        self.d_i = 0
        self.cam_pose = Pose()
        self.start_x = 0
        self.last_Ls = 0
        self.Ls = 0
        rospy.Subscriber('/currentCamArmPose', Pose, self.update_pose, queue_size=1)
        robot_prefix = 'trina2_1'
        rospy.Subscriber(robot_prefix + '/right_arm_cam/color/image_raw', Image, self.right_image, queue_size=1)
        self.bridge = CvBridge()

    def right_image(self,data):
        self.right_cv_image = self.bridge.imgmsg_to_cv2(data,'bgr8')

    def get_beta_L_di_pm(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            self.can_pos = []
            for i in self.can_name:
                result = model_coordinates(i, '')
                self.can_pos.append([result.pose.position.x, result.pose.position.y, result.pose.position.z])

        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))
        x_distance = self.can_pos[0][0]-self.can_pos[1][0]
        y_distance = self.can_pos[0][1]-self.can_pos[1][1]
        self.pm_x = (self.can_pos[0][0]+self.can_pos[1][0])/2
        self.pm_y = (self.can_pos[0][1] + self.can_pos[1][1]) / 2
        self.L = m.sqrt(x_distance*x_distance+y_distance*y_distance)

        self.beta = m.atan2(abs(self.can_pos[0][0]-self.can_pos[1][0]),abs(self.can_pos[0][1]-self.can_pos[1][1]))
        self.d_i = abs((self.pm_y-self.right_ee_post[1]))
        self.start_x = self.pm_x
        rospy.loginfo('beta: '+str(self.beta))
        rospy.loginfo('L: ' + str(self.L))
        rospy.loginfo('Di: '+str(self.d_i))


    def get_right_ee_pos(self):
        rospy.loginfo('get right ee pos')
        ee_pos = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        right_cam = ee_pos('trina2_1/right_arm_bracelet_link', '')
        right_cam_x = right_cam.link_state.pose.position.x
        right_cam_y = right_cam.link_state.pose.position.y
        right_cam_z = right_cam.link_state.pose.position.z

        self.right_ee_post = [right_cam_x, right_cam_y, right_cam_z]


    def update_pose(self,p):
        self.cam_pose = p



    def get_dr_alpha(self):
        self.get_right_ee_pos()
        x_distance = self.pm_x - self.right_ee_post[0]
        y_distance = self.pm_y - self.right_ee_post[1]
        self.d_r = m.sqrt(x_distance*x_distance+y_distance*y_distance)
        delta_x = self.right_ee_post[0]-self.start_x
        self.turn = m.atan2(abs(delta_x),abs(self.d_i))
        self.alpha = m.atan2(abs(delta_x),abs(self.d_i))+self.beta
        rospy.loginfo('dr: '+str(self.d_r))
        rospy.loginfo('alpha: ' + str(self.alpha))


    def cal_Ls(self):
        Lv = self.L * m.sin(self.alpha)*m.cos(self.alpha-self.beta)
        D = 2*self.d_i-self.L*m.cos(self.alpha)*m.cos(self.alpha-self.beta)
        self.Ls = Lv/D

    def move_robot(self):
        self.get_dr_alpha()
        self.last_Ls = self.Ls
        self.cal_Ls()
        newPose = self.cam_pose
        rospy.loginfo("y:" + str(newPose.position.y))
        if abs(self.Ls-self.last_Ls)>0.001:
            cv2.imwrite('algo_test/ls'+str(self.Ls)+'.jpg',self.right_cv_image)
        yaw= 1.57-self.turn
        rospy.loginfo("yaw:" + str(yaw))
        # if Starting Pose 1
        #arr = quaternion_from_euler(1.57, 0, yaw)
        # if Starting Pose 2
        arr = quaternion_from_euler(2.1, 0, yaw)
        ox = arr[0]
        oy = arr[1]
        oz = arr[2]
        ow = arr[3]
        newPose.orientation.w = ow
        newPose.orientation.x = ox
        newPose.orientation.y = oy
        newPose.orientation.z = oz


        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = ow
        pose_goal.orientation.x = ox
        pose_goal.orientation.y = oy
        pose_goal.orientation.z = oz
        pose_goal.position.x = newPose.position.x
        pose_goal.position.y = newPose.position.y+0.05
        pose_goal.position.z = newPose.position.z
        self.pub_cam.publish(pose_goal)



    def run(self):

        rospy.spin()

    def home_robot(self):
        newPose = Pose()
        # Starting Pose 1
        # x =0.6
        # y = -0.1
        # z=0.8
        # r=1.57
        # p = 0
        # yaw = 1.57

        # Starting Pose 2
        x = -0.05
        y = 0
        z = 1.37
        r = 2.1
        p = 0
        yaw = 1.57


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


if __name__ == '__main__':
    algo = algo()
    comm_rate = 10
    rate = rospy.Rate(comm_rate)
    t_end = time.time()+20
    while time.time()<t_end:
        algo.home_robot()
    rospy.loginfo('start')
    algo.get_right_ee_pos()
    algo.get_beta_L_di_pm()
    algo.get_dr_alpha()

    while not rospy.is_shutdown():
        if int(time.time())%5 == 0:
            algo.move_robot()
